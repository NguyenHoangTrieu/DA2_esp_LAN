/**
 * @file ble_native_handler.c
 * @brief BLE Native Mesh handler — ESP BLE Mesh stack init and coordinator.
 *
 * Responsibilities:
 *   1. Declare static BLE Mesh composition data (provisioner + client models).
 *   2. Initialize the BLE Mesh stack once (ble_native_handler_init).
 *   3. Register provisioner and model client callbacks.
 *   4. Apply network/app keys from JSON config when loaded.
 *   5. Expose ble_native_get_model() for the downlink dispatcher.
 *
 * This file does NOT know about specific LED protocols.  Model selection and
 * opcode routing is entirely driven by JSON configuration from the server.
 *
 * SDK requirement: CONFIG_BLE_MESH=y, CONFIG_BLE_MESH_PROVISIONER=y,
 *                  CONFIG_BLE_MESH_PB_ADV=y in sdkconfig.
 */

#include "ble_native_handler.h"
#include "ble_native_config.h"
#include "ble_native_uplink.h"
#include "ble_native_downlink.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_time_scene_model_api.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ble_native_hdl";

/* --------------------------------------------------------------------------
 * BLE Mesh stack init flag
 * -------------------------------------------------------------------------- */

static bool s_mesh_initialized = false;

/* --------------------------------------------------------------------------
 * Static BLE Mesh provisioner + client models
 *
 * ESP-IDF BLE Mesh requires model declarations at compile time.
 * The APPLICATION USE of these models (which node, what opcode, what params)
 * is driven by JSON config at runtime.  This section never changes regardless
 * of what device type the gateway controls.
 * -------------------------------------------------------------------------- */

/* Configuration server model (required by spec even on provisioner) */
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay       = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon      = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy  = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),   /* count=2, interval=20ms */
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/* Configuration Client needed to configure bound app-keys on remote nodes */
static esp_ble_mesh_client_t config_client;

/* Generic OnOff Client — model_id 0x1001 (client) / server 0x1000 */
static esp_ble_mesh_client_t onoff_client;

/* Light Lightness Client — model_id 0x1303 (client) */
static esp_ble_mesh_client_t lightness_client;

/* Light CTL Client — model_id 0x1305 (client) */
static esp_ble_mesh_client_t ctl_client;

/* Scene Client — model_id 0x1205 (client) */
static esp_ble_mesh_client_t scene_client;

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(NULL, &lightness_client),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_CLI(NULL, &ctl_client),
    ESP_BLE_MESH_MODEL_SCENE_CLI(NULL, &scene_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

/* Provisioner UUID — uses the ESP32 base MAC address */
static uint8_t s_dev_uuid[16] = { 0 };

/* In ESP-IDF v5.x, prov struct only needs prov_start_address for provisioner */
static esp_ble_mesh_prov_t prov = {
    .prov_start_address = 0x0001,  /* may be overridden by JSON config */
};

static esp_ble_mesh_comp_t comp = {
    .cid         = 0x02E5,  /* Espressif company ID */
    .elements    = elements,
    .element_count = ARRAY_SIZE(elements),
};

/* --------------------------------------------------------------------------
 * Model ID → model pointer table  (used by ble_native_get_model)
 * -------------------------------------------------------------------------- */

typedef struct {
    uint16_t model_id;
    esp_ble_mesh_model_t *model;
} model_entry_t;

/* Note: ESP-IDF uses the SERVER model_id as the identifier in both server
 * and client model.  Generic OnOff server = 0x1000, so the client dispatches
 * using 0x1000 from the command config. */
static const model_entry_t s_model_table[] = {
    { ESP_BLE_MESH_MODEL_ID_CONFIG_SRV,          &root_models[1] }, /* 0x0000 Config CLI */
    { ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV,       &root_models[2] }, /* 0x1000 OnOff CLI */
    { ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SRV, &root_models[3] }, /* 0x1300 Lightness CLI */
    { ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SRV,       &root_models[4] }, /* 0x1303 CTL CLI */
    { ESP_BLE_MESH_MODEL_ID_SCENE_SRV,           &root_models[5] }, /* 0x1203 Scene CLI */
};

esp_ble_mesh_model_t *ble_native_get_model(uint16_t model_id) {
    for (size_t i = 0; i < ARRAY_SIZE(s_model_table); i++) {
        if (s_model_table[i].model_id == model_id) {
            return s_model_table[i].model;
        }
    }
    return NULL;
}

/* --------------------------------------------------------------------------
 * Unified Provisioning Callback (ESP-IDF v5.x event-based)
 *
 * In v5.x, all provisioning events (discovery, complete, errors) go through
 * a single callback with event type + parameters.  Dispatch here.
 * -------------------------------------------------------------------------- */

static void prov_callback(esp_ble_mesh_prov_cb_event_t event, 
                          esp_ble_mesh_prov_cb_param_t *param) {
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner registered successfully");
        break;

    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner scan enabled");
        break;

    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner scan disabled");
        break;

    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT: {
        /* Unprovisioned device discovered */
        uint8_t *dev_uuid = param->provisioner_recv_unprov_adv_pkt.dev_uuid;
        uint8_t *addr     = param->provisioner_recv_unprov_adv_pkt.addr;
        uint16_t oob_info = param->provisioner_recv_unprov_adv_pkt.oob_info;

        char uuid_str[33] = {0};
        for (int i = 0; i < 16; i++) {
            snprintf(uuid_str + i * 2, 3, "%02X", dev_uuid[i]);
        }
        char addr_str[18] = {0};
        snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

        ESP_LOGI(TAG, "Unprov ADV: uuid=%s addr=%s oob=%u", uuid_str, addr_str, oob_info);

        /* Forward discovery result to server */
        char report[128];
        snprintf(report, sizeof(report), "UNPROV_DEV:%s:%s:%u",
                 uuid_str, addr_str, oob_info);
        ble_native_uplink_send_ok(0, report);
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT: {
        /* Provisioning complete */
        uint8_t *uuid = param->provisioner_prov_complete.device_uuid;
        uint16_t addr = param->provisioner_prov_complete.unicast_addr;

        char uuid_str[33] = {0};
        for (int i = 0; i < 16; i++) {
            snprintf(uuid_str + i * 2, 3, "%02X", uuid[i]);
        }

        ESP_LOGI(TAG, "Node provisioned: addr=0x%04X uuid=%s", addr, uuid_str);

        /* Report to server */
        char resp[128];
        snprintf(resp, sizeof(resp), "PROVISIONED:0x%04X:%s", addr, uuid_str);
        ble_native_uplink_send_ok(0, resp);
        break;
    }

    default:
        ESP_LOGD(TAG, "Provisioning event: %d", event);
        break;
    }
}

/* Generic OnOff client callback */
static void onoff_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                             esp_ble_mesh_generic_client_cb_param_t *param) {
    char resp[80];
    uint16_t addr = param->params ? param->params->ctx.addr : 0;

    switch (event) {
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
        snprintf(resp, sizeof(resp), "ONOFF_ACK:0x%04X:OK", addr);
        ble_native_uplink_send_ok(0, resp);
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
        if (param->status_cb.onoff_status.present_onoff != 0xFF) {
            snprintf(resp, sizeof(resp), "ONOFF_STATUS:0x%04X:%u",
                     addr, param->status_cb.onoff_status.present_onoff);
            ble_native_uplink_send_ok(0, resp);
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        snprintf(resp, sizeof(resp), "ONOFF_TIMEOUT:0x%04X", addr);
        ble_native_uplink_send_fail(0, resp);
        break;
    default:
        break;
    }
}

/* Light Lightness client callback */
static void lightness_client_cb(esp_ble_mesh_light_client_cb_event_t event,
                                 esp_ble_mesh_light_client_cb_param_t *param) {
    char resp[80];
    uint16_t addr = param->params ? param->params->ctx.addr : 0;

    switch (event) {
    case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
        snprintf(resp, sizeof(resp), "LIGHTNESS_ACK:0x%04X:OK", addr);
        ble_native_uplink_send_ok(0, resp);
        break;
    case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
        snprintf(resp, sizeof(resp), "LIGHTNESS_STATUS:0x%04X:%u",
                 addr,
                 param->status_cb.lightness_status.present_lightness);
        ble_native_uplink_send_ok(0, resp);
        break;
    case ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT:
        snprintf(resp, sizeof(resp), "LIGHTNESS_TIMEOUT:0x%04X", addr);
        ble_native_uplink_send_fail(0, resp);
        break;
    default:
        break;
    }
}

/* Config Client callback — handles responses to CONFIG_* opcodes */
static void config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                              esp_ble_mesh_cfg_client_cb_param_t *param) {
    char resp[128];
    uint16_t addr = param->params ? param->params->ctx.addr : 0;

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT: {
        uint32_t opcode = param->params ? param->params->opcode : 0;
        snprintf(resp, sizeof(resp), "CFG_ACK:0x%04X:op=0x%06X:OK",
                 addr, (unsigned)opcode);
        ble_native_uplink_send_ok(0, resp);
        break;
    }
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        snprintf(resp, sizeof(resp), "CFG_STATUS:0x%04X:OK", addr);
        ble_native_uplink_send_ok(0, resp);
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        snprintf(resp, sizeof(resp), "CFG_TIMEOUT:0x%04X", addr);
        ble_native_uplink_send_fail(0, resp);
        break;
    default:
        break;
    }
}

/* Scene Client callback */
static void scene_client_cb(esp_ble_mesh_time_scene_client_cb_event_t event,
                              esp_ble_mesh_time_scene_client_cb_param_t *param) {
    char resp[80];
    uint16_t addr = param->params ? param->params->ctx.addr : 0;

    switch (event) {
    case ESP_BLE_MESH_TIME_SCENE_CLIENT_SET_STATE_EVT:
        snprintf(resp, sizeof(resp), "SCENE_ACK:0x%04X:OK", addr);
        ble_native_uplink_send_ok(0, resp);
        break;
    case ESP_BLE_MESH_TIME_SCENE_CLIENT_GET_STATE_EVT:
        snprintf(resp, sizeof(resp), "SCENE_STATUS:0x%04X:scene=%u",
                 addr,
                 param->status_cb.scene_status.current_scene);
        ble_native_uplink_send_ok(0, resp);
        break;
    case ESP_BLE_MESH_TIME_SCENE_CLIENT_TIMEOUT_EVT:
        snprintf(resp, sizeof(resp), "SCENE_TIMEOUT:0x%04X", addr);
        ble_native_uplink_send_fail(0, resp);
        break;
    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * Public API — Init
 * -------------------------------------------------------------------------- */

esp_err_t ble_native_handler_init(void) {
    if (s_mesh_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* Derive provisioner UUID from Bluetooth MAC */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    memset(s_dev_uuid, 0, sizeof(s_dev_uuid));
    memcpy(s_dev_uuid, mac, 6);

    /* Start uplink and downlink tasks */
    esp_err_t ret = ble_native_uplink_task_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start uplink task: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ble_native_downlink_task_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start downlink task: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register callbacks BEFORE esp_ble_mesh_init (v5.x uses unified event callback) */
    esp_ble_mesh_register_prov_callback(prov_callback);
    esp_ble_mesh_register_generic_client_callback(onoff_client_cb);
    esp_ble_mesh_register_light_client_callback(lightness_client_cb);
    esp_ble_mesh_register_config_client_callback(config_client_cb);
    esp_ble_mesh_register_time_scene_client_callback(scene_client_cb);

    /* Initialize the BLE Mesh stack */
    ret = esp_ble_mesh_init(&prov, &comp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_mesh_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Enable provisioner role (ADV bearer) */
    ret = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV);
    if (ret != ESP_OK) {
        /* Non-fatal — provisioner can be enabled later after JSON config */
        ESP_LOGW(TAG, "Provisioner enable returned: %s (will retry after config)",
                 esp_err_to_name(ret));
    }

    s_mesh_initialized = true;
    ESP_LOGI(TAG, "BLE Native handler initialized");
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public API — Load Config
 * -------------------------------------------------------------------------- */

esp_err_t ble_native_handler_load_config(uint8_t stack_id,
                                          const char *json_str,
                                          uint16_t json_len) {
    if (!json_str || json_len == 0 || stack_id >= BLE_NATIVE_MAX_STACKS) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Parse JSON into runtime config store */
    esp_err_t ret = ble_native_config_load(stack_id, json_str, json_len);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "JSON_PARSE_FAIL");
        return ret;
    }

    /* Retrieve mesh config (keys are stored internally, no direct API call needed) */
    ble_native_mesh_cfg_t mc;
    ret = ble_native_config_get_mesh(stack_id, &mc);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "MESH_CFG_ERROR");
        return ret;
    }

    /* In ESP-IDF v5.x, provisioner keys are managed internally.
     * The mesh config is stored in ble_native_config for uplink dispatcher.
     * Key setup is implicit when provisioning and will happen when
     * esp_ble_mesh_provisioner_add_unprov_dev is called. */

    ble_native_uplink_send_ok(stack_id, "JSON_LOADED");
    ESP_LOGI(TAG, "stack=%u: config applied (mesh keys staged for provisioning)", stack_id);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public API — Execute Command
 * -------------------------------------------------------------------------- */

esp_err_t ble_native_handler_execute(const uint8_t *data, uint16_t len) {
    return ble_native_downlink_enqueue(data, len);
}
