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
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
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
 * Provision-complete synchronisation
 *
 * handle_provision() in ble_native_downlink.c calls ble_native_start_provision_wait()
 * before issuing add_unprov_dev, then blocks on ble_native_wait_provision_complete().
 * PROVISIONER_PROV_COMPLETE_EVT gives the semaphore and stores the assigned address.
 * -------------------------------------------------------------------------- */

static SemaphoreHandle_t s_prov_sem   = NULL;
static volatile uint16_t s_prov_addr  = 0;

esp_err_t ble_native_start_provision_wait(void) {
    if (!s_prov_sem) {
        s_prov_sem = xSemaphoreCreateBinary();
        if (!s_prov_sem) return ESP_ERR_NO_MEM;
    }
    /* Drain any stale give from a previous provisioning */
    xSemaphoreTake(s_prov_sem, 0);
    s_prov_addr = 0;
    return ESP_OK;
}

esp_err_t ble_native_wait_provision_complete(uint16_t *addr_out, uint32_t timeout_ms) {
    if (!s_prov_sem || !addr_out) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_prov_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *addr_out = s_prov_addr;
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/* --------------------------------------------------------------------------
 * Scan accumulation buffer
 * Unprov ADV packets received during a scan are accumulated here instead of
 * being sent individually over SPI.  ble_native_scan_flush() sends them all
 * as a single batched uplink once the scan timer expires.
 * -------------------------------------------------------------------------- */
#define BLE_NATIVE_MAX_SCAN_DEVICES  32

typedef struct {
    char     uuid_str[33];  /* 32-char hex + NUL */
    char     addr_str[18];  /* xx:xx:xx:xx:xx:xx + NUL */
    uint16_t oob_info;
    uint8_t  uuid_raw[16];  /* raw bytes for dedup check */
    uint8_t  addr_raw[6];   /* raw BT address bytes (for add_unprov_dev) */
    uint8_t  addr_type;     /* 0=public, 1=random */
} native_scan_dev_t;

static native_scan_dev_t s_scan_buf[BLE_NATIVE_MAX_SCAN_DEVICES];
static volatile uint8_t  s_scan_count    = 0;
static uint8_t           s_scan_stack_id = 0;

void ble_native_scan_reset(uint8_t stack_id) {
    s_scan_count    = 0;
    s_scan_stack_id = stack_id;
    memset(s_scan_buf, 0, sizeof(s_scan_buf));
    ESP_LOGI(TAG, "Scan accumulation buffer reset (stack=%u)", stack_id);
}

void ble_native_scan_flush(void) {
    /* Build one batched message: SCAN_DONE:<n>\x1EUNPROV_DEV:...\x1E... */
    size_t buf_size = 64 + (size_t)s_scan_count * 96;
    char *buf = malloc(buf_size);
    if (!buf) {
        /* Fallback: just send the count with no device list */
        char small[32];
        snprintf(small, sizeof(small), "SCAN_DONE:%u", s_scan_count);
        ble_native_uplink_send_ok(s_scan_stack_id, small);
        s_scan_count = 0;
        return;
    }
    int pos = snprintf(buf, buf_size, "SCAN_DONE:%u", s_scan_count);
    for (uint8_t i = 0; i < s_scan_count; i++) {
        native_scan_dev_t *d = &s_scan_buf[i];
        /* Format: UNPROV_DEV:<uuid32><addr_type02X><addr12hex>:<oob>
         * Compact: uuid(32) + addr_type(2) + addr(12) = 46 chars, no inner separators.
         * Widget extracts uuid=[:32], addr_type=[32:34], addr=[34:46].         */
        pos += snprintf(buf + pos, buf_size - (size_t)pos,
                        "\x1EUNPROV_DEV:%s%02X%02X%02X%02X%02X%02X%02X:%u",
                        d->uuid_str,
                        d->addr_type,
                        d->addr_raw[0], d->addr_raw[1], d->addr_raw[2],
                        d->addr_raw[3], d->addr_raw[4], d->addr_raw[5],
                        d->oob_info);
    }
    ble_native_uplink_send_ok(s_scan_stack_id, buf);
    free(buf);
    ESP_LOGI(TAG, "Scan flush: sent %u device(s) in one batch", s_scan_count);
    s_scan_count = 0;
}

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

/* prov_unicast_addr: provisioner's OWN element address (must be non-zero).
 * prov_start_address: unicast address assigned to the FIRST provisioned node.
 * Both are set once at init — cannot change after esp_ble_mesh_init(). */
static esp_ble_mesh_prov_t prov = {
    .prov_unicast_addr  = 0x0001,  /* provisioner's own element address */
    .prov_start_address = 0x0002,  /* first provisioned node gets 0x0002 */
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
        if (param->prov_register_comp.err_code != 0) {
            ESP_LOGE(TAG, "BLE Mesh register FAILED: err=%d",
                     param->prov_register_comp.err_code);
        } else {
            ESP_LOGI(TAG, "BLE Mesh provisioner stack ready");
            /* Enable provisioner once immediately so the Zephyr mesh stack
             * assigns elem->addr = prov_unicast_addr for each element.
             * Without this, esp_ble_mesh_provisioner_bind_app_key_to_local_model
             * returns -ENODEV because bt_mesh_elem_find() finds no element. */
            esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV);
        }
        break;

    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner scan enabled");
        break;

    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner scan disabled");
        break;

    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "Provisioner PB-ADV link OPENED with node (bearer=%u)",
                 param->provisioner_prov_link_open.bearer);
        break;

    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "Provisioner PB-ADV link CLOSED (bearer=%u reason=%u)",
                 param->provisioner_prov_link_close.bearer,
                 param->provisioner_prov_link_close.reason);
        break;

    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_NET_KEY_COMP_EVT:
        if (param->provisioner_add_net_key_comp.err_code != 0) {
            ESP_LOGE(TAG, "Add net_key FAILED: err=%d net_idx=%u",
                     param->provisioner_add_net_key_comp.err_code,
                     param->provisioner_add_net_key_comp.net_idx);
        } else {
            ESP_LOGI(TAG, "Net key added: net_idx=%u",
                     param->provisioner_add_net_key_comp.net_idx);
        }
        break;

    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT: {
        uint16_t app_idx = param->provisioner_add_app_key_comp.app_idx;
        if (param->provisioner_add_app_key_comp.err_code != 0) {
            ESP_LOGE(TAG, "Add app_key FAILED: err=%d app_idx=%u",
                     param->provisioner_add_app_key_comp.err_code, app_idx);
            break;
        }
        ESP_LOGI(TAG, "App key added: app_idx=%u — binding to client models", app_idx);
        /* Bind the app key to every client model on this provisioner element.
         * element_addr = prov.prov_unicast_addr (provisioner's own element address).
         * NOTE: must NOT use prov_start_address here — that is the first NODE's address.
         * ESP_BLE_MESH_CID_NVAL = 0xFFFF means SIG model (no company ID).
         * Binding is done one-per-tick to avoid simultaneous BLE + SPI DMA allocations
         * that previously caused "request_data TX buffer alloc failed" errors.       */
        static const uint16_t client_model_ids[] = {
            ESP_BLE_MESH_MODEL_ID_CONFIG_CLI,
            ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI,
            ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI,
            ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_CLI,
            ESP_BLE_MESH_MODEL_ID_SCENE_CLI,
        };
        for (size_t i = 0; i < ARRAY_SIZE(client_model_ids); i++) {
            esp_err_t bind_ret = esp_ble_mesh_provisioner_bind_app_key_to_local_model(
                prov.prov_unicast_addr, app_idx,
                client_model_ids[i], ESP_BLE_MESH_CID_NVAL);
            if (bind_ret != ESP_OK) {
                ESP_LOGW(TAG, "Bind model 0x%04X to app_idx=%u failed: %s",
                         client_model_ids[i], app_idx, esp_err_to_name(bind_ret));
            }
            /* Yield after each binding so the WAN_UL task gets a chance to
             * allocate its DMA TX buffer without competing with BLE allocs. */
            taskYIELD();
        }
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        if (param->provisioner_bind_app_key_to_model_comp.err_code != 0) {
            ESP_LOGW(TAG, "Bind model 0x%04X to app_idx=%u err=%d",
                     param->provisioner_bind_app_key_to_model_comp.model_id,
                     param->provisioner_bind_app_key_to_model_comp.app_idx,
                     param->provisioner_bind_app_key_to_model_comp.err_code);
        } else {
            ESP_LOGI(TAG, "Model 0x%04X bound to app_idx=%u",
                     param->provisioner_bind_app_key_to_model_comp.model_id,
                     param->provisioner_bind_app_key_to_model_comp.app_idx);
        }
        break;

    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT: {
        /* Unprovisioned device discovered — accumulate, do NOT send individually.
         * ble_native_scan_flush() will send all results as one batch at SCAN_DONE. */
        uint8_t *dev_uuid = param->provisioner_recv_unprov_adv_pkt.dev_uuid;
        uint8_t *addr     = param->provisioner_recv_unprov_adv_pkt.addr;
        uint16_t oob_info = param->provisioner_recv_unprov_adv_pkt.oob_info;

        /* Deduplicate: skip if UUID already in buffer (same device sends many packets) */
        bool duplicate = false;
        for (uint8_t j = 0; j < s_scan_count; j++) {
            if (memcmp(s_scan_buf[j].uuid_raw, dev_uuid, 16) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) break;

        if (s_scan_count < BLE_NATIVE_MAX_SCAN_DEVICES) {
            native_scan_dev_t *d = &s_scan_buf[s_scan_count];
            memcpy(d->uuid_raw, dev_uuid, 16);
            for (int i = 0; i < 16; i++) {
                snprintf(d->uuid_str + i * 2, 3, "%02X", dev_uuid[i]);
            }
            snprintf(d->addr_str, sizeof(d->addr_str),
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            memcpy(d->addr_raw, addr, 6);
            d->addr_type = param->provisioner_recv_unprov_adv_pkt.addr_type;
            d->oob_info  = oob_info;
            s_scan_count++;
            ESP_LOGI(TAG, "Scan[%u]: uuid=%s addr=%s",
                     s_scan_count - 1, d->uuid_str, d->addr_str);
        } else {
            ESP_LOGW(TAG, "Scan buffer full, ignoring device");
        }
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT: {
        /* Provisioning complete — unblock handle_provision() in downlink task */
        uint8_t *uuid = param->provisioner_prov_complete.device_uuid;
        uint16_t addr = param->provisioner_prov_complete.unicast_addr;

        char uuid_str[33] = {0};
        for (int i = 0; i < 16; i++) {
            snprintf(uuid_str + i * 2, 3, "%02X", uuid[i]);
        }

        ESP_LOGI(TAG, "Node provisioned: addr=0x%04X uuid=%s", addr, uuid_str);

        /* Give semaphore so handle_provision() can build the final response */
        s_prov_addr = addr;
        if (s_prov_sem) {
            xSemaphoreGive(s_prov_sem);
        } else {
            /* Semaphore not initialised — fallback unsolicited uplink */
            char resp[128];
            snprintf(resp, sizeof(resp), "PROVISIONED:0x%04X:%s", addr, uuid_str);
            ble_native_uplink_send_ok(0, resp);
        }
        break;
    }

    default:
        ESP_LOGI(TAG, "Provisioning event: %d", event);
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

    /* Initialize BT controller and Bluedroid if not already done.
     * Normally initialized at startup in app_main; this is a safety fallback. */
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_LOGW(TAG, "BT controller not yet started — doing late init (may cause memory issues)");
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ret = esp_bluedroid_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = esp_bluedroid_enable();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

#if BLE_NATIVE_MESH_SUPPORTED
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

    /* NOTE: Do NOT call esp_ble_mesh_provisioner_prov_enable() here.
     * Provisioner enable/disable is managed exclusively by handle_scan() in
     * ble_native_downlink.c to avoid double-enable errors. */

    s_mesh_initialized = true;
#else
    ESP_LOGI(TAG, "BLE Mesh stack skipped (BLE_NATIVE_MESH_SUPPORTED=0) — GATT Central only");
#endif
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

    /* Retrieve mesh config */
    ble_native_mesh_cfg_t mc;
    ret = ble_native_config_get_mesh(stack_id, &mc);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "MESH_CFG_ERROR");
        return ret;
    }

    /* Register net key with provisioner stack.
     * net_idx = stack_id + 1 because index 0 (ESP_BLE_MESH_KEY_PRIMARY) is
     * reserved and rejected by esp_ble_mesh_provisioner_add_local_net_key().
     * If the key already exists (re-config), update it instead.
     * Both calls are async — completion fires PROVISIONER_ADD_LOCAL_NET_KEY_COMP_EVT
     * and PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT in prov_callback. */
    const uint16_t net_idx = (uint16_t)(stack_id + 1);
    const uint16_t app_idx = (uint16_t)(stack_id + 1);

    ret = esp_ble_mesh_provisioner_add_local_net_key(mc.net_key, net_idx);
    if (ret != ESP_OK) {
        /* ESP_ERR_INVALID_STATE means the key already exists — try update */
        if (ret == ESP_ERR_INVALID_STATE) {
            ret = esp_ble_mesh_provisioner_update_local_net_key(mc.net_key, net_idx);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "stack=%u: add/update net_key failed: %s",
                     stack_id, esp_err_to_name(ret));
            ble_native_uplink_send_fail(stack_id, "NET_KEY_FAIL");
            return ret;
        }
    }

    ret = esp_ble_mesh_provisioner_add_local_app_key(mc.app_key, net_idx, app_idx);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_STATE) {
            ret = esp_ble_mesh_provisioner_update_local_app_key(mc.app_key, net_idx, app_idx);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "stack=%u: add/update app_key failed: %s",
                     stack_id, esp_err_to_name(ret));
            ble_native_uplink_send_fail(stack_id, "APP_KEY_FAIL");
            return ret;
        }
    }

    ESP_LOGI(TAG, "stack=%u: net_key and app_key registered (net_idx=%u app_idx=%u)",
             stack_id, net_idx, app_idx);

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

/* --------------------------------------------------------------------------
 * Public API — Deinitialize Handler
 * -------------------------------------------------------------------------- */

/**
 * @brief Deinitialize the BLE Native (Mesh) handler.
 *
 * Stops uplink/downlink tasks and deinitializes the BLE Mesh provisioner stack.
 * Safe to call even if not initialized.
 *
 * @return ESP_OK on success, or an error code if mesh deinit fails.
 */
esp_err_t ble_native_handler_deinit(void) {
    ESP_LOGI(TAG, "BLE Native handler deinitializing...");

    /* These tasks are created independently from the mesh stack and must be
     * stopped explicitly to release their stacks/TCBs before FOTA. */
    ble_native_downlink_task_stop();
    ble_native_uplink_task_stop();

    if (!s_mesh_initialized) {
        ESP_LOGI(TAG, "BLE Native mesh stack not initialized, tasks stopped only");
        return ESP_OK;
    }

    /* Disable mesh provisioner — disable all bearers (PB-ADV + PB-GATT) */
    esp_err_t ret = esp_ble_mesh_provisioner_prov_disable(
        ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE Mesh provisioner disable returned: %s", esp_err_to_name(ret));
    }

    /* Deinitialize mesh stack — requires deinit_param */
    esp_ble_mesh_deinit_param_t deinit_param = {0};
    ret = esp_ble_mesh_deinit(&deinit_param);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE Mesh deinit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_mesh_initialized = false;
    ESP_LOGI(TAG, "BLE Native handler deinitialized successfully");
    return ESP_OK;
}
