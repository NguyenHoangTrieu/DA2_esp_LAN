/**
 * @file ble_native_downlink.c
 * @brief BLE Native downlink — parse CFBN commands, dispatch to ESP BLE Mesh.
 *
 * Design intent:
 *   - No LED protocol is hardcoded here.
 *   - Command → model_id/opcode mapping loaded from JSON config at runtime.
 *   - This file only dispatches; it does not know what "ONOFF" or "CTL" means
 *     beyond looking it up in the config table.
 */

#include "ble_native_downlink.h"
#include "ble_native_config.h"
#include "ble_native_uplink.h"
#include "esp_log.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "ble_native_dn";

/* --------------------------------------------------------------------------
 * Internal types
 * -------------------------------------------------------------------------- */

typedef struct {
    uint8_t  raw[BLE_NATIVE_DOWNLINK_ITEM_MAX];
    uint16_t len;
} downlink_item_t;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static QueueHandle_t  s_dn_queue    = NULL;
static TaskHandle_t   s_dn_task     = NULL;
static volatile bool  s_task_running = false;

/* 
 * External reference: the provisioner models are declared in ble_native_handler.c
 * where the BLE Mesh composition data lives.  The downlink layer calls those
 * helpers so models never need to be re-declared here.
 */
extern esp_ble_mesh_model_t *ble_native_get_model(uint16_t model_id);

/* --------------------------------------------------------------------------
 * Verb handlers
 * -------------------------------------------------------------------------- */

/**
 * @brief Handle SCAN verb: start provisioner ADV scan for the given duration.
 *
 * Command: "CFBN:<stack_id>:SCAN:<duration_ms_or_empty>"
 * Response: "CFBN:<stack_id>:OK:SCAN_STARTED" / "CFBN:<stack_id>:OK:SCAN_RESULT:..."
 */
static void handle_scan(uint8_t stack_id, const char *params) {
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    uint32_t duration_ms = 10000; /* default 10 s */
    if (params && params[0] != '\0') {
        unsigned long v = strtoul(params, NULL, 10);
        if (v > 0 && v <= 120000) {
            duration_ms = (uint32_t)v;
        }
    }

    /* Enable provisioner scan — discovered devices received in provisioning cb */
    esp_err_t ret = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "stack=%u: enable prov_adv failed: %s",
                 stack_id, esp_err_to_name(ret));
        ble_native_uplink_send_fail(stack_id, "SCAN_ENABLE_FAILED");
        return;
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "SCAN_STARTED:%u", (unsigned)duration_ms);
    ble_native_uplink_send_ok(stack_id, resp);

    /* Schedule scan stop after duration */
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    esp_ble_mesh_provisioner_prov_disable(ESP_BLE_MESH_PROV_ADV);
    ble_native_uplink_send_ok(stack_id, "SCAN_DONE");
}

/**
 * @brief Handle PROVISION verb: add an unprovisioned device by UUID.
 *
 * Command: "CFBN:<stack_id>:PROVISION:<uuid_hex_32chars>"
 * Response: "CFBN:<stack_id>:OK:PROVISIONED:<unicast_addr>" or FAIL
 */
static void handle_provision(uint8_t stack_id, const char *params) {
    if (!params || strlen(params) < 32) {
        ble_native_uplink_send_fail(stack_id, "PROVISION:INVALID_UUID");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    /* Parse 16-byte UUID from 32-char hex string */
    uint8_t uuid[16] = {0};
    for (int i = 0; i < 16; i++) {
        char b[3] = { params[i * 2], params[i * 2 + 1], '\0' };
        uuid[i] = (uint8_t)strtoul(b, NULL, 16);
    }

    /* Allocate unicast address for the new node */
    uint16_t unicast_addr = 0;
    if (ble_native_config_alloc_unicast(stack_id, &unicast_addr) != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "PROVISION:NO_ADDR");
        return;
    }

    esp_ble_mesh_unprov_dev_add_t add_dev = {
        .addr_type = 0,  /* will be filled by stack from adv report */
        .oob_info = 0,
        .bearer = ESP_BLE_MESH_PROV_ADV,
    };
    memcpy(add_dev.uuid, uuid, 16);

    esp_err_t ret = esp_ble_mesh_provisioner_add_unprov_dev(
        &add_dev,
        ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG |
        ADD_DEV_FLUSHABLE_DEV_FLAG);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "stack=%u: add_unprov_dev failed: %s",
                 stack_id, esp_err_to_name(ret));
        ble_native_uplink_send_fail(stack_id, "PROVISION:FAILED");
        return;
    }

    /* Actual provisioning result arrives in provisioner_prov_complete callback
     * (registered in ble_native_handler.c).  Send an immediate ACK. */
    char resp[64];
    snprintf(resp, sizeof(resp), "PROVISION_IN_PROGRESS:0x%04X", unicast_addr);
    ble_native_uplink_send_ok(stack_id, resp);
}

/**
 * @brief Handle CONTROL verb: send a mesh model message to a node.
 *
 * Command: "CFBN:<stack_id>:CONTROL:<json>"
 * JSON format: { "cmd": "<cmd_name>", "addr": "0x0005", "params": { ... } }
 *
 * All semantics (which model, which opcode, what parameters) come from the
 * runtime config loaded via CFBN:JSON.  Nothing here knows about LEDs.
 */
static void handle_control(uint8_t stack_id, const char *params_json) {
    if (!params_json || params_json[0] == '\0') {
        ble_native_uplink_send_fail(stack_id, "CONTROL:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ESP_LOGE(TAG, "CONTROL: JSON parse failed");
        ble_native_uplink_send_fail(stack_id, "CONTROL:JSON_PARSE_FAIL");
        return;
    }

    /* Extract required fields */
    cJSON *j_cmd  = cJSON_GetObjectItemCaseSensitive(j, "cmd");
    cJSON *j_addr = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_par  = cJSON_GetObjectItemCaseSensitive(j, "params");

    if (!cJSON_IsString(j_cmd) || !cJSON_IsString(j_addr)) {
        ESP_LOGE(TAG, "CONTROL: missing 'cmd' or 'addr'");
        ble_native_uplink_send_fail(stack_id, "CONTROL:MISSING_CMD_OR_ADDR");
        cJSON_Delete(j);
        return;
    }

    const char *cmd_name = j_cmd->valuestring;
    uint16_t dst_addr = (uint16_t)strtoul(
        j_addr->valuestring[0] == '0' && (j_addr->valuestring[1] == 'x' || j_addr->valuestring[1] == 'X')
            ? j_addr->valuestring + 2 : j_addr->valuestring,
        NULL, 16);

    if (dst_addr == 0) {
        ble_native_uplink_send_fail(stack_id, "CONTROL:INVALID_ADDR");
        cJSON_Delete(j);
        return;
    }

    /* Look up command in config table */
    ble_native_cmd_entry_t cmd_entry;
    esp_err_t find_ret = ble_native_config_find_cmd(stack_id, cmd_name, &cmd_entry);
    if (find_ret != ESP_OK) {
        ESP_LOGE(TAG, "CONTROL: cmd '%s' not in config table", cmd_name);
        char fail_msg[80];
        snprintf(fail_msg, sizeof(fail_msg), "CONTROL:UNKNOWN_CMD:%s", cmd_name);
        ble_native_uplink_send_fail(stack_id, fail_msg);
        cJSON_Delete(j);
        return;
    }

    /* Get model reference from handler */
    esp_ble_mesh_model_t *model = ble_native_get_model(cmd_entry.model_id);
    if (!model) {
        ESP_LOGE(TAG, "CONTROL: model 0x%04X not registered", cmd_entry.model_id);
        ble_native_uplink_send_fail(stack_id, "CONTROL:MODEL_NOT_SUPPORTED");
        cJSON_Delete(j);
        return;
    }

    /* Retrieve mesh config for TTL and app_key_idx */
    ble_native_mesh_cfg_t mesh_cfg;
    if (ble_native_config_get_mesh(stack_id, &mesh_cfg) != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "CONTROL:NO_MESH_CFG");
        cJSON_Delete(j);
        return;
    }

    /* Build common client params */
    esp_ble_mesh_client_common_param_t common = {
        .opcode      = cmd_entry.opcode,
        .model        = model,
        .ctx.net_idx  = 0,       /* primary net key index */
        .ctx.app_idx  = 0,       /* primary app key index */
        .ctx.addr     = dst_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,    /* 4 s ack timeout */
    };

    esp_err_t mesh_ret = ESP_FAIL;

    /* ---------------------------------------------------------------
     * Dispatch by model_id — add new models here as the project grows.
     * The actual operation is still decided by opcode from JSON config;
     * we just need model-specific structs for ESP-IDF.
     * --------------------------------------------------------------- */

    if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI) {
        /* Generic OnOff (model 0x1000) */
        cJSON *j_value = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "value") : NULL;
        uint8_t onoff_val = cJSON_IsNumber(j_value) ? (uint8_t)j_value->valuedouble : 0;

        esp_ble_mesh_generic_client_set_state_t set_state = {
            .onoff_set = {
                .op_en     = false,
                .onoff     = onoff_val,
                .tid       = (uint8_t)(xTaskGetTickCount() & 0xFF),
                .trans_time = 0,
                .delay     = 0,
            }
        };
        mesh_ret = esp_ble_mesh_generic_client_set_state(&common, &set_state);

    } else if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI) {
        /* Light Lightness (model 0x1302) */
        cJSON *j_ln = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "lightness") : NULL;
        uint16_t lightness = cJSON_IsNumber(j_ln) ? (uint16_t)j_ln->valuedouble : 0;

        esp_ble_mesh_light_client_set_state_t set_state = {
            .lightness_set = {
                .op_en     = false,
                .lightness = lightness,
                .tid       = (uint8_t)(xTaskGetTickCount() & 0xFF),
                .trans_time = 0,
                .delay     = 0,
            }
        };
        mesh_ret = esp_ble_mesh_light_client_set_state(&common, &set_state);

    } else if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_CLI) {
        /* Light CTL (model 0x1305) */
        cJSON *j_lv = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "lightness")   : NULL;
        cJSON *j_tp = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "temperature") : NULL;
        cJSON *j_dv = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "delta_uv")    : NULL;

        esp_ble_mesh_light_client_set_state_t set_state = {
            .ctl_set = {
                .op_en       = false,
                .ctl_lightness = cJSON_IsNumber(j_lv) ? (uint16_t)j_lv->valuedouble : 0,
                .ctl_temperatrue = cJSON_IsNumber(j_tp) ? (uint16_t)j_tp->valuedouble : 4000,
                .ctl_delta_uv  = cJSON_IsNumber(j_dv) ? (int16_t)j_dv->valuedouble : 0,
                .tid           = (uint8_t)(xTaskGetTickCount() & 0xFF),
                .trans_time    = 0,
                .delay         = 0,
            }
        };
        mesh_ret = esp_ble_mesh_light_client_set_state(&common, &set_state);

    } else {
        ESP_LOGW(TAG, "CONTROL: model 0x%04X has no handler; raw opcode send not yet implemented",
                 cmd_entry.model_id);
        ble_native_uplink_send_fail(stack_id, "CONTROL:MODEL_HANDLER_MISSING");
        cJSON_Delete(j);
        return;
    }

    cJSON_Delete(j);

    if (mesh_ret != ESP_OK) {
        ESP_LOGE(TAG, "CONTROL: mesh send failed: %s", esp_err_to_name(mesh_ret));
        ble_native_uplink_send_fail(stack_id, "CONTROL:MESH_SEND_FAIL");
    } else {
        /* ACK will arrive in generic/light client callbacks registered in handler */
        char resp[64];
        snprintf(resp, sizeof(resp), "CONTROL:SENT:%s:0x%04X", cmd_name, dst_addr);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * @brief Handle NODE_LIST verb: report all provisioned nodes.
 *
 * Command: "CFBN:<stack_id>:NODE_LIST"
 * Response: "CFBN:<stack_id>:OK:NODE_LIST:[{addr,uuid}...]"
 */
static void handle_node_list(uint8_t stack_id) {
    /* esp_ble_mesh_provisioner_get_node_table() is not a standard ESP-IDF API;
     * nodes are tracked via provisioner callbacks in ble_native_handler.c.
     * This stub forwards the request to the handler for now. */
    ble_native_uplink_send_ok(stack_id, "NODE_LIST:SEE_HANDLER");
}

/* --------------------------------------------------------------------------
 * Downlink command parser
 * -------------------------------------------------------------------------- */

/**
 * @brief Parse and dispatch one downlink item.
 *
 * Format: "CFBN:<stack_id>:<verb>[:<params>]"
 */
static void dispatch_item(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) return;
    if (strncmp((const char *)data, "CFBN:", 5) != 0) return;

    /* Parse stack_id */
    const char *ptr = (const char *)(data + 5);
    const char *c1 = strchr(ptr, ':');
    if (!c1) return;

    uint8_t stack_id = (uint8_t)strtoul(ptr, NULL, 10);
    if (stack_id >= BLE_NATIVE_MAX_STACKS) {
        ESP_LOGE(TAG, "Invalid stack_id %u", stack_id);
        return;
    }

    /* Parse verb */
    const char *verb = c1 + 1;
    const char *c2 = strchr(verb, ':');
    size_t verb_len = c2 ? (size_t)(c2 - verb) : strlen(verb);
    const char *params = c2 ? c2 + 1 : "";

    char verb_buf[32] = {0};
    if (verb_len >= sizeof(verb_buf)) verb_len = sizeof(verb_buf) - 1;
    memcpy(verb_buf, verb, verb_len);

    ESP_LOGI(TAG, "stack=%u verb='%s'", stack_id, verb_buf);

    if (strncmp(verb_buf, "SCAN", 4) == 0) {
        handle_scan(stack_id, params);
    } else if (strncmp(verb_buf, "PROVISION", 9) == 0) {
        handle_provision(stack_id, params);
    } else if (strncmp(verb_buf, "CONTROL", 7) == 0) {
        handle_control(stack_id, params);
    } else if (strncmp(verb_buf, "NODE_LIST", 9) == 0) {
        handle_node_list(stack_id);
    } else {
        ESP_LOGW(TAG, "Unknown verb '%s'", verb_buf);
        ble_native_uplink_send_fail(stack_id, "UNKNOWN_VERB");
    }
}

/* --------------------------------------------------------------------------
 * Task
 * -------------------------------------------------------------------------- */

static void downlink_task(void *arg) {
    downlink_item_t *item = NULL;
    ESP_LOGI(TAG, "Downlink task started");

    while (s_task_running) {
        if (!item) {
            item = (downlink_item_t *)malloc(sizeof(downlink_item_t));
            if (!item) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

        if (xQueueReceive(s_dn_queue, item, pdMS_TO_TICKS(100)) == pdTRUE) {
            dispatch_item(item->raw, item->len);
        }
    }

    free(item);
    ESP_LOGI(TAG, "Downlink task exiting");
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_native_downlink_task_start(void) {
    if (s_task_running) {
        return ESP_OK;
    }

    if (!s_dn_queue) {
        s_dn_queue = xQueueCreate(BLE_NATIVE_DOWNLINK_QUEUE_DEPTH,
                                   sizeof(downlink_item_t));
        if (!s_dn_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    s_task_running = true;
    BaseType_t ret = xTaskCreate(downlink_task, "ble_native_dn",
                                  8 * 1024, NULL, 5, &s_dn_task);
    if (ret != pdPASS) {
        s_task_running = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ble_native_downlink_task_stop(void) {
    s_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    s_dn_task = NULL;
}

esp_err_t ble_native_downlink_enqueue(const uint8_t *data, uint16_t len) {
    if (!s_dn_queue || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > BLE_NATIVE_DOWNLINK_ITEM_MAX - 1) {
        len = BLE_NATIVE_DOWNLINK_ITEM_MAX - 1;
    }

    downlink_item_t item;
    memcpy(item.raw, data, len);
    item.raw[len] = '\0';
    item.len = len;

    if (xQueueSend(s_dn_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Downlink queue full");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
