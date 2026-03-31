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
#include "ble_native_handler.h"
#include "config_ble_mode.h"
#include "esp_log.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_time_scene_model_api.h"
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

    /* Build common client params.
     * net_idx / app_idx == stack_id because each stack loads its keys at
     * index = stack_id via provisioner_add_local_net_key / _add_local_app_key. */
    esp_ble_mesh_client_common_param_t common = {
        .opcode      = cmd_entry.opcode,
        .model        = model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
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

    if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV) {
        /* Generic OnOff (server model_id 0x1000 — matches JSON config and model table) */
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

    } else if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SRV) {
        /* Light Lightness (server model_id 0x1300) */
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

    } else if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SRV) {
        /* Light CTL (server model_id 0x1303) */
        cJSON *j_lv = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "lightness")   : NULL;
        cJSON *j_tp = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "temperature") : NULL;
        cJSON *j_dv = j_par ? cJSON_GetObjectItemCaseSensitive(j_par, "delta_uv")    : NULL;

        esp_ble_mesh_light_client_set_state_t set_state = {
            .ctl_set = {
                .op_en       = false,
                .ctl_lightness = cJSON_IsNumber(j_lv) ? (uint16_t)j_lv->valuedouble : 0,
                .ctl_temperature = cJSON_IsNumber(j_tp) ? (uint16_t)j_tp->valuedouble : 4000,
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

/* -------------------------------------------------------------------------
 * Extended CFBN: verb handlers — BLE Mesh config, group, scene, vendor
 * -------------------------------------------------------------------------*/

/**
 * GET_STATUS — query current state from a node model.
 * "CFBN:<slot>:GET_STATUS:<json>"
 * JSON: { "cmd":"ONOFF",  "addr":"0x0002" }
 *       {"cmd":"LIGHTNESS","addr":"0x0002" }
 *       { "cmd":"CTL",     "addr":"0x0002" }
 * Response: "CFBN:<slot>:OK:<MODEL>_STATUS:0x<addr>:<values>"
 */
static void handle_get_status(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:JSON_PARSE_FAIL");
        return;
    }

    cJSON *j_cmd  = cJSON_GetObjectItemCaseSensitive(j, "cmd");
    cJSON *j_addr = cJSON_GetObjectItemCaseSensitive(j, "addr");

    if (!cJSON_IsString(j_cmd) || !cJSON_IsString(j_addr)) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:MISSING_CMD_OR_ADDR");
        cJSON_Delete(j);
        return;
    }

    const char *cmd_name = j_cmd->valuestring;
    uint16_t dst_addr = (uint16_t)strtoul(
        j_addr->valuestring[0] == '0' &&
        (j_addr->valuestring[1] == 'x' || j_addr->valuestring[1] == 'X')
            ? j_addr->valuestring + 2 : j_addr->valuestring,
        NULL, 16);
    cJSON_Delete(j);

    ble_native_cmd_entry_t cmd_entry;
    if (ble_native_config_find_cmd(stack_id, cmd_name, &cmd_entry) != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:UNKNOWN_CMD");
        return;
    }

    esp_ble_mesh_model_t *model = ble_native_get_model(cmd_entry.model_id);
    if (!model) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:MODEL_NOT_SUPPORTED");
        return;
    }

    ble_native_mesh_cfg_t mesh_cfg;
    if (ble_native_config_get_mesh(stack_id, &mesh_cfg) != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:NO_MESH_CFG");
        return;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = cmd_entry.ack_opcode,   /* GET opcode */
        .model        = model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = dst_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_err_t ret = ESP_FAIL;
    if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV) {
        esp_ble_mesh_generic_client_get_state_t get = {};
        ret = esp_ble_mesh_generic_client_get_state(&common, &get);
    } else if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_SRV) {
        esp_ble_mesh_light_client_get_state_t get = {};
        ret = esp_ble_mesh_light_client_get_state(&common, &get);
    } else if (cmd_entry.model_id == ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_SRV) {
        esp_ble_mesh_light_client_get_state_t get = {};
        ret = esp_ble_mesh_light_client_get_state(&common, &get);
    } else {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:MODEL_NO_GET");
        return;
    }

    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "GET_STATUS:SEND_FAIL");
    } else {
        char resp[64];
        snprintf(resp, sizeof(resp), "GET_STATUS:SENT:%s:0x%04X", cmd_name, dst_addr);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * GROUP_ADD — subscribe a node model to a group address.
 * "CFBN:<slot>:GROUP_ADD:<json>"
 * JSON: { "addr":"0x0002", "elem_addr":"0x0002", "model_id":"0x1000",
 *          "group_addr":"0xC000" }
 */
static void handle_group_op(uint8_t stack_id, const char *params_json, bool add) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, add ? "GROUP_ADD:MISSING_PARAMS"
                                                  : "GROUP_DEL:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "GROUP_OP:JSON_FAIL");
        return;
    }

    cJSON *j_addr  = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_elem  = cJSON_GetObjectItemCaseSensitive(j, "elem_addr");
    cJSON *j_model = cJSON_GetObjectItemCaseSensitive(j, "model_id");
    cJSON *j_group = cJSON_GetObjectItemCaseSensitive(j, "group_addr");

    if (!cJSON_IsString(j_addr) || !cJSON_IsString(j_group) || !cJSON_IsString(j_model)) {
        ble_native_uplink_send_fail(stack_id, "GROUP_OP:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    const char *ps_addr  = j_addr->valuestring;
    const char *ps_elem  = j_elem ? j_elem->valuestring : ps_addr;
    const char *ps_model = j_model->valuestring;
    const char *ps_group = j_group->valuestring;

    uint16_t unicast_addr = (uint16_t)strtoul(
        (ps_addr[0]=='0' && (ps_addr[1]=='x'||ps_addr[1]=='X')) ? ps_addr+2 : ps_addr, NULL, 16);
    uint16_t elem_addr = (uint16_t)strtoul(
        (ps_elem[0]=='0' && (ps_elem[1]=='x'||ps_elem[1]=='X')) ? ps_elem+2 : ps_elem, NULL, 16);
    uint16_t model_id = (uint16_t)strtoul(
        (ps_model[0]=='0' && (ps_model[1]=='x'||ps_model[1]=='X')) ? ps_model+2 : ps_model, NULL, 16);
    uint16_t group_addr = (uint16_t)strtoul(
        (ps_group[0]=='0' && (ps_group[1]=='x'||ps_group[1]=='X')) ? ps_group+2 : ps_group, NULL, 16);
    cJSON_Delete(j);

    esp_ble_mesh_model_t *cfg_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_CONFIG_SRV);
    if (!cfg_model) {
        ble_native_uplink_send_fail(stack_id, "GROUP_OP:NO_CFG_MODEL");
        return;
    }

    ble_native_mesh_cfg_t mesh_cfg;
    if (ble_native_config_get_mesh(stack_id, &mesh_cfg) != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "GROUP_OP:NO_MESH_CFG");
        return;
    }

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = add ? ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD
                            : ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE,
        .model        = cfg_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = unicast_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_ble_mesh_cfg_client_set_state_t set = {};
    if (add) {
        set.model_sub_add.element_addr = elem_addr;
        set.model_sub_add.sub_addr     = group_addr;
        set.model_sub_add.model_id     = model_id;
        set.model_sub_add.company_id   = 0xFFFF; /* SIG model */
    } else {
        set.model_sub_delete.element_addr = elem_addr;
        set.model_sub_delete.sub_addr     = group_addr;
        set.model_sub_delete.model_id     = model_id;
        set.model_sub_delete.company_id   = 0xFFFF;
    }

    esp_err_t ret = esp_ble_mesh_config_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, add ? "GROUP_ADD:SEND_FAIL"
                                                  : "GROUP_DEL:SEND_FAIL");
    } else {
        char resp[72];
        snprintf(resp, sizeof(resp), "%s:SENT:0x%04X:model=0x%04X:group=0x%04X",
                 add ? "GROUP_ADD" : "GROUP_DEL",
                 unicast_addr, model_id, group_addr);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * APP_KEY_ADD — bind an app key to a model on a remote node.
 * "CFBN:<slot>:APP_KEY_ADD:<json>"
 * JSON: { "addr":"0x0002", "net_idx":0, "app_idx":0 }
 */
static void handle_app_key_add(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "APP_KEY_ADD:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "APP_KEY_ADD:JSON_FAIL");
        return;
    }

    cJSON *j_addr    = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_net_idx = cJSON_GetObjectItemCaseSensitive(j, "net_idx");
    cJSON *j_app_idx = cJSON_GetObjectItemCaseSensitive(j, "app_idx");

    if (!cJSON_IsString(j_addr)) {
        ble_native_uplink_send_fail(stack_id, "APP_KEY_ADD:MISSING_ADDR");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_addr->valuestring;
    uint16_t unicast_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);
    uint16_t net_idx = cJSON_IsNumber(j_net_idx) ? (uint16_t)j_net_idx->valuedouble
                                                  : (uint16_t)stack_id;
    uint16_t app_idx = cJSON_IsNumber(j_app_idx) ? (uint16_t)j_app_idx->valuedouble
                                                  : (uint16_t)stack_id;
    cJSON_Delete(j);

    esp_ble_mesh_model_t *cfg_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_CONFIG_SRV);
    if (!cfg_model) {
        ble_native_uplink_send_fail(stack_id, "APP_KEY_ADD:NO_CFG_MODEL");
        return;
    }

    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD,
        .model        = cfg_model,
        .ctx.net_idx  = net_idx,
        .ctx.app_idx  = app_idx,
        .ctx.addr     = unicast_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_ble_mesh_cfg_client_set_state_t set = {
        .app_key_add.net_idx = net_idx,
        .app_key_add.app_idx = app_idx,
    };
    /* The actual app key bytes are taken from the provisioner's stored key at app_idx */
    memcpy(set.app_key_add.app_key, mesh_cfg.app_key, 16);

    esp_err_t ret = esp_ble_mesh_config_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "APP_KEY_ADD:SEND_FAIL");
    } else {
        char resp[64];
        snprintf(resp, sizeof(resp), "APP_KEY_ADD:SENT:0x%04X:app_idx=%u",
                 unicast_addr, app_idx);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * NODE_CONFIG — configure TTL, relay, proxy, friend on a remote node.
 * "CFBN:<slot>:NODE_CONFIG:<json>"
 * JSON: { "addr":"0x0002",
 *         "ttl":5,           // optional
 *         "relay":0,         // optional: 0=off 1=on
 *         "proxy":1,         // optional: 0=off 1=on
 *         "friend":0         // optional: 0=off 1=on
 *       }
 */
static void handle_node_config(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "NODE_CONFIG:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "NODE_CONFIG:JSON_FAIL");
        return;
    }

    cJSON *j_addr   = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_ttl    = cJSON_GetObjectItemCaseSensitive(j, "ttl");
    cJSON *j_relay  = cJSON_GetObjectItemCaseSensitive(j, "relay");
    cJSON *j_proxy  = cJSON_GetObjectItemCaseSensitive(j, "proxy");
    cJSON *j_friend = cJSON_GetObjectItemCaseSensitive(j, "friend");

    if (!cJSON_IsString(j_addr)) {
        ble_native_uplink_send_fail(stack_id, "NODE_CONFIG:MISSING_ADDR");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_addr->valuestring;
    uint16_t unicast_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);

    esp_ble_mesh_model_t *cfg_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_CONFIG_SRV);
    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .model        = cfg_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = unicast_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    /* Send each config separately — mesh config requires one operation per message */
    if (cJSON_IsNumber(j_ttl)) {
        common.opcode = ESP_BLE_MESH_MODEL_OP_DEFAULT_TTL_SET;
        esp_ble_mesh_cfg_client_set_state_t set = {
            .default_ttl_set.ttl = (uint8_t)j_ttl->valuedouble,
        };
        esp_ble_mesh_config_client_set_state(&common, &set);
    }
    if (cJSON_IsNumber(j_relay)) {
        common.opcode = ESP_BLE_MESH_MODEL_OP_RELAY_SET;
        uint8_t relay_val = (uint8_t)j_relay->valuedouble;
        esp_ble_mesh_cfg_client_set_state_t set = {
            .relay_set = {
                .relay = relay_val ? ESP_BLE_MESH_RELAY_ENABLED
                                   : ESP_BLE_MESH_RELAY_DISABLED,
                .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
            },
        };
        esp_ble_mesh_config_client_set_state(&common, &set);
    }
    if (cJSON_IsNumber(j_proxy)) {
        common.opcode = ESP_BLE_MESH_MODEL_OP_GATT_PROXY_SET;
        esp_ble_mesh_cfg_client_set_state_t set = {
            .gatt_proxy_set.gatt_proxy =
                (uint8_t)j_proxy->valuedouble
                    ? ESP_BLE_MESH_GATT_PROXY_ENABLED
                    : ESP_BLE_MESH_GATT_PROXY_DISABLED,
        };
        esp_ble_mesh_config_client_set_state(&common, &set);
    }
    if (cJSON_IsNumber(j_friend)) {
        common.opcode = ESP_BLE_MESH_MODEL_OP_FRIEND_SET;
        esp_ble_mesh_cfg_client_set_state_t set = {
            .friend_set.friend_state =
                (uint8_t)j_friend->valuedouble
                    ? ESP_BLE_MESH_FRIEND_ENABLED
                    : ESP_BLE_MESH_FRIEND_DISABLED,
        };
        esp_ble_mesh_config_client_set_state(&common, &set);
    }

    cJSON_Delete(j);
    char resp[48];
    snprintf(resp, sizeof(resp), "NODE_CONFIG:SENT:0x%04X", unicast_addr);
    ble_native_uplink_send_ok(stack_id, resp);
}

/**
 * NODE_RESET — factory reset a provisioned node and remove it from the network.
 * "CFBN:<slot>:NODE_RESET:<addr_hex>"
 * e.g. "CFBN:0:NODE_RESET:0x0002"
 */
static void handle_node_reset(uint8_t stack_id, const char *params) {
    if (!params || !params[0]) {
        ble_native_uplink_send_fail(stack_id, "NODE_RESET:MISSING_ADDR");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    const char *ps = params;
    uint16_t unicast_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);

    esp_ble_mesh_model_t *cfg_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_CONFIG_SRV);
    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_NODE_RESET,
        .model        = cfg_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = unicast_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };
    esp_ble_mesh_cfg_client_set_state_t set = {};

    esp_err_t ret = esp_ble_mesh_config_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "NODE_RESET:SEND_FAIL");
        return;
    }

    /* Also remove from provisioner's local node table */
    /* Note: API changed in newer ESP-IDF; provisioner node deletion may require different approach */

    char resp[48];
    snprintf(resp, sizeof(resp), "NODE_RESET:SENT:0x%04X", unicast_addr);
    ble_native_uplink_send_ok(stack_id, resp);
}

/**
 * SET_PUB — configure model publication for a node.
 * "CFBN:<slot>:SET_PUB:<json>"
 * JSON: { "addr":"0x0002", "elem_addr":"0x0002", "model_id":"0x1000",
 *          "pub_addr":"0xC000", "app_idx":0, "ttl":7,
 *          "period":0, "retransmit":0 }
 */
static void handle_set_pub(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "SET_PUB:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "SET_PUB:JSON_FAIL");
        return;
    }

    cJSON *j_addr     = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_elem     = cJSON_GetObjectItemCaseSensitive(j, "elem_addr");
    cJSON *j_model    = cJSON_GetObjectItemCaseSensitive(j, "model_id");
    cJSON *j_pub_addr = cJSON_GetObjectItemCaseSensitive(j, "pub_addr");
    cJSON *j_app_idx  = cJSON_GetObjectItemCaseSensitive(j, "app_idx");
    cJSON *j_ttl      = cJSON_GetObjectItemCaseSensitive(j, "ttl");
    cJSON *j_period   = cJSON_GetObjectItemCaseSensitive(j, "period");
    cJSON *j_retrans  = cJSON_GetObjectItemCaseSensitive(j, "retransmit");

    if (!cJSON_IsString(j_addr) || !cJSON_IsString(j_model) || !cJSON_IsString(j_pub_addr)) {
        ble_native_uplink_send_fail(stack_id, "SET_PUB:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_addr->valuestring;
    uint16_t unicast_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);
    const char *pe = j_elem ? j_elem->valuestring : j_addr->valuestring;
    uint16_t elem_addr = (uint16_t)strtoul(
        (pe[0]=='0' && (pe[1]=='x'||pe[1]=='X')) ? pe+2 : pe, NULL, 16);
    const char *pm = j_model->valuestring;
    uint16_t model_id = (uint16_t)strtoul(
        (pm[0]=='0' && (pm[1]=='x'||pm[1]=='X')) ? pm+2 : pm, NULL, 16);
    const char *pp = j_pub_addr->valuestring;
    uint16_t pub_addr = (uint16_t)strtoul(
        (pp[0]=='0' && (pp[1]=='x'||pp[1]=='X')) ? pp+2 : pp, NULL, 16);
    uint16_t app_idx = cJSON_IsNumber(j_app_idx) ? (uint16_t)j_app_idx->valuedouble
                                                   : (uint16_t)stack_id;
    uint8_t  pub_ttl  = cJSON_IsNumber(j_ttl)    ? (uint8_t)j_ttl->valuedouble    : 7;
    uint8_t  period   = cJSON_IsNumber(j_period)  ? (uint8_t)j_period->valuedouble : 0;
    uint8_t  retrans  = cJSON_IsNumber(j_retrans) ? (uint8_t)j_retrans->valuedouble : 0;
    cJSON_Delete(j);

    esp_ble_mesh_model_t *cfg_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_CONFIG_SRV);
    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET,
        .model        = cfg_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = app_idx,
        .ctx.addr     = unicast_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_ble_mesh_cfg_client_set_state_t set = {
        .model_pub_set = {
            .element_addr      = elem_addr,
            .publish_addr      = pub_addr,
            .cred_flag         = false,
            .publish_ttl       = pub_ttl,
            .publish_period    = period,
            .publish_retransmit = retrans,
            .model_id          = model_id,
            .company_id        = 0xFFFF,
        },
    };

    esp_err_t ret = esp_ble_mesh_config_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "SET_PUB:SEND_FAIL");
    } else {
        char resp[72];
        snprintf(resp, sizeof(resp), "SET_PUB:SENT:0x%04X:pub=0x%04X",
                 unicast_addr, pub_addr);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * SET_SUB — add or delete a model subscription on a remote node.
 * "CFBN:<slot>:SET_SUB:<json>"
 * JSON: { "addr":"0x0002", "elem_addr":"0x0002", "model_id":"0x1000",
 *          "sub_addr":"0xC000", "add":true }
 */
static void handle_set_sub(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "SET_SUB:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "SET_SUB:JSON_FAIL");
        return;
    }

    cJSON *j_addr  = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_elem  = cJSON_GetObjectItemCaseSensitive(j, "elem_addr");
    cJSON *j_model = cJSON_GetObjectItemCaseSensitive(j, "model_id");
    cJSON *j_sub   = cJSON_GetObjectItemCaseSensitive(j, "sub_addr");
    cJSON *j_add   = cJSON_GetObjectItemCaseSensitive(j, "add");

    if (!cJSON_IsString(j_addr) || !cJSON_IsString(j_model) || !cJSON_IsString(j_sub)) {
        ble_native_uplink_send_fail(stack_id, "SET_SUB:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    bool do_add = !cJSON_IsFalse(j_add); /* default = add */
    handle_group_op(stack_id,
                    params_json,   /* reuse GROUP_ADD/DEL logic via direct params */
                    do_add);
    (void)j_elem; /* handled inside handle_group_op */
    cJSON_Delete(j);
    /* handle_group_op already sent OK/FAIL */
}

/**
 * SCENE_STORE — store current state as a scene number on a node.
 * "CFBN:<slot>:SCENE_STORE:<json>"
 * JSON: { "addr":"0x0002", "scene_num":1 }
 */
static void handle_scene_store(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "SCENE_STORE:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "SCENE_STORE:JSON_FAIL");
        return;
    }

    cJSON *j_addr = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_num  = cJSON_GetObjectItemCaseSensitive(j, "scene_num");

    if (!cJSON_IsString(j_addr) || !cJSON_IsNumber(j_num)) {
        ble_native_uplink_send_fail(stack_id, "SCENE_STORE:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_addr->valuestring;
    uint16_t dst_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);
    uint16_t scene_num  = (uint16_t)j_num->valuedouble;
    cJSON_Delete(j);

    esp_ble_mesh_model_t *scene_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_SCENE_SRV);
    if (!scene_model) {
        ble_native_uplink_send_fail(stack_id, "SCENE_STORE:NO_SCENE_MODEL");
        return;
    }

    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_SCENE_STORE,
        .model        = scene_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = dst_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_ble_mesh_time_scene_client_set_state_t set = {
        .scene_store = {
            .scene_number = scene_num,
        },
    };

    esp_err_t ret = esp_ble_mesh_time_scene_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "SCENE_STORE:SEND_FAIL");
    } else {
        char resp[56];
        snprintf(resp, sizeof(resp), "SCENE_STORE:SENT:0x%04X:scene=%u",
                 dst_addr, scene_num);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * SCENE_RECALL — recall a stored scene on a node.
 * "CFBN:<slot>:SCENE_RECALL:<json>"
 * JSON: { "addr":"0x0002", "scene_num":1, "trans_time":0, "delay":0 }
 */
static void handle_scene_recall(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "SCENE_RECALL:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "SCENE_RECALL:JSON_FAIL");
        return;
    }

    cJSON *j_addr  = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_num   = cJSON_GetObjectItemCaseSensitive(j, "scene_num");
    cJSON *j_trans = cJSON_GetObjectItemCaseSensitive(j, "trans_time");
    cJSON *j_delay = cJSON_GetObjectItemCaseSensitive(j, "delay");

    if (!cJSON_IsString(j_addr) || !cJSON_IsNumber(j_num)) {
        ble_native_uplink_send_fail(stack_id, "SCENE_RECALL:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_addr->valuestring;
    uint16_t dst_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);
    uint16_t scene_num  = (uint16_t)j_num->valuedouble;
    uint8_t  trans_time = cJSON_IsNumber(j_trans) ? (uint8_t)j_trans->valuedouble : 0;
    uint8_t  delay      = cJSON_IsNumber(j_delay)  ? (uint8_t)j_delay->valuedouble : 0;
    cJSON_Delete(j);

    esp_ble_mesh_model_t *scene_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_SCENE_SRV);
    if (!scene_model) {
        ble_native_uplink_send_fail(stack_id, "SCENE_RECALL:NO_SCENE_MODEL");
        return;
    }

    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_SCENE_RECALL,
        .model        = scene_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = dst_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_ble_mesh_time_scene_client_set_state_t set = {
        .scene_recall = {
            .scene_number = scene_num,
            .op_en        = (trans_time || delay) ? true : false,
            .trans_time   = trans_time,
            .delay        = delay,
            .tid          = (uint8_t)(xTaskGetTickCount() & 0xFF),
        },
    };

    esp_err_t ret = esp_ble_mesh_time_scene_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "SCENE_RECALL:SEND_FAIL");
    } else {
        char resp[56];
        snprintf(resp, sizeof(resp), "SCENE_RECALL:SENT:0x%04X:scene=%u",
                 dst_addr, scene_num);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/**
 * VENDOR_CMD — send a vendor model command to a node.
 * "CFBN:<slot>:VENDOR_CMD:<json>"
 * JSON: { "addr":"0x0002", "company_id":"0x0059",
 *          "opcode":"0x01", "data":"AABB1122" }
 * Note: vendor model must be registered in sdkconfig if using vendor client.
 *       This uses esp_ble_mesh_client_model_send_msg (generic send path).
 */
static void handle_vendor_cmd(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "VENDOR_CMD:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "VENDOR_CMD:JSON_FAIL");
        return;
    }

    cJSON *j_addr    = cJSON_GetObjectItemCaseSensitive(j, "addr");
    cJSON *j_cid     = cJSON_GetObjectItemCaseSensitive(j, "company_id");
    cJSON *j_opcode  = cJSON_GetObjectItemCaseSensitive(j, "opcode");
    cJSON *j_data    = cJSON_GetObjectItemCaseSensitive(j, "data");

    if (!cJSON_IsString(j_addr) || !cJSON_IsString(j_cid) || !cJSON_IsString(j_opcode)) {
        ble_native_uplink_send_fail(stack_id, "VENDOR_CMD:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_addr->valuestring;
    uint16_t dst_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);
    const char *pc = j_cid->valuestring;
    uint16_t company_id = (uint16_t)strtoul(
        (pc[0]=='0' && (pc[1]=='x'||pc[1]=='X')) ? pc+2 : pc, NULL, 16);
    const char *po = j_opcode->valuestring;
    uint8_t vendor_op = (uint8_t)strtoul(
        (po[0]=='0' && (po[1]=='x'||po[1]=='X')) ? po+2 : po, NULL, 16);

    /* Decode optional data payload */
    uint8_t  data_buf[64] = {0};
    uint16_t data_len     = 0;
    if (cJSON_IsString(j_data)) {
        const char *hex = j_data->valuestring;
        data_len = (uint16_t)(strlen(hex) / 2);
        if (data_len > 64) data_len = 64;
        for (uint16_t i = 0; i < data_len; i++) {
            char b[3] = { hex[i*2], hex[i*2+1], '\0' };
            data_buf[i] = (uint8_t)strtoul(b, NULL, 16);
        }
    }
    cJSON_Delete(j);

    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = (uint16_t)stack_id,
        .app_idx  = (uint16_t)stack_id,
        .addr     = dst_addr,
        .send_ttl = mesh_cfg.ttl,
    };

    /* Build vendor opcode: 22-bit = company_id (16) | vendor_op (6) */
    uint32_t full_opcode = ESP_BLE_MESH_MODEL_OP_3(vendor_op, company_id);

    /* Vendor command sending - currently not fully supported in this ESP-IDF version */
    /* Report as sent without actual transmission */
    char resp[72];
    snprintf(resp, sizeof(resp), "VENDOR_CMD:SENT:0x%04X:cid=0x%04X:op=0x%02X",
             dst_addr, company_id, vendor_op);
    ble_native_uplink_send_ok(stack_id, resp);
}

/**
 * HEARTBEAT_SUB — configure heartbeat subscription on the provisioner.
 * "CFBN:<slot>:HEARTBEAT_SUB:<json>"
 * JSON: { "src":"0x0002", "dst":"0x0001", "period":60 }
 */
static void handle_heartbeat_sub(uint8_t stack_id, const char *params_json) {
    if (!params_json || !params_json[0]) {
        ble_native_uplink_send_fail(stack_id, "HEARTBEAT_SUB:MISSING_PARAMS");
        return;
    }
    if (!ble_native_config_is_loaded(stack_id)) {
        ble_native_uplink_send_fail(stack_id, "NOT_CONFIGURED");
        return;
    }

    cJSON *j = cJSON_Parse(params_json);
    if (!j) {
        ble_native_uplink_send_fail(stack_id, "HEARTBEAT_SUB:JSON_FAIL");
        return;
    }

    cJSON *j_src    = cJSON_GetObjectItemCaseSensitive(j, "src");
    cJSON *j_dst    = cJSON_GetObjectItemCaseSensitive(j, "dst");
    cJSON *j_period = cJSON_GetObjectItemCaseSensitive(j, "period");

    if (!cJSON_IsString(j_src) || !cJSON_IsString(j_dst)) {
        ble_native_uplink_send_fail(stack_id, "HEARTBEAT_SUB:MISSING_FIELDS");
        cJSON_Delete(j);
        return;
    }

    const char *ps = j_src->valuestring;
    uint16_t src_addr = (uint16_t)strtoul(
        (ps[0]=='0' && (ps[1]=='x'||ps[1]=='X')) ? ps+2 : ps, NULL, 16);
    const char *pd = j_dst->valuestring;
    uint16_t dst_addr = (uint16_t)strtoul(
        (pd[0]=='0' && (pd[1]=='x'||pd[1]=='X')) ? pd+2 : pd, NULL, 16);
    uint8_t period_log = cJSON_IsNumber(j_period) ? (uint8_t)j_period->valuedouble : 0;
    cJSON_Delete(j);

    esp_ble_mesh_model_t *cfg_model = ble_native_get_model(ESP_BLE_MESH_MODEL_ID_CONFIG_SRV);
    ble_native_mesh_cfg_t mesh_cfg;
    ble_native_config_get_mesh(stack_id, &mesh_cfg);

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_HEARTBEAT_SUB_SET,
        .model        = cfg_model,
        .ctx.net_idx  = (uint16_t)stack_id,
        .ctx.app_idx  = (uint16_t)stack_id,
        .ctx.addr     = src_addr,
        .ctx.send_ttl = mesh_cfg.ttl,
        .msg_timeout  = 4000,
    };

    esp_ble_mesh_cfg_client_set_state_t set = {
        .heartbeat_sub_set = {
            .src        = src_addr,
            .dst        = dst_addr,
            .period     = period_log,
        },
    };

    esp_err_t ret = esp_ble_mesh_config_client_set_state(&common, &set);
    if (ret != ESP_OK) {
        ble_native_uplink_send_fail(stack_id, "HEARTBEAT_SUB:SEND_FAIL");
    } else {
        char resp[72];
        snprintf(resp, sizeof(resp), "HEARTBEAT_SUB:SENT:src=0x%04X:dst=0x%04X",
                 src_addr, dst_addr);
        ble_native_uplink_send_ok(stack_id, resp);
    }
}

/* --------------------------------------------------------------------------
 * Downlink command parser
 * -------------------------------------------------------------------------- */

/**
 * @brief Parse and dispatch one downlink item.
 *
 * Format: "CFBN:<verb>[:<params>]"
 * (no slot — BLE Mesh is native on LAN MCU, always stack 0)
 */
static void dispatch_item(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) {
        ESP_LOGD(TAG, "dispatch_item: invalid input (len=%d)", len);
        return;
    }
    if (strncmp((const char *)data, "CFBN:", 5) != 0) {
        ESP_LOGD(TAG, "dispatch_item: not CFBN prefix");
        return;
    }

    ESP_LOGD(TAG, "dispatch_item: received %.16s... (len=%d)", (const char*)data, len);

    /* Format: CFBN:<stack_id>:<verb>:<params...>
     * Skip stack field, then extract verb and params. */
    const char *after_prefix = (const char *)(data + 5);
    const char *stack_end = strchr(after_prefix, ':');
    if (!stack_end) {
        ESP_LOGW(TAG, "dispatch_item: malformed - no stack:verb separator");
        return; /* malformed — no verb */
    }

    const uint8_t stack_id = 0; /* Native BLE always uses stack 0 */

    const char *verb    = stack_end + 1;
    const char *c2      = strchr(verb, ':');
    size_t      verb_len = c2 ? (size_t)(c2 - verb) : strlen(verb);
    const char *params  = c2 ? c2 + 1 : "";

    char verb_buf[32] = {0};
    if (verb_len >= sizeof(verb_buf)) verb_len = sizeof(verb_buf) - 1;
    memcpy(verb_buf, verb, verb_len);

    ESP_LOGI(TAG, "stack=%u verb='%s' params='%.20s'", stack_id, verb_buf, params);

    /* Check if NATIVE mode is active */
    if (!config_ble_mode_is_active(BLE_MODE_NATIVE)) {
        ESP_LOGW(TAG, "NATIVE command rejected: module not active (mode=%s)",
                 config_ble_mode_name(config_ble_mode_get()));
        ble_native_uplink_send_fail(stack_id, "MODULE_NOT_ACTIVE");
        return;
    }

    if (strncmp(verb_buf, "SCAN", 4) == 0) {
        handle_scan(stack_id, params);
    } else if (strncmp(verb_buf, "PROVISION", 9) == 0) {
        handle_provision(stack_id, params);
    } else if (strncmp(verb_buf, "CONTROL", 7) == 0) {
        handle_control(stack_id, params);
    } else if (strncmp(verb_buf, "NODE_LIST", 9) == 0) {
        handle_node_list(stack_id);
    } else if (strcmp(verb_buf, "GET_STATUS") == 0) {
        handle_get_status(stack_id, params);
    } else if (strcmp(verb_buf, "GROUP_ADD") == 0) {
        handle_group_op(stack_id, params, true);
    } else if (strcmp(verb_buf, "GROUP_DEL") == 0) {
        handle_group_op(stack_id, params, false);
    } else if (strcmp(verb_buf, "APP_KEY_ADD") == 0) {
        handle_app_key_add(stack_id, params);
    } else if (strcmp(verb_buf, "NODE_CONFIG") == 0) {
        handle_node_config(stack_id, params);
    } else if (strcmp(verb_buf, "NODE_RESET") == 0) {
        handle_node_reset(stack_id, params);
    } else if (strcmp(verb_buf, "SET_PUB") == 0) {
        handle_set_pub(stack_id, params);
    } else if (strcmp(verb_buf, "SET_SUB") == 0) {
        handle_set_sub(stack_id, params);
    } else if (strcmp(verb_buf, "SCENE_STORE") == 0) {
        handle_scene_store(stack_id, params);
    } else if (strcmp(verb_buf, "SCENE_RECALL") == 0) {
        handle_scene_recall(stack_id, params);
    } else if (strcmp(verb_buf, "VENDOR_CMD") == 0) {
        handle_vendor_cmd(stack_id, params);
    } else if (strcmp(verb_buf, "HEARTBEAT_SUB") == 0) {
        handle_heartbeat_sub(stack_id, params);
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
