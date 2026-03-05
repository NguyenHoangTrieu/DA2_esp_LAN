/**
 * @file json_zigbee_config_parser.c
 * @brief Zigbee-specific JSON configuration parser implementation
 *
 * Mirrors json_lora_config_parser.c extended with Zigbee-specific fields:
 *   cmd_type, cmd_code, response_format, is_async_event.
 */

#include "json_zigbee_config_parser.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ZIGBEE_PARSER";

/* ============================================================================
 * Hardcoded Function Name Table
 * ========================================================================== */

/**
 * @brief Ordered function name table – index MUST match json_zigbee_function_id_t.
 */
static const char *ZIGBEE_FUNCTION_NAMES[JSON_ZIGBEE_FUNC_MAX] = {
    // Group 1 – Lifecycle (0-4)
    "MODULE_HW_RESET",              // 0
    "MODULE_SW_RESET",              // 1
    "MODULE_FACTORY_RESET",         // 2
    "MODULE_GET_INFO",              // 3
    "MODULE_ENTER_HEX_MODE",        // 4
    // Group 2 – Network Management (5-11)
    "MODULE_START_NETWORK",         // 5
    "MODULE_STOP_NETWORK",          // 6
    "MODULE_GET_NET_STATUS",        // 7
    "MODULE_SET_CHANNEL",           // 8
    "MODULE_SET_PANID",             // 9
    "MODULE_SET_TX_POWER",          // 10
    "MODULE_SET_PERMIT_JOIN",       // 11
    // Group 3 – Node Discovery (12-17)
    "MODULE_NODE_JOIN_NOTIFY",      // 12
    "MODULE_NODE_LEAVE_NOTIFY",     // 13
    "MODULE_NODE_ANNOUNCE_NOTIFY",  // 14
    "MODULE_QUERY_SHORT_ADDR",      // 15
    "MODULE_QUERY_NODE_PORT_INFO",  // 16
    "MODULE_DELETE_NODE",           // 17
    // Group 4 – ZCL Control (18-23)
    "MODULE_ZCL_READ_ATTR",         // 18
    "MODULE_ZCL_WRITE_ATTR",        // 19
    "MODULE_ZCL_SEND_CONTROL_CMD",  // 20
    "MODULE_ZCL_RECV_CONTROL_CMD",  // 21
    "MODULE_ZCL_RECV_ATTR_REPORT",  // 22
    "MODULE_ZCL_SET_REPORT_RULE",   // 23
    // Group 5 – Data TX (24-25)
    "MODULE_SEND_UNICAST",          // 24
    "MODULE_SEND_BROADCAST",        // 25
};

/* ============================================================================
 * Internal Helpers
 * ========================================================================== */

static json_zigbee_function_id_t get_function_id(const char *name) {
    for (int i = 0; i < JSON_ZIGBEE_FUNC_MAX; i++) {
        if (strcmp(name, ZIGBEE_FUNCTION_NAMES[i]) == 0) {
            return (json_zigbee_function_id_t)i;
        }
    }
    return JSON_ZIGBEE_FUNC_MAX;
}

/**
 * @brief Parse GPIO control array from a JSON array node.
 */
static esp_err_t parse_gpio_array(cJSON *arr, gpio_control_t *out, uint8_t *count) {
    if (!cJSON_IsArray(arr)) { *count = 0; return ESP_OK; }
    int n = cJSON_GetArraySize(arr);
    if (n > MAX_GPIO_ACTIONS) {
        ESP_LOGE(TAG, "Too many GPIO actions: %d", n);
        return ESP_ERR_INVALID_ARG;
    }
    *count = 0;
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, arr) {
        cJSON *pin   = cJSON_GetObjectItem(item, "pin");
        cJSON *state = cJSON_GetObjectItem(item, "state");
        if (!cJSON_IsString(pin) || !cJSON_IsString(state)) {
            ESP_LOGE(TAG, "Invalid GPIO entry");
            return ESP_ERR_INVALID_ARG;
        }
        strncpy(out[*count].pin, pin->valuestring, MAX_PIN_ID_LEN - 1);
        out[*count].state = (strcmp(state->valuestring, "HIGH") == 0);
        (*count)++;
    }
    return ESP_OK;
}

/**
 * @brief Parse a space-separated hex string into a byte array.
 *        e.g. "55 80 03" -> {0x55, 0x80, 0x03}, len=3
 */
static uint8_t parse_hex_string(const char *hex_str, uint8_t *out, uint8_t max_len) {
    uint8_t count = 0;
    const char *p = hex_str;
    while (*p && count < max_len) {
        while (*p == ' ') p++;          // skip spaces
        if (*p == '\0') break;
        char byte_str[3] = {p[0], (p[1] ? p[1] : '\0'), '\0'};
        out[count++] = (uint8_t)strtol(byte_str, NULL, 16);
        p += 2;
    }
    return count;
}

/**
 * @brief Parse a single function JSON object into a function config struct.
 */
static esp_err_t parse_function(cJSON *func_json,
                                json_zigbee_function_config_t *out) {
    memset(out, 0, sizeof(*out));
    out->cmd_type = -1;
    out->cmd_code = -1;
    strncpy(out->response_format, "ascii", sizeof(out->response_format) - 1);

    // function_name (required)
    cJSON *fn = cJSON_GetObjectItem(func_json, "function_name");
    if (!cJSON_IsString(fn)) {
        ESP_LOGE(TAG, "Missing 'function_name'");
        return ESP_ERR_INVALID_ARG;
    }
    json_zigbee_function_id_t fid = get_function_id(fn->valuestring);
    if (fid == JSON_ZIGBEE_FUNC_MAX) {
        ESP_LOGE(TAG, "Unknown Zigbee function: %s", fn->valuestring);
        return ESP_ERR_INVALID_ARG;
    }
    out->function_id = fid;
    out->available   = true;

    // command (AT mode only, optional)
    cJSON *cmd = cJSON_GetObjectItem(func_json, "command");
    if (cJSON_IsString(cmd)) {
        strncpy(out->command, cmd->valuestring, ZIGBEE_COMMAND_LEN - 1);
    }

    // is_prefix
    cJSON *isp = cJSON_GetObjectItem(func_json, "is_prefix");
    out->is_prefix = cJSON_IsBool(isp) && cJSON_IsTrue(isp);

    // cmd_type (-1 = AT mode)
    cJSON *ctype = cJSON_GetObjectItem(func_json, "cmd_type");
    if (cJSON_IsNumber(ctype)) {
        out->cmd_type = (int8_t)ctype->valueint;
    }

    // cmd_code
    cJSON *ccode = cJSON_GetObjectItem(func_json, "cmd_code");
    if (cJSON_IsNumber(ccode)) {
        out->cmd_code = (int8_t)ccode->valueint;
    }

    // response_format ("ascii" | "hex")
    cJSON *rfmt = cJSON_GetObjectItem(func_json, "response_format");
    if (cJSON_IsString(rfmt)) {
        strncpy(out->response_format, rfmt->valuestring,
                sizeof(out->response_format) - 1);
    }

    // expect_response
    cJSON *er = cJSON_GetObjectItem(func_json, "expect_response");
    if (cJSON_IsString(er)) {
        strncpy(out->expect_response, er->valuestring, ZIGBEE_RESPONSE_LEN - 1);
        if (strcmp(out->response_format, "hex") == 0) {
            out->expect_response_len = parse_hex_string(
                out->expect_response,
                out->expect_response_bytes,
                ZIGBEE_RESPONSE_BYTES);
        }
    }

    // is_async_event
    cJSON *async = cJSON_GetObjectItem(func_json, "is_async_event");
    out->is_async_event = cJSON_IsBool(async) && cJSON_IsTrue(async);

    // timeout
    cJSON *to = cJSON_GetObjectItem(func_json, "timeout");
    out->timeout_ms = cJSON_IsNumber(to) ? (uint16_t)to->valueint : 500;

    // gpio_start_control
    cJSON *gstart = cJSON_GetObjectItem(func_json, "gpio_start_control");
    esp_err_t ret = parse_gpio_array(gstart, out->gpio_start, &out->gpio_start_count);
    if (ret != ESP_OK) return ret;

    // delay_start
    cJSON *ds = cJSON_GetObjectItem(func_json, "delay_start");
    out->delay_start_ms = cJSON_IsNumber(ds) ? (uint16_t)ds->valueint : 0;

    // gpio_end_control
    cJSON *gend = cJSON_GetObjectItem(func_json, "gpio_end_control");
    ret = parse_gpio_array(gend, out->gpio_end, &out->gpio_end_count);
    if (ret != ESP_OK) return ret;

    // delay_end
    cJSON *de = cJSON_GetObjectItem(func_json, "delay_end");
    out->delay_end_ms = cJSON_IsNumber(de) ? (uint16_t)de->valueint : 0;

    ESP_LOGI(TAG, "Parsed: %s (id=%d, cmd_type=%d, cmd_code=%d, fmt=%s)",
             fn->valuestring, fid, out->cmd_type, out->cmd_code, out->response_format);
    return ESP_OK;
}

/* ============================================================================
 * Public API
 * ========================================================================== */

esp_err_t json_zigbee_config_parse(const char *json_str,
                                   json_zigbee_module_config_t *config) {
    if (!json_str || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Parse common metadata
    esp_err_t ret = json_config_parse_metadata(json_str, &config->metadata);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse metadata");
        return ret;
    }

    if (strcmp(config->metadata.module_type, "ZIGBEE") != 0) {
        ESP_LOGE(TAG, "module_type is not ZIGBEE: '%s'",
                 config->metadata.module_type);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "cJSON_Parse failed");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *functions = cJSON_GetObjectItem(root, "functions");
    if (!cJSON_IsArray(functions)) {
        ESP_LOGE(TAG, "Missing 'functions' array");
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    int n = cJSON_GetArraySize(functions);
    if (n > ZIGBEE_MAX_FUNCTIONS) {
        ESP_LOGE(TAG, "Too many functions: %d (max %d)", n, ZIGBEE_MAX_FUNCTIONS);
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    memset(config->functions, 0, sizeof(config->functions));
    // Pre-fill all cmd_type to -1 (AT mode sentinel)
    for (int i = 0; i < ZIGBEE_MAX_FUNCTIONS; i++) {
        config->functions[i].cmd_type = -1;
        config->functions[i].cmd_code = -1;
        strncpy(config->functions[i].response_format, "ascii",
                sizeof(config->functions[i].response_format) - 1);
    }
    config->function_count = 0;

    cJSON *fj = NULL;
    cJSON_ArrayForEach(fj, functions) {
        json_zigbee_function_config_t tmp;
        ret = parse_function(fj, &tmp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to parse function entry");
            cJSON_Delete(root);
            return ret;
        }
        config->functions[tmp.function_id] = tmp;
        config->function_count++;
    }

    ESP_LOGI(TAG, "Zigbee config parsed OK: %d functions", config->function_count);
    cJSON_Delete(root);
    return ESP_OK;
}
