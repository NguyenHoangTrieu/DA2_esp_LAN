/**
 * @file json_rs485_config_parser.c
 * @brief RS485 JSON configuration parser implementation
 *
 * Parses GPIO-only mode-switching config for RS485 DE/RE pin control.
 * Format: CFRS:JSON:0:{...}  (stack_id in JSON body or from command prefix)
 */

#include "json_rs485_config_parser.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RS485_PARSER";

/* ============================================================================
 * Internal Constants
 * ========================================================================== */

static const char *RS485_FUNCTION_NAMES[JSON_RS485_FUNC_MAX] = {
    "RS485_SEND_MODE",
    "RS485_RECEIVE_MODE",
};

/* ============================================================================
 * Internal Helpers
 * ========================================================================== */

static json_rs485_function_id_t get_rs485_function_id(const char *name) {
    for (int i = 0; i < JSON_RS485_FUNC_MAX; i++) {
        if (strcmp(name, RS485_FUNCTION_NAMES[i]) == 0) {
            return (json_rs485_function_id_t)i;
        }
    }
    return JSON_RS485_FUNC_MAX;
}

static esp_err_t parse_gpio_array(cJSON *gpio_array, gpio_control_t *gpio_out,
                                  uint8_t *count) {
    *count = 0;
    if (!cJSON_IsArray(gpio_array)) {
        return ESP_OK; // Empty array is valid
    }

    int arr_size = cJSON_GetArraySize(gpio_array);
    if (arr_size > RS485_MAX_GPIO_ACTIONS) {
        ESP_LOGE(TAG, "Too many GPIO actions: %d (max %d)", arr_size, RS485_MAX_GPIO_ACTIONS);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, gpio_array) {
        cJSON *pin   = cJSON_GetObjectItem(item, "pin");
        cJSON *state = cJSON_GetObjectItem(item, "state");

        if (!cJSON_IsString(pin) || !cJSON_IsString(state)) {
            ESP_LOGE(TAG, "Invalid GPIO pin/state field");
            return ESP_ERR_INVALID_ARG;
        }

        strncpy(gpio_out[*count].pin, pin->valuestring, MAX_PIN_ID_LEN - 1);
        gpio_out[*count].pin[MAX_PIN_ID_LEN - 1] = '\0';

        if (strcmp(state->valuestring, "HIGH") == 0) {
            gpio_out[*count].state = true;
        } else if (strcmp(state->valuestring, "LOW") == 0) {
            gpio_out[*count].state = false;
        } else {
            ESP_LOGE(TAG, "Invalid GPIO state: %s", state->valuestring);
            return ESP_ERR_INVALID_ARG;
        }
        (*count)++;
    }
    return ESP_OK;
}

/* ============================================================================
 * Public API
 * ========================================================================== */

esp_err_t json_rs485_config_parse(const char *json_str, uint16_t json_len,
                                  json_rs485_module_config_t *out_config) {
    if (!json_str || json_len < 2 || !out_config) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_config, 0, sizeof(json_rs485_module_config_t));

    cJSON *root = cJSON_ParseWithLength(json_str, json_len);
    if (!root) {
        const char *err = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "JSON parse error near: %s", err ? err : "unknown");
        return ESP_FAIL;
    }

    /* module_type validation */
    cJSON *mod_type = cJSON_GetObjectItem(root, "module_type");
    if (cJSON_IsString(mod_type) && strcmp(mod_type->valuestring, "RS485") != 0) {
        ESP_LOGE(TAG, "Invalid module_type: %s (expected RS485)", mod_type->valuestring);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    /* module_id */
    cJSON *mod_id = cJSON_GetObjectItem(root, "module_id");
    if (cJSON_IsString(mod_id)) {
        strncpy(out_config->module_id, mod_id->valuestring, MAX_MODULE_ID_LEN - 1);
        out_config->module_id[MAX_MODULE_ID_LEN - 1] = '\0';
    }

    /* stack_id */
    cJSON *stack_id = cJSON_GetObjectItem(root, "stack_id");
    if (cJSON_IsNumber(stack_id)) {
        out_config->stack_id = (uint8_t)stack_id->valueint;
    }

    /* functions array */
    cJSON *functions = cJSON_GetObjectItem(root, "functions");
    if (!cJSON_IsArray(functions)) {
        ESP_LOGE(TAG, "Missing 'functions' array");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    int parsed = 0;
    cJSON *func = NULL;
    cJSON_ArrayForEach(func, functions) {
        cJSON *fn_name = cJSON_GetObjectItem(func, "function_name");
        if (!cJSON_IsString(fn_name)) continue;

        json_rs485_function_id_t fid = get_rs485_function_id(fn_name->valuestring);
        if (fid == JSON_RS485_FUNC_MAX) {
            ESP_LOGW(TAG, "Unknown RS485 function: %s (skipping)", fn_name->valuestring);
            continue;
        }

        json_rs485_function_config_t *fc = &out_config->functions[fid];
        fc->function_id = fid;
        fc->available   = true;

        /* gpio_start_control */
        cJSON *gpio_start = cJSON_GetObjectItem(func, "gpio_start_control");
        esp_err_t ret = parse_gpio_array(gpio_start, fc->gpio_start, &fc->gpio_start_count);
        if (ret != ESP_OK) {
            cJSON_Delete(root);
            return ret;
        }

        /* delay_start */
        cJSON *dstart = cJSON_GetObjectItem(func, "delay_start");
        fc->delay_start_ms = cJSON_IsNumber(dstart) ? (uint16_t)dstart->valueint : 0;

        /* gpio_end_control */
        cJSON *gpio_end = cJSON_GetObjectItem(func, "gpio_end_control");
        ret = parse_gpio_array(gpio_end, fc->gpio_end, &fc->gpio_end_count);
        if (ret != ESP_OK) {
            cJSON_Delete(root);
            return ret;
        }

        /* delay_end */
        cJSON *dend = cJSON_GetObjectItem(func, "delay_end");
        fc->delay_end_ms = cJSON_IsNumber(dend) ? (uint16_t)dend->valueint : 0;

        parsed++;
        ESP_LOGI(TAG, "Parsed RS485 function: %s (gpio_start=%d, gpio_end=%d)",
                 fn_name->valuestring, fc->gpio_start_count, fc->gpio_end_count);
    }

    cJSON_Delete(root);

    if (parsed == 0) {
        ESP_LOGE(TAG, "No valid RS485 functions found in JSON");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RS485 JSON config parsed: stack=%d, functions=%d",
             out_config->stack_id, parsed);
    return ESP_OK;
}
