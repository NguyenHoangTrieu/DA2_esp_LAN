/**
 * @file config_handler_zigbee_commands.c
 * @brief Zigbee command parsers implementation
 *
 * Command format : "CFZB:<stack_id>:<command>"
 * JSON format    : "CFZB:JSON:<stack_id>:<json_data>"
 *
 * The command field is the raw AT string (e.g. "AT+INFO?") forwarded verbatim
 * to the Zigbee module.  The loaded JSON config is matched by command string to
 * resolve GPIO sequencing, timeout, and expected response — the same approach
 * used by the LoRa and BLE handlers.
 */

#include "config_handler.h"
#include "config_handler_zigbee_commands.h"
#include "zigbee_handler.h"
#include "zigbee_handler_task.h"
#include "module_monitor_task.h"
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include "frame_types.h"
#include <string.h>
#include <stdlib.h>

#define ZIGBEE_MAX_STACKS 2

static const char *TAG = "zigbee_commands";

/* ============================================================================
 * Unified Zigbee Command Parser
 *
 * New Protocol (function-name based):
 *   CFZB:<stack_id>:<function_name>             (non-prefix, is_prefix=false)
 *   CFZB:<stack_id>:<function_name>:<data>      (prefix, is_prefix=true)
 *
 * Examples:
 *   CFZB:0:MODULE_GET_INFO                      → executes AT+INFO?
 *   CFZB:0:MODULE_START_NETWORK                 → executes AT+CREATENW
 *   CFZB:0:MODULE_SET_PERMIT_JOIN:60            → executes AT+OPENWNET=60
 *   CFZB:0:MODULE_ZCL_SEND_CONTROL_CMD:1234,01,0006,01 → AT+ZCL=1234,01,0006,01
 *
 * Legacy fallback: if command starts with "AT" or doesn't match a function name,
 * falls back to old raw-command pass-through via get_function_by_command().
 * ========================================================================== */

esp_err_t config_parse_zigbee_command(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) {
        ESP_LOGE(TAG, "ZIGBEE CMD: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFZB:" */
    if (strncmp((const char *)data, "CFZB:", 5) != 0) {
        ESP_LOGE(TAG, "ZIGBEE CMD: wrong prefix");
        return ESP_FAIL;
    }

    /* Parse: CFZB:<stack_id>:<function_name_or_command>[:data] */
    const char *ptr   = (const char *)(data + 5);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "ZIGBEE CMD: missing stack_id separator");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id >= ZIGBEE_MAX_STACKS) {
        ESP_LOGE(TAG, "ZIGBEE CMD: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }

    const char *func_start = colon + 1;
    uint16_t    remaining_len = len - (uint16_t)(func_start - (const char *)data);

    if (remaining_len == 0 || remaining_len > 255) {
        ESP_LOGE(TAG, "ZIGBEE CMD: invalid command length %u", remaining_len);
        return ESP_FAIL;
    }

    /* Split function_name and data at first ':' */
    char func_name[64] = {0};
    const char *data_str = NULL;
    uint16_t data_str_len = 0;

    const char *data_colon = memchr(func_start, ':', remaining_len);
    if (data_colon && strncmp(func_start, "MODULE_", 7) == 0) {
        uint16_t name_len = data_colon - func_start;
        if (name_len >= sizeof(func_name)) name_len = sizeof(func_name) - 1;
        memcpy(func_name, func_start, name_len);
        func_name[name_len] = '\0';
        data_str = data_colon + 1;
        data_str_len = remaining_len - name_len - 1;
    } else if (strncmp(func_start, "MODULE_", 7) == 0) {
        uint16_t name_len = remaining_len;
        if (name_len >= sizeof(func_name)) name_len = sizeof(func_name) - 1;
        memcpy(func_name, func_start, name_len);
        func_name[name_len] = '\0';
    }

    zigbee_function_config_t func_config = {0};
    char final_command[256] = {0};

    if (func_name[0] != '\0') {
        /* New function-name-based lookup */
        esp_err_t ret = zigbee_handler_get_function_by_name(stack_id, func_name, &func_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ZIGBEE CMD: No function '%s' for stack %u", func_name, stack_id);
            char err_resp[64];
            int  el = snprintf(err_resp, sizeof(err_resp),
                               "CFZB:%d:FAIL:NO_FUNC:%s", stack_id, func_name);
            if (el > 0) mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, (uint8_t *)err_resp, (uint16_t)el);
            return ESP_FAIL;
        }

        /* Build actual command to send to module */
        if (func_config.is_prefix && data_str && data_str_len > 0) {
            snprintf(final_command, sizeof(final_command), "%s%.*s",
                     func_config.command, data_str_len, data_str);
        } else {
            strncpy(final_command, func_config.command, sizeof(final_command) - 1);
        }

        ESP_LOGI(TAG, "ZIGBEE CMD: func='%s' → cmd='%s' (is_prefix=%d, is_hex=%d)",
                 func_name, final_command, func_config.is_prefix, func_config.is_hex);
    } else {
        /* Legacy fallback: raw command pass-through */
        const char *command = func_start;
        uint16_t cmd_len = remaining_len;

        esp_err_t cfg_ret = zigbee_handler_get_function_by_command(stack_id, command, &func_config);
        if (cfg_ret != ESP_OK) {
            ESP_LOGW(TAG, "ZIGBEE CMD: no config match for '%.*s', using defaults", cmd_len, command);
            func_config.available   = true;
            func_config.is_hex      = false;
            func_config.timeout_ms  = 2000;
        }

        if (cmd_len >= sizeof(final_command)) cmd_len = sizeof(final_command) - 1;
        memcpy(final_command, command, cmd_len);
        final_command[cmd_len] = '\0';
    }

    ESP_LOGI(TAG, "ZIGBEE CMD: stack=%u command='%s'", stack_id, final_command);

    /* Build command request */
    zigbee_command_request_t req = {0};
    req.stack_id    = stack_id;
    uint16_t copy_len = strlen(final_command);
    if (copy_len > sizeof(req.command) - 1)
        copy_len = (uint16_t)(sizeof(req.command) - 1);
    memcpy(req.command, final_command, copy_len);
    req.command[copy_len] = '\0';
    req.command_len = copy_len;
    memcpy(&req.func_config, &func_config, sizeof(zigbee_function_config_t));

    esp_err_t ret = zigbee_handler_task_execute_command(&req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ZIGBEE CMD: queue failed: %s", esp_err_to_name(ret));
        char err[64];
        int  el = snprintf(err, sizeof(err), "CFZB:%d:FAIL:%.*s:QUEUE_FULL",
                           stack_id, copy_len, req.command);
        if (el > 0) mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, (uint8_t *)err, (uint16_t)el);
        return ret;
    }

    ESP_LOGI(TAG, "[Stack %d] Command enqueued: %.*s", stack_id, copy_len, req.command);
    ESP_LOGI(TAG, "ZIGBEE CMD: enqueued successfully");
    return ESP_OK;
}

/* ============================================================================
 * JSON Config Parser
 * ========================================================================== */

esp_err_t config_parse_zigbee_json(const uint8_t *data, uint16_t len) {
    if (!data || len < 15) {
        ESP_LOGE(TAG, "ZIGBEE JSON: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFZB:JSON:" */
    if (strncmp((const char *)data, "CFZB:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "ZIGBEE JSON: wrong prefix");
        return ESP_FAIL;
    }

    const char *ptr   = (const char *)(data + 10);
    uint8_t     stack_id;
    const char *json_data;

    if (ptr[0] == '{') {
      /* Legacy / no-slot-prefix format — use stack 0 */
      stack_id  = 0;
      json_data = ptr;
    } else {
      const char *colon = strchr(ptr, ':');
      if (!colon) {
          ESP_LOGE(TAG, "ZIGBEE JSON: missing separator");
          return ESP_FAIL;
      }
      stack_id  = (uint8_t)atoi(ptr);
      json_data = colon + 1;
    }

    uint16_t    json_len  = len - (uint16_t)(json_data - (const char *)data);

    if (stack_id >= ZIGBEE_MAX_STACKS) {
        ESP_LOGE(TAG, "ZIGBEE JSON: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }
    if (json_len < 2 || json_len > 16384) {
        ESP_LOGE(TAG, "ZIGBEE JSON: invalid length %u", json_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ZIGBEE JSON: stack=%u length=%u", stack_id, json_len);

    /* Ensure module_type is present */
    const char *send_json = json_data;
    uint16_t    send_len  = json_len;
    char       *patched   = NULL;

    if (json_data[0] == '{' && strstr(json_data, "module_type") == NULL) {
        const char *inject = "\"module_type\":\"ZIGBEE\",";
        size_t inject_len  = strlen(inject);
        size_t new_len     = (size_t)json_len + inject_len;
        patched = malloc(new_len + 1);
        if (patched) {
            patched[0] = '{';
            memcpy(patched + 1, inject, inject_len);
            memcpy(patched + 1 + inject_len, json_data + 1, json_len - 1);
            patched[new_len] = '\0';
            send_json = patched;
            send_len  = (uint16_t)new_len;
            ESP_LOGI(TAG, "Injected missing module_type=ZIGBEE into JSON");
        }
    }

    esp_err_t ret = module_monitor_send_config(stack_id, send_json, send_len);
    if (patched) free(patched);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ZIGBEE JSON: module_monitor queue failed: %s",
                 esp_err_to_name(ret));
        uint8_t err_resp[] = "CFZB:JSON:FAIL:QUEUE";
        mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, err_resp, sizeof(err_resp) - 1);
        return ret;
    }

    ESP_LOGI(TAG, "ZIGBEE JSON: forwarded to module_monitor_task");
    return ESP_OK;
}
