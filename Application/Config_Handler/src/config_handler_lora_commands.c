/**
 * @file config_handler_lora_commands.c
 * @brief LoRa command parsers – RAW COMMAND PASS-THROUGH
 *
 * Mirrors config_handler_ble_commands.c with LoRa prefix "CFLR:" and
 * LoRa-specific task/handler types.
 *
 * KEY DESIGN: Commands are NOT interpreted here – they are passed through
 * verbatim to the LoRa module.  The JSON config loaded in lora_handler
 * determines GPIO sequencing, timing, and response matching.
 */

#include "config_handler.h"
#include "config_handler_lora_commands.h"
#include "lora_handler.h"
#include "lora_handler_task.h"
#include "module_monitor_task.h"
#include "json_lora_config_parser.h"
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include "frame_types.h"
#include <string.h>
#include <stdlib.h>

#define LORA_MAX_STACKS 2

static const char *TAG = "lora_commands";

/* ===========================================================================
 * Unified LoRa Command Parser
 * ========================================================================= */

/**
 * @brief Unified LoRa command parser using function names
 *
 * New Protocol (function-name based):
 *   "CFLR:<stack_id>:<function_name>"           (non-prefix, is_prefix=false)
 *   "CFLR:<stack_id>:<function_name>:<data>"    (prefix, is_prefix=true)
 *
 * Examples:
 *   "CFLR:0:MODULE_SW_RESET"                   → executes AT+RESET
 *   "CFLR:0:MODULE_SET_REGION:EU868"            → executes AT+DR EU868
 *   "CFLR:0:MODULE_SET_DEVEUI:0011223344556677" → executes AT+ID DevEui,0011223344556677
 *
 * Legacy fallback: if command starts with "AT" or doesn't match a function name,
 * falls back to old raw-command pass-through via get_function_by_command().
 */
esp_err_t config_parse_lora_command(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) { /* minimum: "CFLR:0:X" */
        ESP_LOGE(TAG, "LORA CMD: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFLR:" */
    if (strncmp((const char *)data, "CFLR:", 5) != 0) {
        ESP_LOGE(TAG, "LORA CMD: invalid prefix");
        return ESP_FAIL;
    }

    /* Parse: CFLR:<stack_id>:<function_name_or_command>[:data] */
    const char *ptr   = (const char *)(data + 5);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "LORA CMD: missing separator");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id >= LORA_MAX_STACKS) {
        ESP_LOGE(TAG, "LORA CMD: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }

    const char *func_start = colon + 1;
    uint16_t    remaining_len = len - (uint16_t)(func_start - (const char *)data);

    if (remaining_len == 0 || remaining_len > 255) {
        ESP_LOGE(TAG, "LORA CMD: invalid command length %u", remaining_len);
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

    lora_function_config_t func_config = {0};
    char final_command[256] = {0};

    if (func_name[0] != '\0') {
        /* New function-name-based lookup */
        esp_err_t ret = lora_handler_get_function_by_name(stack_id, func_name, &func_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "LORA CMD: No function '%s' for stack %u", func_name, stack_id);
            char err_resp[64];
            int  err_len = snprintf(err_resp, sizeof(err_resp),
                                    "CFLR:%d:FAIL:NO_FUNC:%s", stack_id, func_name);
            if (err_len > 0) {
                mcu_wan_enqueue_uplink_local(HANDLER_LORA, (uint8_t *)err_resp, (uint16_t)err_len);
            }
            return ESP_FAIL;
        }

        /* Build actual command to send to module */
        if (func_config.is_prefix && data_str && data_str_len > 0) {
            snprintf(final_command, sizeof(final_command), "%s%.*s",
                     func_config.command, data_str_len, data_str);
        } else {
            strncpy(final_command, func_config.command, sizeof(final_command) - 1);
        }

        ESP_LOGI(TAG, "LORA CMD: func='%s' → cmd='%s' (is_prefix=%d, is_hex=%d)",
                 func_name, final_command, func_config.is_prefix, func_config.is_hex);
    } else {
        /* Legacy fallback: raw command pass-through */
        const char *command = func_start;
        uint16_t cmd_len = remaining_len;

        esp_err_t ret = lora_handler_get_function_by_command(stack_id, command, &func_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "LORA CMD: no matching function for '%.*s'", cmd_len, command);
            char err_resp[64];
            int  err_len = snprintf(err_resp, sizeof(err_resp),
                                    "CFLR:%d:FAIL:NO_MATCH", stack_id);
            if (err_len > 0) {
                mcu_wan_enqueue_uplink_local(HANDLER_LORA, (uint8_t *)err_resp, (uint16_t)err_len);
            }
            return ESP_FAIL;
        }

        if (cmd_len >= sizeof(final_command)) cmd_len = sizeof(final_command) - 1;
        memcpy(final_command, command, cmd_len);
        final_command[cmd_len] = '\0';
    }

    /* Build command request */
    lora_command_request_t cmd_req = {0};
    cmd_req.stack_id    = stack_id;
    cmd_req.is_streaming = false;

    uint16_t copy_len = strlen(final_command);
    if (copy_len > sizeof(cmd_req.command) - 1)
        copy_len = sizeof(cmd_req.command) - 1;
    memcpy(cmd_req.command, final_command, copy_len);
    cmd_req.command[copy_len] = '\0';
    cmd_req.command_len = copy_len;

    memcpy(&cmd_req.func_config, &func_config, sizeof(lora_function_config_t));

    /* Enqueue to downlink task */
    esp_err_t ret = lora_handler_task_execute_command(&cmd_req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LORA CMD: failed to enqueue: %s", esp_err_to_name(ret));
        char err_resp[64];
        int  err_len = snprintf(err_resp, sizeof(err_resp),
                                "CFLR:%d:FAIL:QUEUE_FULL", stack_id);
        if (err_len > 0) {
            mcu_wan_enqueue_uplink_local(HANDLER_LORA, (uint8_t *)err_resp, (uint16_t)err_len);
        }
        return ret;
    }

    ESP_LOGI(TAG, "LORA CMD: command enqueued successfully");
    return ESP_OK;
}

/* ===========================================================================
 * JSON Config Parser
 * ========================================================================= */

esp_err_t config_parse_lora_json(const uint8_t *data, uint16_t len) {
    if (!data || len < 15) { /* minimum: "CFLR:JSON:0:{}" */
        ESP_LOGE(TAG, "LORA JSON: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFLR:JSON:" */
    if (strncmp((const char *)data, "CFLR:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "LORA JSON: invalid prefix");
        return ESP_FAIL;
    }

    /* Parse: CFLR:JSON:<stack_id>:<json_data> */
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
          ESP_LOGE(TAG, "LORA JSON: missing separator");
          return ESP_FAIL;
      }
      stack_id  = (uint8_t)atoi(ptr);
      json_data = colon + 1;
    }

    uint16_t    json_len  = len - (uint16_t)(json_data - (const char *)data);

    ESP_LOGI(TAG, "LORA JSON: stack=%u, length=%u bytes", stack_id, json_len);

    if (stack_id >= LORA_MAX_STACKS) {
        ESP_LOGE(TAG, "LORA JSON: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }
    if (json_len < 2 || json_len > 16*1024) {
        ESP_LOGE(TAG, "LORA JSON: invalid length %u", json_len);
        return ESP_FAIL;
    }

    /* Ensure module_type is present */
    const char *send_json = json_data;
    uint16_t    send_len  = json_len;
    char       *patched   = NULL;

    if (json_data[0] == '{' && strstr(json_data, "module_type") == NULL) {
        const char *inject = "\"module_type\":\"LORA\",";
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
            ESP_LOGI(TAG, "Injected missing module_type=LORA into JSON");
        }
    }

    esp_err_t ret = module_monitor_send_config(stack_id, send_json, send_len);
    if (patched) free(patched);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LORA JSON: failed to forward to module_monitor: %s",
                 esp_err_to_name(ret));
        /* Notify App – module_monitor not running or queue full */
        uint8_t err_resp[] = "LR:JSON:FAIL:QUEUE";
        mcu_wan_enqueue_uplink_local(HANDLER_LORA, err_resp, sizeof(err_resp) - 1);
        return ret;
    }

    ESP_LOGI(TAG, "LORA JSON: config forwarded to module_monitor_task");
    /* ACK ("CFLR:JSON:OK" / "CFLR:JSON:FAIL") sent by module_monitor_task */
    return ESP_OK;
}
