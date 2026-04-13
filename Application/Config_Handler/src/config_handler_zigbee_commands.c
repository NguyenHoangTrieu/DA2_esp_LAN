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
 * Format: CFZB:<stack_id>:<command>
 * Example: CFZB:0:AT+INFO?
 *
 * The command string is matched against the loaded JSON config to resolve
 * GPIO sequencing, timeout, and expected response.  If no config entry matches,
 * a sensible default (no GPIO, 2 s timeout, no expected response) is used so
 * the command is still forwarded to the module.
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

    /* Parse: CFZB:<stack_id>:<command> */
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

    const char *command = colon + 1;
    uint16_t    cmd_len = len - (uint16_t)(command - (const char *)data);

    if (cmd_len == 0 || cmd_len > 255) {
        ESP_LOGE(TAG, "ZIGBEE CMD: invalid command length %u", cmd_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ZIGBEE CMD: stack=%u command='%.*s'", stack_id, cmd_len, command);

    /* Match command against loaded JSON config (GPIO, timeout, expect_response) */
    zigbee_function_config_t func_config = {0};
    esp_err_t cfg_ret = zigbee_handler_get_function_by_command(stack_id, command, &func_config);
    if (cfg_ret != ESP_OK) {
        /* No JSON config match — use safe defaults so the command is still sent */
        ESP_LOGW(TAG, "ZIGBEE CMD: no config match for '%.*s', using defaults", cmd_len, command);
        func_config.available   = true;
        func_config.is_hex      = false;
        func_config.timeout_ms  = 2000;
        /* expect_response left empty — accept any response */
    }

    /* Build command request */
    zigbee_command_request_t req = {0};
    req.stack_id    = stack_id;
    uint16_t copy_len = (cmd_len > sizeof(req.command) - 1)
                         ? (uint16_t)(sizeof(req.command) - 1) : cmd_len;
    memcpy(req.command, command, copy_len);
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
