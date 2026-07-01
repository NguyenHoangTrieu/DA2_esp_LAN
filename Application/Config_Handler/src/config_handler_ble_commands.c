/**
 * @file config_handler_ble_commands.c
 * @brief BLE command parsers - RAW COMMAND PASS-THROUGH
 * 
 * Implements BLE command parsing and raw command forwarding:
 * - SCAN: Send raw command, stream responses (AT/binary/ASCII agnostic)
 * - SETUP: Send raw command, forward single response (AT/binary/ASCII agnostic)
 * - JSON: Configuration loading
 * 
 * KEY DESIGN: Commands are NOT parsed or interpreted - they are passed
 * directly to the BLE module in whatever format the App sends (AT, binary,
 * ASCII, etc.). The module configuration determines how to interpret them.
 * 
 * All responses are automatically forwarded to WAN MCU for routing
 * to PC App (UART) or Server (MQTT/HTTP).
 */

#include "config_handler.h"
#include "config_handler_ble_commands.h"
#include "ble_handler.h"
#include "ble_handler_task.h"
#include "module_monitor_task.h"
#include "json_ble_config_parser.h"
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include <string.h>
#include <stdlib.h>

#define BLE_MAX_STACKS 2  // Must match ble_handler.c

static const char *TAG = "ble_commands";

/* ===========================================================================
 * Unified BLE Command Parser
 * ========================================================================= */

/**
 * @brief Unified BLE command parser using function names
 * 
 * New Protocol (function-name based):
 *   "CFBL:<stack_id>:<function_name>"           (non-prefix, is_prefix=false)
 *   "CFBL:<stack_id>:<function_name>:<data>"    (prefix, is_prefix=true)
 * 
 * Examples:
 *   "CFBL:0:MODULE_SW_RESET"                   → executes AT+RESET (non-prefix)
 *   "CFBL:0:MODULE_START_DISCOVERY:5000"        → executes AT+SCAN=5000 (prefix, data appended)
 *   "CFBL:0:MODULE_CONNECT:001122334455"        → executes AT+CONNECT=001122334455 (prefix)
 * 
 * Legacy fallback: if command starts with "AT" or contains no matching function name,
 * falls back to the old raw-command pass-through via get_function_by_command().
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_command(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) { // "CFBL:0:X"
        ESP_LOGE(TAG, "BLE CMD: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    // Check prefix
    if (strncmp((const char *)data, "CFBL:", 5) != 0) {
        ESP_LOGE(TAG, "BLE CMD: invalid prefix");
        return ESP_FAIL;
    }

    // Parse: CFBL:<stack_id>:<function_name_or_command>[:data]
    const char *ptr = (const char *)(data + 5);
    
    // Extract stack_id
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "BLE CMD: missing separator");
        return ESP_FAIL;
    }
    
    uint8_t stack_id = atoi(ptr);
    if (stack_id >= BLE_MAX_STACKS) {
        ESP_LOGE(TAG, "BLE CMD: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }
    
    // Extract function name and optional data
    const char *func_start = colon + 1;
    uint16_t remaining_len = len - (func_start - (const char *)data);
    
    if (remaining_len == 0 || remaining_len > 255) {
        ESP_LOGE(TAG, "BLE CMD: invalid command length %u", remaining_len);
        return ESP_FAIL;
    }
    
    // Split function_name and data at first ':'
    // e.g. "MODULE_SET_REGION:EU868" → func_name="MODULE_SET_REGION", data_str="EU868"
    char func_name[64] = {0};
    const char *data_str = NULL;
    uint16_t data_str_len = 0;
    
    const char *data_colon = memchr(func_start, ':', remaining_len);
    if (data_colon && strncmp(func_start, "MODULE_", 7) == 0) {
        // Function name with data suffix
        uint16_t name_len = data_colon - func_start;
        if (name_len >= sizeof(func_name)) name_len = sizeof(func_name) - 1;
        memcpy(func_name, func_start, name_len);
        func_name[name_len] = '\0';
        data_str = data_colon + 1;
        data_str_len = remaining_len - name_len - 1;
    } else if (strncmp(func_start, "MODULE_", 7) == 0) {
        // Function name without data
        uint16_t name_len = remaining_len;
        if (name_len >= sizeof(func_name)) name_len = sizeof(func_name) - 1;
        memcpy(func_name, func_start, name_len);
        func_name[name_len] = '\0';
    }
    
    ble_function_config_t func_config = {0};
    char final_command[256] = {0};
    
    if (func_name[0] != '\0') {
        // New function-name-based lookup
        esp_err_t ret = ble_handler_get_function_by_name(stack_id, func_name, &func_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BLE CMD: No function '%s' for stack %u", func_name, stack_id);
            char err_resp[64];
            int err_len = snprintf(err_resp, sizeof(err_resp),
                                   "CFBL:%d:FAIL:NO_FUNC:%s", stack_id, func_name);
            if (err_len > 0) {
                mcu_wan_enqueue_uplink_local(HANDLER_BLE, (uint8_t *)err_resp, (uint16_t)err_len);
            }
            return ESP_FAIL;
        }
        
        // Build the actual command to send to the module
        if (func_config.is_prefix && data_str && data_str_len > 0) {
            // Prefix command: append data after the stored command
            // e.g. command="AT+SCAN=" + data="5000" → "AT+SCAN=5000"
            snprintf(final_command, sizeof(final_command), "%s%.*s",
                     func_config.command, data_str_len, data_str);
        } else if (!func_config.is_prefix) {
            // Non-prefix command: use stored command as-is
            strncpy(final_command, func_config.command, sizeof(final_command) - 1);
        } else {
            // Prefix but no data — send command as-is (may fail on module)
            strncpy(final_command, func_config.command, sizeof(final_command) - 1);
        }
        
        ESP_LOGI(TAG, "BLE CMD: func='%s' → cmd='%s' (is_prefix=%d, is_hex=%d)",
                 func_name, final_command, func_config.is_prefix, func_config.is_hex);
    } else {
        // Legacy fallback: raw command pass-through (for AT commands sent directly)
        const char *command = func_start;
        uint16_t cmd_len = remaining_len;
        
        esp_err_t ret = ble_handler_get_function_by_command(stack_id, command, &func_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BLE CMD: No matching function for '%.*s'", cmd_len, command);
            char err_resp[64];
            int err_len = snprintf(err_resp, sizeof(err_resp),
                                   "CFBL:%d:FAIL:NO_MATCH", stack_id);
            if (err_len > 0) {
                mcu_wan_enqueue_uplink_local(HANDLER_BLE, (uint8_t *)err_resp, (uint16_t)err_len);
            }
            return ESP_FAIL;
        }
        
        if (cmd_len >= sizeof(final_command)) cmd_len = sizeof(final_command) - 1;
        memcpy(final_command, command, cmd_len);
        final_command[cmd_len] = '\0';
    }
    
    // Build command request
    ble_command_request_t cmd_req = {0};
    cmd_req.stack_id = stack_id;
    cmd_req.is_streaming = false;
    
    uint16_t cmd_len = strlen(final_command);
    if (cmd_len > sizeof(cmd_req.command) - 1) {
        cmd_len = sizeof(cmd_req.command) - 1;
    }
    memcpy(cmd_req.command, final_command, cmd_len);
    cmd_req.command[cmd_len] = '\0';
    cmd_req.command_len = cmd_len;
    
    // Copy entire function config struct
    memcpy(&cmd_req.func_config, &func_config, sizeof(ble_function_config_t));
    
    // Execute command via queue
    esp_err_t ret = ble_handler_task_execute_command(&cmd_req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE CMD: Failed to enqueue command: %s", esp_err_to_name(ret));
        char err_resp[64];
        int err_len = snprintf(err_resp, sizeof(err_resp),
                               "CFBL:%d:FAIL:QUEUE_FULL", stack_id);
        if (err_len > 0) {
            mcu_wan_enqueue_uplink_local(HANDLER_BLE, (uint8_t *)err_resp, (uint16_t)err_len);
        }
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE CMD: Command enqueued successfully");
    return ESP_OK;
}

/* ===========================================================================
 * JSON Config Parser
 * ========================================================================= */

/**
 * @brief Parse and load BLE JSON configuration
 * 
 * Format: "CFBL:JSON:<stack_id>:<json_data>"
 * Example: "CFBL:JSON:0:{"module_type":"BLE",...}"
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_json(const uint8_t *data, uint16_t len) {
  if (!data || len < 14) { // "CFBL:JSON:0:{"
    ESP_LOGE(TAG, "BLE JSON: invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  // Check prefix
  if (strncmp((const char *)data, "CFBL:JSON:", 10) != 0) {
    ESP_LOGE(TAG, "BLE JSON: invalid prefix");
    return ESP_FAIL;
  }

  // Parse: CFBL:JSON:<stack_id>:<json_data>
  const char *ptr = (const char *)(data + 10);
  uint8_t     stack_id;
  const char *json_data;

  if (ptr[0] == '{') {
    /* Legacy / no-slot-prefix format: CFBL:JSON:{...json...}
     * Web app sent JSON directly without prepending "<slot>:".  Use stack 0. */
    stack_id  = 0;
    json_data = ptr;
  } else {
    const char *colon = strchr(ptr, ':');
    if (!colon) {
      ESP_LOGE(TAG, "BLE JSON: missing separator");
      return ESP_FAIL;
    }
    stack_id  = (uint8_t)atoi(ptr);
    json_data = colon + 1;
  }

  uint16_t json_len = len - (uint16_t)(json_data - (const char *)data);


  ESP_LOGI(TAG, "Stack ID: %u, JSON length: %u bytes", stack_id, json_len);


  // Validate
  if (stack_id > 1) {
    ESP_LOGE(TAG, "BLE JSON: invalid stack_id %u", stack_id);
    return ESP_FAIL;
  }
  if (json_len < 2 || json_len > 8912) {
    ESP_LOGE(TAG, "BLE JSON: invalid length %u", json_len);
    return ESP_FAIL;
  }

  /* Ensure module_type is present — inject "BLE" if the sender omitted it */
  const char *send_json = json_data;
  uint16_t    send_len  = json_len;
  char       *patched   = NULL;

  if (json_data[0] == '{' && strstr(json_data, "module_type") == NULL) {
    const char *inject = "\"module_type\":\"BLE\",";
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
      ESP_LOGI(TAG, "Injected missing module_type=BLE into JSON");
    }
  }

  esp_err_t ret = module_monitor_send_config(stack_id, send_json, send_len);
  if (patched) free(patched);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send config to module_monitor: %s", esp_err_to_name(ret));
    uint8_t error_resp[] = "BR:JSON:FAIL:QUEUE";
    mcu_wan_enqueue_uplink_local(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
    return ret;
  }

  ESP_LOGI(TAG, "BLE JSON config forwarded to module_monitor_task");
  // ACK response will be sent by module_monitor_task after processing
  return ESP_OK;
}
