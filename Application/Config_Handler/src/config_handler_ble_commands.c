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
 * @brief Unified BLE command parser using JSON configuration
 * 
 * Format: "CFBL:<stack_id>:<command>"
 * Example: "CFBL:0:AT+SCAN=5000" (prefix match)
 * Example: "CFBL:1:AT+CONNECT=001122334455" (prefix match AT+CONNECT=)
 * Example: "CFBL:0:HW_RESET" (exact match)
 * 
 * Matches command against JSON config (prefix or exact), extracts GPIO/delays,
 * and executes via command queue.
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

    // Parse: CFBL:<stack_id>:<command>
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
    
    // Extract command string
    const char *command = colon + 1;
    uint16_t cmd_len = len - (command - (const char *)data);
    
    if (cmd_len == 0 || cmd_len > 255) {
        ESP_LOGE(TAG, "BLE CMD: invalid command length %u", cmd_len);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "BLE CMD: stack=%u, command='%.*s'", stack_id, cmd_len, command);
    
    // Get function config matching this command from BLE handler
    ble_function_config_t func_config = {0};
    esp_err_t ret = ble_handler_get_function_by_command(stack_id, command, &func_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE CMD: No matching function for '%.*s'", cmd_len, command);
        // Notify App: command not recognized by JSON config
        char err_resp[64];
        int err_len = snprintf(err_resp, sizeof(err_resp),
                               "CFBL:%d:FAIL:NO_MATCH", stack_id);
        if (err_len > 0) {
            mcu_wan_enqueue_uplink(HANDLER_BLE, (uint8_t *)err_resp, (uint16_t)err_len);
        }
        return ESP_FAIL;
    }
    
    // Build command request - reuse ble_function_config_t struct
    ble_command_request_t cmd_req = {0};
    cmd_req.stack_id = stack_id;
    cmd_req.is_streaming = false; // For now, no streaming support
    
    // Copy command string
    if (cmd_len > sizeof(cmd_req.command) - 1) {
        cmd_len = sizeof(cmd_req.command) - 1;
    }
    memcpy(cmd_req.command, command, cmd_len);
    cmd_req.command[cmd_len] = '\0';
    cmd_req.command_len = cmd_len;
    
    // Copy entire function config struct (no field duplication!)
    memcpy(&cmd_req.func_config, &func_config, sizeof(ble_function_config_t));
    
    // Execute command via queue
    ret = ble_handler_task_execute_command(&cmd_req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE CMD: Failed to enqueue command: %s", esp_err_to_name(ret));
        // Notify App: command queue full or handler not running
        char err_resp[64];
        int err_len = snprintf(err_resp, sizeof(err_resp),
                               "CFBL:%d:FAIL:QUEUE_FULL", stack_id);
        if (err_len > 0) {
            mcu_wan_enqueue_uplink(HANDLER_BLE, (uint8_t *)err_resp, (uint16_t)err_len);
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
    mcu_wan_enqueue_uplink(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
    return ret;
  }

  ESP_LOGI(TAG, "BLE JSON config forwarded to module_monitor_task");
  // ACK response will be sent by module_monitor_task after processing
  return ESP_OK;
}
