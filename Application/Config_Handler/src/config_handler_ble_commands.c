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
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ble_commands";

/* ===========================================================================
 * Response Forwarding Callback
 * ========================================================================= */

/**
 * @brief Streaming response callback - forwards BLE responses to WAN MCU
 * 
 * Called by ble_execute_function_streaming() for each response received.
 * Frames response and sends to WAN MCU via uplink queue.
 * 
 * @param data Response data from BLE module
 * @param len Length of response
 * @param user_data User context (unused)
 */
void ble_stream_response_to_wan_callback(const uint8_t *data,
                                        uint16_t len,
                                        void *user_data) {
  if (!data || len == 0) {
    return;
  }

  // Frame: "BR:" (BLE Response) + data
  uint8_t packet[260]; // 2 (prefix) + 256 (max response) + margin
  packet[0] = 'B';
  packet[1] = 'R';  // BLE Response marker
  
  // Copy response data
  if (len > sizeof(packet) - 2) {
    len = sizeof(packet) - 2; // Truncate if too long
  }
  memcpy(&packet[2], data, len);

  // Send to WAN MCU via uplink queue
  esp_err_t ret = mcu_wan_enqueue_uplink(HANDLER_BLE, packet, len + 2);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to forward BLE response to WAN: %s",
             esp_err_to_name(ret));
  } else {
    ESP_LOGD(TAG, "→ WAN: %.*s (%u bytes)", len, data, len);
  }
}

/* ===========================================================================
 * SCAN Command Parser with Streaming
 * ========================================================================= */

/**
 * @brief Parse and execute BLE SCAN command with streaming responses
 * 
 * Format: "CFBL:SCAN:<stack_id>:<raw_command>:<timeout>"
 * Example: "CFBL:SCAN:0:AT+SCAN:5000" (AT command)
 * Example: "CFBL:SCAN:1:\x01\x05:3000" (binary command)
 * 
 * Raw command can be any format (AT, binary, ASCII) - module-specific.
 * All responses are streamed back to WAN MCU via callback.
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_scan(const uint8_t *data, uint16_t len) {
  if (!data || len < 14) { // "CFBL:SCAN:0:X:1000"
    ESP_LOGE(TAG, "BLE SCAN: invalid parameters (len=%u)", len);
    return ESP_ERR_INVALID_ARG;
  }

  // Check prefix
  if (strncmp((const char *)data, "CFBL:SCAN:", 10) != 0) {
    ESP_LOGE(TAG, "BLE SCAN: invalid prefix");
    return ESP_FAIL;
  }

  // Parse: CFBL:SCAN:<stack_id>:<raw_command>:<timeout>
  const char *ptr = (const char *)(data + 10);
  
  // Extract stack_id
  const char *colon1 = strchr(ptr, ':');
  if (!colon1) {
    ESP_LOGE(TAG, "BLE SCAN: missing separator");
    return ESP_FAIL;
  }
  uint8_t stack_id = atoi(ptr);
  
  // Extract raw command (thay vì function_id)
  const uint8_t *raw_command = (const uint8_t *)(colon1 + 1);
  const char *colon2 = strchr(colon1 + 1, ':');
  
  uint16_t cmd_len;
  uint32_t timeout_ms = 5000; // Default
  
  if (colon2) {
    cmd_len = colon2 - (colon1 + 1);
    timeout_ms = atoi(colon2 + 1);
  } else {
    // No timeout specified, use full remaining as command
    cmd_len = len - (raw_command - data);
  }

  // Validate
  if (stack_id > 1) {
    ESP_LOGE(TAG, "BLE SCAN: invalid stack_id %u", stack_id);
    return ESP_FAIL;
  }
  if (cmd_len == 0 || cmd_len > 256) {
    ESP_LOGE(TAG, "BLE SCAN: invalid command length %u", cmd_len);
    return ESP_FAIL;
  }
  if (timeout_ms < 1000 || timeout_ms > 30000) {
    ESP_LOGW(TAG, "BLE SCAN: timeout %lu ms out of range, using 5000", timeout_ms);
    timeout_ms = 5000;
  }

  ESP_LOGI(TAG, "Starting BLE SCAN (stack=%u, cmd_len=%u, timeout=%lu ms)", 
           stack_id, cmd_len, timeout_ms);
  ESP_LOG_BUFFER_HEXDUMP(TAG, raw_command, cmd_len, ESP_LOG_DEBUG);

  // Send raw command to BLE module and stream responses
  // TODO: Implement ble_send_raw_command_streaming() in ble_handler
  // For now, use module_config_controller directly if available
  esp_err_t ret = ble_send_raw_command_streaming(
      stack_id,
      raw_command,
      cmd_len,
      timeout_ms,
      ble_stream_response_to_wan_callback,
      NULL
  );

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BLE SCAN failed: %s", esp_err_to_name(ret));
    
    // Send error response
    uint8_t error_resp[] = "BR:SCAN:ERROR";
    mcu_wan_enqueue_uplink(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
    return ret;
  }

  ESP_LOGI(TAG, "BLE SCAN completed");
  
  // Send completion marker
  uint8_t done_resp[] = "BR:SCAN:DONE";
  mcu_wan_enqueue_uplink(HANDLER_BLE, done_resp, sizeof(done_resp) - 1);

  return ESP_OK;
}

/* ===========================================================================
 * SETUP Command Parser
 * ========================================================================= */

/**
 * @brief Parse and execute BLE SETUP command (raw command pass-through)
 * 
 * Format: "CFBL:SETUP:<stack_id>:<raw_command>"
 * Example: "CFBL:SETUP:0:AT+NAME=TestDevice" (AT command)
 * Example: "CFBL:SETUP:1:\x01\x04\x00TestDevice" (binary command)
 * Example: "CFBL:SETUP:0:name TestDevice" (ASCII command)
 * 
 * Raw command can be any format - module interprets based on its type.
 * Result is sent back to WAN MCU as: "BR:SETUP:<status>:<response>"
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_setup(const uint8_t *data, uint16_t len) {
  if (!data || len < 14) { // "CFBL:SETUP:0:X"
    ESP_LOGE(TAG, "BLE SETUP: invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  // Check prefix
  if (strncmp((const char *)data, "CFBL:SETUP:", 11) != 0) {
    ESP_LOGE(TAG, "BLE SETUP: invalid prefix");
    return ESP_FAIL;
  }

  // Parse: CFBL:SETUP:<stack_id>:<raw_command>
  const char *ptr = (const char *)(data + 11);
  const char *colon = strchr(ptr, ':');
  if (!colon) {
    ESP_LOGE(TAG, "BLE SETUP: missing separator");
    return ESP_FAIL;
  }

  uint8_t stack_id = atoi(ptr);
  
  // Extract raw command (không parse function_id hay params)
  const uint8_t *raw_command = (const uint8_t *)(colon + 1);
  uint16_t cmd_len = len - (raw_command - data);

  // Validate
  if (stack_id > 1) {
    ESP_LOGE(TAG, "BLE SETUP: invalid stack_id %u", stack_id);
    return ESP_FAIL;
  }
  if (cmd_len == 0 || cmd_len > 256) {
    ESP_LOGE(TAG, "BLE SETUP: invalid command length %u", cmd_len);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Executing BLE SETUP (stack=%u, cmd_len=%u)", stack_id, cmd_len);
  ESP_LOG_BUFFER_HEXDUMP(TAG, raw_command, cmd_len, ESP_LOG_DEBUG);

  // Send raw command to BLE module (pass-through)
  uint8_t response[256] = {0};
  uint16_t resp_len = 0;
  
  // TODO: Implement ble_send_raw_command() in ble_handler
  // For now, use module_config_controller directly if available
  esp_err_t ret = ble_send_raw_command(
      stack_id,
      raw_command,
      cmd_len,
      response,
      &resp_len,
      3000  // 3 second timeout
  );

  // Format response: "BR:SETUP:<status>:<response>"
  uint8_t full_response[300];
  const char *status = (ret == ESP_OK) ? "OK" : "FAIL";
  
  // Build response frame
  int header_len = snprintf((char *)full_response, sizeof(full_response),
                           "BR:SETUP:%s:", status);
  
  if (header_len < 0 || header_len >= sizeof(full_response)) {
    ESP_LOGE(TAG, "BLE SETUP: response header overflow");
    return ESP_FAIL;
  }
  
  // Append actual response (có thể là binary)
  uint16_t available = sizeof(full_response) - header_len;
  if (resp_len > available) {
    resp_len = available;
  }
  memcpy(&full_response[header_len], response, resp_len);
  uint16_t total_len = header_len + resp_len;

  // Send result to WAN MCU
  esp_err_t send_ret = mcu_wan_enqueue_uplink(HANDLER_BLE, full_response, total_len);
  if (send_ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send BLE SETUP result to WAN");
  }

  ESP_LOGI(TAG, "BLE SETUP completed (status=%s, resp_len=%u)", status, resp_len);
  return ret;
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
  const char *colon = strchr(ptr, ':');
  if (!colon) {
    ESP_LOGE(TAG, "BLE JSON: missing separator");
    return ESP_FAIL;
  }

  uint8_t stack_id = atoi(ptr);
  const char *json_data = colon + 1;
  uint16_t json_len = len - (json_data - (const char *)data);

  // Validate
  if (stack_id > 1) {
    ESP_LOGE(TAG, "BLE JSON: invalid stack_id %u", stack_id);
    return ESP_FAIL;
  }
  if (json_len < 2 || json_len > 4096) {
    ESP_LOGE(TAG, "BLE JSON: invalid length %u", json_len);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Loading BLE JSON config (stack=%u, %u bytes)", 
           stack_id, json_len);

  // Load configuration
  esp_err_t ret = ble_handler_load_config(stack_id, json_data, json_len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to load BLE config: %s", esp_err_to_name(ret));
    
    // Send error response
    uint8_t error_resp[] = "BR:JSON:FAIL";
    mcu_wan_enqueue_uplink(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
    return ret;
  }

  ESP_LOGI(TAG, "BLE JSON config loaded successfully");
  
  // Send success response
  uint8_t ok_resp[] = "BR:JSON:OK";
  mcu_wan_enqueue_uplink(HANDLER_BLE, ok_resp, sizeof(ok_resp) - 1);

  return ESP_OK;
}
