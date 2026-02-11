/**
 * @file config_handler_ble_commands.h
 * @brief BLE command parsers - Public declarations
 * 
 * Provides command parsing functions for BLE module control:
 * - SCAN: Streaming discovery
 * - SETUP: Function execution  
 * - JSON: Configuration loading
 */

#ifndef CONFIG_HANDLER_BLE_COMMANDS_H
#define CONFIG_HANDLER_BLE_COMMANDS_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Streaming response callback - forwards BLE responses to WAN MCU
 * 
 * This callback is invoked by ble_execute_function_streaming() for each
 * response received during streaming operations (e.g., SCAN).
 * 
 * @param data Response data from BLE module
 * @param len Length of response
 * @param user_data User context (unused)
 */
void ble_stream_response_to_wan_callback(const uint8_t *data,
                                        uint16_t len,
                                        void *user_data);

/**
 * @brief Parse and execute BLE SCAN command with streaming responses
 * 
 * Format: "CFBL:SCAN:<timeout>:<stack_id>"
 * Example: "CFBL:SCAN:5000:0"
 * 
 * All scan results are streamed back to WAN MCU via callback as they arrive.
 * Sends "BR:SCAN:DONE" marker when completed.
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_scan(const uint8_t *data, uint16_t len);

/**
 * @brief Parse and execute BLE SETUP command
 * 
 * Format: "CFBL:SETUP:<function_id>:<stack_id>:<params>"
 * Example: "CFBL:SETUP:4:0:TestDevice"
 * 
 * Executes function and sends result to WAN MCU as:
 * "BR:SETUP:<func>:<status>:<response>"
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_setup(const uint8_t *data, uint16_t len);

/**
 * @brief Parse and load BLE JSON configuration
 * 
 * Format: "CFBL:JSON:<stack_id>:<json_data>"
 * Example: "CFBL:JSON:0:{"module_type":"BLE",...}"
 * 
 * Loads JSON config into ble_handler and sends "BR:JSON:OK/FAIL" response.
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_json(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_HANDLER_BLE_COMMANDS_H
