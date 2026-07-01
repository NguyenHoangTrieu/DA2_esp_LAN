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
 * @brief Unified BLE command parser using JSON configuration
 * 
 * Format: "CFBL:<stack_id>:<command>"
 * Example: "CFBL:0:AT+SCAN" (matches prefix AT+SCAN in JSON)
 * Example: "CFBL:1:AT+CONNECT=001122334455" (matches prefix AT+CONNECT=)
 * Example: "CFBL:0:HW_RESET" (exact match)
 * 
 * Matches command against JSON config (prefix or exact match), extracts
 * GPIO controls and delays from JSON, then executes via command queue.
 * 
 * @param data Command data buffer
 * @param len Length of command
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_command(const uint8_t *data, uint16_t len);

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
