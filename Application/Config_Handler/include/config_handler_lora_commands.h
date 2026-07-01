/**
 * @file config_handler_lora_commands.h
 * @brief LoRa command parsers – Public declarations
 *
 * Provides command parsing functions for LoRa module control:
 *  - Command: raw AT / RAK API command pass-through
 *  - JSON:    configuration loading
 */

#ifndef CONFIG_HANDLER_LORA_COMMANDS_H
#define CONFIG_HANDLER_LORA_COMMANDS_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Unified LoRa command parser using JSON configuration.
 *
 * Format: "CFLR:<stack_id>:<command>"
 * Examples:
 *   "CFLR:0:AT+JOIN"            – prefix/exact match
 *   "CFLR:1:AT+SEND=2:01020304" – prefix match AT+SEND=
 *   "CFLR:0:MODULE_HW_RESET"   – function_name match (GPIO-only)
 *
 * Matches the command against the loaded JSON config (prefix or exact),
 * copies GPIO / delay / timeout settings, and enqueues the request to the
 * downlink task for execution via lora_handler_execute_command_with_config().
 *
 * @param data  Command data buffer (the raw bytes received from WAN MCU)
 * @param len   Buffer length
 * @return ESP_OK on success, ESP_FAIL if command not matched or queue full
 */
esp_err_t config_parse_lora_command(const uint8_t *data, uint16_t len);

/**
 * @brief Parse and load LoRa JSON configuration.
 *
 * Format: "CFLR:JSON:<stack_id>:<json_data>"
 * Example: "CFLR:JSON:0:{"module_type":"LORA",...}"
 *
 * Forwards the JSON payload to module_monitor_task via
 * module_monitor_send_config().  The ACK ("CFLR:JSON:OK" / "CFLR:JSON:FAIL")
 * is sent by module_monitor_task after it has called
 * lora_handler_task_load_config().
 *
 * @param data  Command data buffer
 * @param len   Buffer length
 * @return ESP_OK on success
 */
esp_err_t config_parse_lora_json(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_HANDLER_LORA_COMMANDS_H
