/**
 * @file config_handler_rs485_commands.h
 * @brief RS485 configuration command handlers
 *
 * Exposes parsers for:
 *   CFRS:BR:<baud>            — set baud rate (existing)
 *   CFRS:JSON:<stack_id>:<json> — load GPIO mode config
 */

#ifndef CONFIG_HANDLER_RS485_COMMANDS_H
#define CONFIG_HANDLER_RS485_COMMANDS_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parse and apply RS485 JSON GPIO configuration
 *
 * Format: "CFRS:JSON:<stack_id>:<json_data>"
 * Example: "CFRS:JSON:0:{\"module_type\":\"RS485\",...}"
 *
 * Parses GPIO pin assignments for RS485_SEND_MODE and RS485_RECEIVE_MODE
 * and stores them in the RS485 comm driver for runtime use.
 *
 * @param data   Command buffer (starts at 'C')
 * @param len    Buffer length
 * @return ESP_OK on success
 */
esp_err_t config_parse_rs485_json(const uint8_t *data, uint16_t len);

/**
 * @brief Apply raw RS485 JSON GPIO configuration for a stack
 *
 * This helper is used both at runtime and when restoring RS485 JSON from NVS.
 * The input is the JSON body only, without the "CFRS:JSON:<stack_id>:" prefix.
 *
 * @param stack_id Stack ID (0 or 1)
 * @param json_data Raw JSON buffer
 * @param json_len JSON length
 * @return ESP_OK on success
 */
esp_err_t config_apply_rs485_json_config(uint8_t stack_id, const char *json_data,
										 uint16_t json_len);

/**
 * @brief Parse and send RS485 downlink data
 *
 * Format: "CFRS:<stack_id>:DATA:<hex_data>"
 * Example: "CFRS:0:DATA:010306000A" → sends hex bytes 01 03 06 00 0A
 *
 * Converts hex string to binary and enqueues for RS485 transmission.
 *
 * @param data   Command buffer (starts at 'C')
 * @param len    Buffer length
 * @return ESP_OK on success
 */
esp_err_t config_parse_rs485_downlink(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HANDLER_RS485_COMMANDS_H */
