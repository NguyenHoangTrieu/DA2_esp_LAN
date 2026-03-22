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

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HANDLER_RS485_COMMANDS_H */
