/**
 * @file config_handler_zigbee_commands.h
 * @brief Zigbee command parsers – Public declarations
 *
 * Provides command parsing for Zigbee module control:
 *
 *  Command format : "CFZB:<stack_id>:<func_name>[:<hex_data>]"
 *    func_name    : Zigbee function name (e.g. "MODULE_START_NETWORK")
 *    hex_data     : Optional space-separated hex bytes for frame payload
 *                   e.g. "60 A8 01 00 06"
 *
 *  JSON format    : "CFZB:JSON:<stack_id>:<json_data>"
 */

#ifndef CONFIG_HANDLER_ZIGBEE_COMMANDS_H
#define CONFIG_HANDLER_ZIGBEE_COMMANDS_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parse a Zigbee function command and enqueue it for execution.
 *
 * Format : "CFZB:<stack_id>:<func_name>[:<hex_data>]"
 * Examples:
 *   "CFZB:0:MODULE_START_NETWORK"
 *   "CFZB:0:MODULE_SET_CHANNEL:0F"
 *   "CFZB:0:MODULE_ZCL_SEND_CONTROL_CMD:60 A8 01 00 06 00 01"
 *
 * @param data  Raw command bytes from WAN MCU
 * @param len   Buffer length
 * @return ESP_OK on success, ESP_FAIL on parse error or queue full
 */
esp_err_t config_parse_zigbee_command(const uint8_t *data, uint16_t len);

/**
 * @brief Parse and load Zigbee JSON configuration.
 *
 * Format : "CFZB:JSON:<stack_id>:<json_data>"
 * Example: "CFZB:JSON:0:{"module_type":"ZIGBEE",...}"
 *
 * Forwards JSON to module_monitor_task.
 * ACK "CFZB:JSON:OK" / "CFZB:JSON:FAIL" sent by module_monitor_task.
 *
 * @param data  Raw command bytes
 * @param len   Buffer length
 */
esp_err_t config_parse_zigbee_json(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HANDLER_ZIGBEE_COMMANDS_H */
