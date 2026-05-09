/**
 * @file config_handler_ble_native_commands.h
 * @brief Config handler integration for BLE Native (ESP32 direct BLE Mesh).
 *
 * Mirrors the pattern of config_handler_ble_commands.h.
 *
 * Command prefixes handled:
 *   "CFBN:JSON:<stack_id>:<json>"  → config_parse_ble_native_json()
 *   "CFBN:<stack_id>:<verb>:…"     → config_parse_ble_native_command()
 */

#ifndef CONFIG_HANDLER_BLE_NATIVE_COMMANDS_H
#define CONFIG_HANDLER_BLE_NATIVE_COMMANDS_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parse and route a BLE native control command.
 *
 * Format: "CFBN:<stack_id>:<verb>[:<params>]"
 *
 * Validates the prefix and stack_id, then routes the full raw command to
 * ble_native_handler_execute() which enqueues it for the downlink task.
 *
 * @param data  Command data buffer
 * @param len   Length in bytes
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_native_command(const uint8_t *data, uint16_t len);

/**
 * @brief Parse and load a BLE native JSON configuration.
 *
 * Format: "CFBN:JSON:<stack_id>:<json_object>"
 *
 * Validates the prefix, extracts the JSON payload, and calls
 * ble_native_handler_load_config() to parse and apply the mesh keys and
 * command table.
 *
 * @param data  Command data buffer
 * @param len   Length in bytes
 * @return ESP_OK on success
 */
esp_err_t config_parse_ble_native_json(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HANDLER_BLE_NATIVE_COMMANDS_H */
