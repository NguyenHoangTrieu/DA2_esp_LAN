/**
 * @file config_handler_ble_gatt_commands.h
 * @brief BLE GATT Central command parser — CFBG: prefix router.
 */

#ifndef CONFIG_HANDLER_BLE_GATT_COMMANDS_H
#define CONFIG_HANDLER_BLE_GATT_COMMANDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parse and route "CFBG:JSON:<slot>:<json>" to ble_gatt_handler_load_config().
 *
 * @param data  Raw command string
 * @param len   Length in bytes
 */
void config_parse_ble_gatt_json(const char *data, uint16_t len);

/**
 * @brief Parse and route "CFBG:<slot>:<verb>[:<params>]" to ble_gatt_handler_execute().
 *
 * @param data  Raw command string
 * @param len   Length in bytes
 */
void config_parse_ble_gatt_command(const char *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HANDLER_BLE_GATT_COMMANDS_H */
