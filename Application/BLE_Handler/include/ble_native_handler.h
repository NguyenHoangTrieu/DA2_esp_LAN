/**
 * @file ble_native_handler.h
 * @brief BLE Native Mesh handler — public interface.
 *
 * Owns the ESP BLE Mesh stack initialization and provisioner lifecycle.
 * Delegates JSON config parsing to ble_native_config, command execution to
 * ble_native_downlink, and uplink reporting to ble_native_uplink.
 *
 * Only DA2_esp_LAN uses this module — DA2_esp (WAN) is untouched.
 */

#ifndef BLE_NATIVE_HANDLER_H
#define BLE_NATIVE_HANDLER_H

#include "esp_err.h"
#include "esp_ble_mesh_defs.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ESP BLE Mesh stack and start uplink/downlink tasks.
 *
 * Must be called once at startup after nvs_flash_init().
 * The mesh stack is initialised with full provisioner support and pre-registered
 * client models (Generic OnOff, Light Lightness, Light CTL).
 *
 * @return ESP_OK on success
 */
esp_err_t ble_native_handler_init(void);

/**
 * @brief Deinitialize the ESP BLE Mesh stack.
 *
 * Stops uplink/downlink tasks, deinitializes the BLE Mesh provisioner,
 * and deinitializes the mesh stack. Safe to call even if not initialized.
 * Call this before switching to a different BLE mode.
 *
 * @return ESP_OK on success
 */
esp_err_t ble_native_handler_deinit(void);

/**
 * @brief Load JSON configuration for one BLE Native stack.
 *
 * Called by config_handler_ble_native_commands.c when CFBN:JSON: is received.
 * Routes to ble_native_config_load() and, on first load, applies mesh network
 * keys to the provisioner via esp_ble_mesh_provisioner_add_local_net_key /
 * esp_ble_mesh_provisioner_add_local_app_key.
 *
 * @param stack_id  Stack index (0 or 1)
 * @param json_str  Null-terminated JSON string
 * @param json_len  Length of json_str in bytes
 * @return ESP_OK on success
 */
esp_err_t ble_native_handler_load_config(uint8_t stack_id,
                                          const char *json_str,
                                          uint16_t json_len);

/**
 * @brief Dispatch a CFBN: command string.
 *
 * Called by config_handler_ble_native_commands.c when CFBN:<stack>:<verb> is
 * received.  Routes to ble_native_downlink_enqueue() which parses the verb and
 * calls the appropriate BLE Mesh API.
 *
 * @param data  Raw command bytes (e.g. "CFBN:0:CONTROL:{...}")
 * @param len   Length in bytes
 * @return ESP_OK on success
 */
esp_err_t ble_native_handler_execute(const uint8_t *data, uint16_t len);

/**
 * @brief Look up a registered ESP BLE Mesh model by model_id.
 *
 * Called from ble_native_downlink.c to obtain the model pointer needed for
 * esp_ble_mesh_*_client_set_state() calls.
 *
 * @param model_id  BT SIG model identifier (e.g. 0x1000 for Generic OnOff)
 * @return Pointer to model, or NULL if not registered
 */
esp_ble_mesh_model_t *ble_native_get_model(uint16_t model_id);

/**
 * @brief Clear the scan accumulation buffer before starting a new scan.
 *        Called from ble_native_downlink.c handle_scan().
 */
void ble_native_scan_reset(uint8_t stack_id);

/**
 * @brief Send all accumulated UNPROV_DEV results as a single batched uplink.
 *        Called from ble_native_downlink.c after the scan timer expires.
 */
void ble_native_scan_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_NATIVE_HANDLER_H */
