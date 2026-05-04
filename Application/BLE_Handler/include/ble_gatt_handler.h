/**
 * @file ble_gatt_handler.h
 * @brief BLE GATT Central handler — public interface.
 *
 * Owns the ESP Bluedroid GAP + GATTC stack initialization.
 * Manages a device table sized by BLE_GATT_MAX_DEVICES for scanned/connected devices.
 * Delegates config parsing to ble_gatt_config, command execution to
 * ble_gatt_downlink, and response reporting to ble_gatt_uplink.
 *
 * Call order: ble_native_handler_init() FIRST (initialises Bluetooth),
 * then ble_gatt_handler_init().
 */

#ifndef BLE_GATT_HANDLER_H
#define BLE_GATT_HANDLER_H

#include "esp_err.h"
#include "ble_gatt_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the GATT Central handler.
 *
 * Registers GAP and GATTC callbacks, registers the GATTC application, and
 * starts the uplink/downlink tasks.  Must be called after nvs_flash_init()
 * and after ble_native_handler_init() (which inits Bluetooth).
 *
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_handler_init(void);

/**
 * @brief Deinitialize the GATT Central handler.
 *
 * Stops uplink/downlink tasks, disconnects all devices, stops advertising,
 * and unregisters callbacks. Safe to call even if not initialized.
 * Call this before switching to a different BLE mode.
 *
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_handler_deinit(void);

/**
 * @brief Load JSON configuration for one GATT Central stack.
 *
 * Called by config_handler_ble_gatt_commands.c when CFBG:JSON: is received.
 *
 * @param stack_id   Stack index (0 or 1)
 * @param json_str   Null-terminated JSON string
 * @param json_len   Length in bytes
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_handler_load_config(uint8_t stack_id,
                                        const char *json_str,
                                        uint16_t json_len);

/**
 * @brief Dispatch a CFBG: command string.
 *
 * Called by config_handler_ble_gatt_commands.c when CFBG:<stack>:<verb> is
 * received.  Routes to ble_gatt_downlink_enqueue().
 *
 * @param data   Raw command bytes (e.g. "CFBG:0:SCAN:5000")
 * @param len    Length in bytes
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_handler_execute(const uint8_t *data, uint16_t len);

/**
 * @brief Get a device slot by MAC address (used by GATTC callbacks).
 *
 * @param addr        BLE MAC address
 * @param[out] idx    Device table index (0-7)
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t ble_gatt_handler_find_by_addr(const uint8_t *addr, uint8_t *idx);

/**
 * @brief Get a device slot by connection ID (used by GATTC callbacks).
 *
 * @param conn_id     GATT connection ID
 * @param[out] idx    Device table index (0-7)
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t ble_gatt_handler_find_by_conn(uint16_t conn_id, uint8_t *idx);

/**
 * @brief Get a pointer to a device table slot by index.
 *
 * @param idx   Device index (0-7)
 * @return Pointer to entry, or NULL if out of range
 */
ble_gatt_device_t *ble_gatt_handler_get_device(uint8_t idx);

/**
 * @brief Get the GATTC interface (filled after esp_ble_gattc_app_register).
 */
esp_gatt_if_t ble_gatt_handler_get_if(void);

/**
 * @brief Count active BLE GATT connections currently tracked in the device table.
 */
uint8_t ble_gatt_handler_count_connected(void);

/**
 * @brief Return the effective concurrent connection limit from firmware config.
 */
uint8_t ble_gatt_handler_max_connections(void);

/**
 * @brief Report scan results to server — called from GAP callback.
 */
void ble_gatt_handler_report_scan_result(uint8_t stack_id,
                                          const ble_gatt_device_t *dev,
                                          uint8_t dev_idx);

/**
 * @brief Clear all device table entries (call before starting a new scan).
 */
void ble_gatt_handler_clear_devices(void);

/**
 * @brief Set the pending stack ID for the next GAP/GATTC operation.
 *
 * Used by downlink handlers to associate incoming GAP events with a specific
 * stack ID (e.g., when initiating a scan or connection).
 *
 * @param stack_id   Stack index (0 or 1)
 */
void ble_gatt_handler_set_pending_stack(uint8_t stack_id);

/**
 * @brief Signal that an explicit STOP command was issued.
 *
 * Must be called before esp_ble_gap_stop_scanning() so that the
 * SCAN_STOP_COMPLETE_EVT handler knows the event is intentional and not a
 * spurious side-effect of set_scan_params() interrupting a background scan.
 */
void ble_gatt_handler_set_scan_stop_requested(void);

/**
 * @brief Returns true if a scan is currently active (start_scanning succeeded
 *        and neither INQ_CMPL_EVT nor an explicit stop has completed yet).
 */
bool ble_gatt_handler_is_scan_active(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_HANDLER_H */
