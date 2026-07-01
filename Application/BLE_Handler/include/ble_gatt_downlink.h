/**
 * @file ble_gatt_downlink.h
 * @brief BLE GATT Central downlink — receive CFBG: commands and dispatch.
 *
 * Supported verbs (mirroring STM32WB55 AT command set):
 *   SCAN:<ms>                          — Scan for connectable BLE devices
 *   STOP                               — Stop active scan
 *   LIST                               — List scanned/connected devices
 *   CLEAR                              — Clear device table
 *   CONNECT:<mac>                      — Connect to a device by MAC
 *   DISCONNECT:<idx>                   — Disconnect device at index
 *   INFO:<idx>                         — Get device info (MAC, name, RSSI)
 *   DISC:<idx>                         — Discover services and characteristics
 *   READ:<idx>:<handle>                — Read a characteristic
 *   WRITE:<idx>:<handle>:<hex>         — Write with response
 *   WRITENR:<idx>:<handle>:<hex>       — Write without response
 *   NOTIFY:<idx>:<cccd_handle>:<1|0>   — Enable/disable notifications
 *   INDICATE:<idx>:<cccd_handle>:<1|0> — Enable/disable indications
 */

#ifndef BLE_GATT_DOWNLINK_H
#define BLE_GATT_DOWNLINK_H

#include "esp_err.h"
#include "frame_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_GATT_DOWNLINK_QUEUE_DEPTH  8
#define BLE_GATT_DOWNLINK_ITEM_MAX     INTER_MCU_PAYLOAD_MAX_LEN

/**
 * @brief Start the downlink task.  Called from ble_gatt_handler_init().
 */
esp_err_t ble_gatt_downlink_task_start(void);

/**
 * @brief Stop the downlink task.
 */
void ble_gatt_downlink_task_stop(void);

/**
 * @brief Enqueue a raw CFBG: command string for execution.
 *
 * @param data  Command bytes (e.g. "CFBG:0:SCAN:5000")
 * @param len   Length in bytes
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t ble_gatt_downlink_enqueue(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_DOWNLINK_H */
