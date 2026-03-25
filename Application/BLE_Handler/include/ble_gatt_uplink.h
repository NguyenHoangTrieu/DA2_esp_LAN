/**
 * @file ble_gatt_uplink.h
 * @brief BLE GATT Central uplink — sends CFBG: responses to the WAN MCU.
 *
 * Response format mirrors CFBN:/CFBL: uplinks:
 *   "CFBG:<stack_id>:OK:<payload>"
 *   "CFBG:<stack_id>:FAIL:<reason>"
 */

#ifndef BLE_GATT_UPLINK_H
#define BLE_GATT_UPLINK_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_GATT_UPLINK_QUEUE_DEPTH  16
#define BLE_GATT_UPLINK_MSG_MAX      512

/**
 * @brief Start the uplink task.  Called from ble_gatt_handler_init().
 */
esp_err_t ble_gatt_uplink_task_start(void);

/**
 * @brief Stop the uplink task.
 */
void ble_gatt_uplink_task_stop(void);

/**
 * @brief Send "CFBG:<stack_id>:OK:<payload>" to WAN MCU.
 */
esp_err_t ble_gatt_uplink_send_ok(uint8_t stack_id, const char *payload);

/**
 * @brief Send "CFBG:<stack_id>:FAIL:<reason>" to WAN MCU.
 */
esp_err_t ble_gatt_uplink_send_fail(uint8_t stack_id, const char *reason);

/**
 * @brief Send a pre-formatted raw CFBG: message to WAN MCU.
 */
esp_err_t ble_gatt_uplink_send_raw(const char *msg, uint16_t msg_len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_UPLINK_H */
