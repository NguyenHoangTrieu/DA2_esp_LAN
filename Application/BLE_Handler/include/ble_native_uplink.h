/**
 * @file ble_native_uplink.h
 * @brief BLE Native uplink — send mesh events to WAN MCU.
 *
 * Collects mesh events (provisioning results, status changes, control ACKs)
 * from the BLE app layer and forwards them to WAN MCU via
 * mcu_wan_enqueue_uplink(HANDLER_BLE_NATIVE, ...).
 *
 * Response format: "CFBN:<stack_id>:OK:<payload>" or
 *                  "CFBN:<stack_id>:FAIL:<reason>"
 */

#ifndef BLE_NATIVE_UPLINK_H
#define BLE_NATIVE_UPLINK_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum length of a single uplink message string */
#define BLE_NATIVE_UPLINK_MSG_MAX  512

/* Queue depth for buffering mesh events */
#define BLE_NATIVE_UPLINK_QUEUE_DEPTH  16

/**
 * @brief Start the uplink task.
 *        Must be called once during system init (from ble_native_handler_init).
 */
esp_err_t ble_native_uplink_task_start(void);

/**
 * @brief Stop and delete the uplink task.
 */
void ble_native_uplink_task_stop(void);

/**
 * @brief Enqueue an OK response to be sent to WAN MCU.
 *
 * Formats as: "CFBN:<stack_id>:OK:<payload>"
 *
 * @param stack_id  Stack index (0 or 1)
 * @param payload   Null-terminated payload string
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t ble_native_uplink_send_ok(uint8_t stack_id, const char *payload);

/**
 * @brief Enqueue a FAIL response to be sent to WAN MCU.
 *
 * Formats as: "CFBN:<stack_id>:FAIL:<reason>"
 *
 * @param stack_id  Stack index
 * @param reason    Null-terminated reason string
 */
esp_err_t ble_native_uplink_send_fail(uint8_t stack_id, const char *reason);

/**
 * @brief Enqueue a raw pre-formatted CFBN response string.
 *
 * Use when the caller already assembled the full "CFBN:..." string.
 *
 * @param msg       Null-terminated, fully formatted message
 * @param msg_len   Length of msg (bytes, excluding null terminator)
 */
esp_err_t ble_native_uplink_send_raw(const char *msg, uint16_t msg_len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_NATIVE_UPLINK_H */
