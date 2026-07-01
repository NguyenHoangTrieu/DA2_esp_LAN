/**
 * @file rs485_handler.h
 * @brief RS485 Handler - Gateway Side
 *
 * - Uplink: Forward received RS485 data to WAN MCU via mcu_wan_enqueue_uplink()
 * - Downlink: Receive from WAN MCU and send to RS485 bus
 */

#ifndef RS485_HANDLER_H
#define RS485_HANDLER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start RS485 handler task
 *
 * Initializes RS485 driver and starts receive/transmit task.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rs485_handler_start(void);

/**
 * @brief Stop RS485 handler task
 *
 * @return ESP_OK on success
 */
esp_err_t rs485_handler_stop(void);

/**
 * @brief Enqueue downlink message to RS485 bus
 *
 * Expected payload format (raw RS485 data):
 * [data(len)]
 *
 * @param data Downlink buffer
 * @param len Total buffer length
 * @return true on success, false on error/queue full
 */
bool rs485_handler_enqueue_downlink(uint8_t *data, uint16_t len);

extern uint32_t g_rs485_baud_rate;

#ifdef __cplusplus
}
#endif

#endif /* RS485_HANDLER_H */
