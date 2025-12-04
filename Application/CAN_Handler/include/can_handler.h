/**
 * @file can_handler.h
 * @brief CAN Handler for LAN MCU
 */
#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "esp_err.h"
#include "frame_types.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Start CAN handler task
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_handler_start(void);

/**
 * @brief Stop CAN handler task
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_handler_stop(void);

/**
 * @brief Enqueue downlink data to CAN bus
 * @param data Data buffer (CAN ID + payload)
 * @param len Data length
 * @return true on success
 */
bool can_handler_enqueue_downlink(uint8_t *data, uint16_t len);

/**
 * @brief Get handler statistics
 * @param rx_count Output: received frames
 * @param tx_count Output: transmitted frames
 * @param errors Output: error count
 */
void can_handler_get_stats(uint32_t *rx_count, uint32_t *tx_count, uint32_t *errors);

#endif // CAN_HANDLER_H
