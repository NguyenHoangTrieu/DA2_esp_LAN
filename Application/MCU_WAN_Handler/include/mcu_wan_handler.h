/**
 * @file mcu_wan_handler.h
 * @brief MCU WAN Communication Handler (SPI Master - LAN MCU side)
 *
 * Runs on LAN MCU, communicates with WAN MCU (slave) via SPI Master.
 * Handles sensor data forwarding with RTC timestamps, periodic RTC/config
 * requests, and SD card buffering when internet is unavailable.
 */

#ifndef MCU_WAN_HANDLER_H
#define MCU_WAN_HANDLER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Internet status from WAN MCU
 */
typedef enum {
  INTERNET_STATUS_OFFLINE = 0,
  INTERNET_STATUS_ONLINE = 1
} internet_status_t;

/**
 * @brief Start MCU WAN handler task
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_start(void);

/**
 * @brief Stop MCU WAN handler task
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_stop(void);

/**
 * @brief Queue sensor data to send to WAN MCU
 *
 * Called by sensor handlers (CAN, LoRa, etc.) to queue data for transmission.
 *
 * @param data Pointer to data buffer
 * @param length Length of data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_queue_data(const uint8_t *data, uint16_t length);

/**
 * @brief Get current internet status
 *
 * @return internet_status_t Current internet status from WAN MCU
 */
internet_status_t mcu_wan_handler_get_internet_status(void);

/**
 * @brief Get cached config data from WAN MCU
 *
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @param actual_length Actual length of config data
 * @return esp_err_t ESP_OK if config exists
 */
esp_err_t mcu_wan_handler_get_config(uint8_t *buffer, uint16_t buffer_size,
                                     uint16_t *actual_length);

#endif // MCU_WAN_HANDLER_H
