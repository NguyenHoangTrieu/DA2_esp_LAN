/**
 * @file mcu_wan_handler.h
 * @brief MCU WAN Handler - LAN Side (SPI Master)
 * 
 * Implements Diagram 1: System Control & Data Handling Logic
 * Runs on LAN MCU, communicates with WAN MCU via SPI Master
 */
#ifndef MCU_WAN_HANDLER_H
#define MCU_WAN_HANDLER_H

#include "esp_err.h"
#include "frame_types.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    INTERNET_STATUS_OFFLINE = 0,
    INTERNET_STATUS_ONLINE = 1
} internet_status_t;

/**
 * @brief Start MCU WAN handler task
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_start(void);

/**
 * @brief Stop MCU WAN handler task
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_stop(void);

/**
 * @brief Enqueue uplink data from LAN handlers (CAN/LoRa/Zigbee -> WAN)
 * @param source_id Handler ID (HANDLER_CAN, HANDLER_LORA, etc.)
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return true on success, false on failure
 */
bool mcu_wan_enqueue_uplink(handler_id_t source_id, uint8_t *data, uint16_t len);

/**
 * @brief Get current internet status (from WAN MCU)
 * @return internet_status_t Current status
 */
internet_status_t mcu_wan_handler_get_internet_status(void);

/**
 * @brief Get cached RTC time string (from WAN MCU)
 * @param buffer Output buffer (min 20 bytes)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_get_rtc(char *buffer);

/**
 * @brief Register callback for config data reception
 * @param callback Function pointer for config handling
 */
void mcu_wan_handler_register_config_callback(void (*callback)(const uint8_t*, uint16_t, bool));

#endif // MCU_WAN_HANDLER_H
