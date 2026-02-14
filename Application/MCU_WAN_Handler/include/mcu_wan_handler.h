/**
 * @file mcu_wan_handler.h
 * @brief MCU WAN Handler - LAN Side (SPI Master)
 */

#ifndef MCU_WAN_HANDLER_H
#define MCU_WAN_HANDLER_H

#include "esp_err.h"
#include "frame_types.h"
#include <stdbool.h>
#include <stdint.h>

// ===== Firmware Version =====
#define LAN_FW_VERSION_MAJOR 1
#define LAN_FW_VERSION_MINOR 1
#define LAN_FW_VERSION_PATCH 1
#define LAN_FW_VERSION_BUILD 2 // Increment after FOTA

#define LAN_FW_VERSION                                                         \
  FW_VERSION_MAKE(LAN_FW_VERSION_MAJOR, LAN_FW_VERSION_MINOR,                  \
                  LAN_FW_VERSION_PATCH, LAN_FW_VERSION_BUILD)

// ===== Public API =====

/**
 * @brief Start MCU WAN handler
 */
esp_err_t mcu_wan_handler_start(void);

/**
 * @brief Stop MCU WAN handler
 */
esp_err_t mcu_wan_handler_stop(void);

/**
 * @brief Enqueue uplink data from LAN handlers
 */
bool mcu_wan_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                            uint16_t len);

/**
 * @brief Get current internet status (cached from WAN MCU)
 */
internet_status_t mcu_wan_handler_get_internet_status(void);

/**
 * @brief Get cached RTC time string (from WAN MCU, updated every 1s)
 */
esp_err_t mcu_wan_handler_get_rtc(char *buffer);

/**
 * @brief Register callback for config data reception
 */
void mcu_wan_handler_register_config_callback(void (*callback)(const uint8_t *,
                                                               uint16_t, bool));

/**
 * @brief Get cached WAN MCU firmware version
 */
uint32_t mcu_wan_handler_get_wan_fw_version(void);

#endif // MCU_WAN_HANDLER_H
