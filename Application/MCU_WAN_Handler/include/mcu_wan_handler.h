/**
 * @file mcu_wan_handler.h
 * @brief MCU WAN Communication Handler
 */

#ifndef MCU_WAN_HANDLER_H
#define MCU_WAN_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Start MCU WAN handler
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_start(void);

/**
 * @brief Stop MCU WAN handler
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcu_wan_handler_stop(void);

#endif // MCU_WAN_HANDLER_H
