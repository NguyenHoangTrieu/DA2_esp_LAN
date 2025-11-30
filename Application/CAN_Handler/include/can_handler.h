/**
 * @file can_handler.h
 * @brief CAN Message Handler for WAN Forwarding
 */

#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "esp_err.h"

/**
 * @brief Initialize and start the CAN handler task
 * 
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_handler_start(void);

/**
 * @brief Stop the CAN handler task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_handler_stop(void);

#endif // CAN_HANDLER_H
