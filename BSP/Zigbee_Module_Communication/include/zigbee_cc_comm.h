/**
 * @file zigbee_cc_comm.h
 * @brief CC2530 Zigbee UART Communication Driver
 * 
 * Simple BSP driver for CC2530 Zigbee module with UART interface.
 * Provides basic read/write operations for transparent data transmission.
 */

#ifndef ZIGBEE_CC_COMM_H
#define ZIGBEE_CC_COMM_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Hardware Configuration (Hard-coded) ===== */
/* UART port and pins for CC2530 module */
#define ZIGBEE_CC_UART_PORT         (2)
#define ZIGBEE_CC_UART_TX_PIN       (17)
#define ZIGBEE_CC_UART_RX_PIN       (16)

/* Default UART configuration */
#define ZIGBEE_CC_DEFAULT_BAUD_RATE (38400)
#define ZIGBEE_CC_RX_BUFFER_SIZE    (1024)
#define ZIGBEE_CC_TX_BUFFER_SIZE    (512)

/* ===== Data Types ===== */

/**
 * @brief CC2530 communication handle
 */
typedef struct zigbee_cc_comm_handle_s *zigbee_cc_comm_handle_t;

/**
 * @brief CC2530 UART configuration structure
 */
typedef struct {
    int baud_rate;          /**< UART baud rate (9600, 38400, 115200, etc.) */
    int rx_buffer_size;     /**< RX buffer size in bytes */
    int tx_buffer_size;     /**< TX buffer size in bytes */
} zigbee_cc_uart_config_t;

/* ===== API Functions ===== */

/**
 * @brief Initialize CC2530 Zigbee communication driver
 * 
 * @param config Pointer to UART configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t zigbee_cc_comm_init(const zigbee_cc_uart_config_t *config, 
                               zigbee_cc_comm_handle_t *handle);

/**
 * @brief Deinitialize CC2530 communication driver
 * 
 * @param handle Communication handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t zigbee_cc_comm_deinit(zigbee_cc_comm_handle_t handle);

/**
 * @brief Write data to CC2530 module via UART
 * 
 * @param handle Communication handle
 * @param data Pointer to data buffer
 * @param length Number of bytes to write
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t zigbee_cc_comm_write(zigbee_cc_comm_handle_t handle,
                                const uint8_t *data,
                                size_t length,
                                uint32_t timeout_ms);

/**
 * @brief Read data from CC2530 module via UART
 * 
 * @param handle Communication handle
 * @param buffer Pointer to receive buffer
 * @param length Maximum number of bytes to read
 * @param actual_length Pointer to store actual bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t zigbee_cc_comm_read(zigbee_cc_comm_handle_t handle,
                               uint8_t *buffer,
                               size_t length,
                               size_t *actual_length,
                               uint32_t timeout_ms);

/**
 * @brief Check number of bytes available in RX buffer
 * 
 * @param handle Communication handle
 * @return size_t Number of bytes available to read
 */
size_t zigbee_cc_comm_available(zigbee_cc_comm_handle_t handle);

/**
 * @brief Flush UART RX buffer
 * 
 * Clears all data in the receive buffer.
 * 
 * @param handle Communication handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t zigbee_cc_comm_flush(zigbee_cc_comm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // ZIGBEE_CC_COMM_H
