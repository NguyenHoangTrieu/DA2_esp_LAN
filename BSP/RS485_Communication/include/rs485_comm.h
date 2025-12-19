/**
 * @file rs485_comm.h
 * @brief RS485 Communication Driver with TCA6424A GPIO control
 * 
 * Provides RS485 communication with automatic DE/RE control via I/O expander.
 */

#ifndef RS485_COMM_H
#define RS485_COMM_H

#include "esp_err.h"
#include "stack_handler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Hardware Configuration - Dual Stack Support ===== */

// Stack 1 configuration
#define RS485_UART_PORT_STACK_1     1
#define RS485_UART_TX_PIN_STACK_1   17
#define RS485_UART_RX_PIN_STACK_1   18
#define RS485_DE_GPIO_STACK_1       STACK_GPIO_PIN_4
#define RS485_RE_GPIO_STACK_1       STACK_GPIO_PIN_5

// Stack 2 configuration
#define RS485_UART_PORT_STACK_2     2
#define RS485_UART_TX_PIN_STACK_2   15
#define RS485_UART_RX_PIN_STACK_2   16
#define RS485_DE_GPIO_STACK_2       STACK_GPIO_PIN_4
#define RS485_RE_GPIO_STACK_2       STACK_GPIO_PIN_5

/* ===== Default Configuration ===== */
#define RS485_DEFAULT_BAUD_RATE     9600
#define RS485_DEFAULT_RX_BUF_SIZE   1024
#define RS485_DEFAULT_TX_BUF_SIZE   512

/* ===== RS485 Mode ===== */
typedef enum {
    RS485_MODE_ONLY_RECEIVE = 1,        /**< Only receive mode (DE=LOW, RE=LOW) */
    RS485_MODE_ONLY_SEND = 2,           /**< Only transmit mode (DE=HIGH, RE=HIGH) */
    RS485_MODE_SEND_AND_RECEIVE = 3     /**< Both TX and RX enabled (DE=HIGH, RE=LOW) */
} rs485_mode_t;

/* ===== Data Types ===== */

/**
 * @brief RS485 communication handle
 */
typedef struct rs485_comm_handle_s *rs485_comm_handle_t;

/**
 * @brief RS485 configuration structure
 */
typedef struct {
    int baud_rate;          /**< Baud rate */
    int rx_buffer_size;     /**< RX buffer size */
    int tx_buffer_size;     /**< TX buffer size */
} rs485_comm_config_t;

/* ===== API Functions ===== */

/**
 * @brief Initialize RS485 communication driver
 * 
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_comm_init(const rs485_comm_config_t *config, rs485_comm_handle_t *handle);

/**
 * @brief Write data via RS485
 * 
 * Automatically switches to transmit mode, sends data, then returns to receive mode.
 * 
 * @param handle Communication handle
 * @param data Data buffer to send
 * @param length Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_comm_write(rs485_comm_handle_t handle, const uint8_t *data, 
                           size_t length, uint32_t timeout_ms);

/**
 * @brief Read data from RS485
 * 
 * @param handle Communication handle
 * @param buffer Buffer to store received data
 * @param length Maximum number of bytes to read
 * @param actual_length Pointer to store actual bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t rs485_comm_read(rs485_comm_handle_t handle, uint8_t *buffer, 
                          size_t length, size_t *actual_length, uint32_t timeout_ms);

/**
 * @brief Check number of bytes available in RX buffer
 * 
 * @param handle Communication handle
 * @return size_t Number of bytes available
 */
size_t rs485_comm_available(rs485_comm_handle_t handle);

/**
 * @brief Flush UART RX buffer
 * 
 * @param handle Communication handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_comm_flush(rs485_comm_handle_t handle);

/**
 * @brief Set RS485 mode
 * 
 * @param handle Communication handle
 * @param mode RS485 mode (ONLY_RECEIVE, ONLY_SEND, SEND_AND_RECEIVE)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rs485_comm_set_mode(rs485_comm_handle_t handle, rs485_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // RS485_COMM_H
