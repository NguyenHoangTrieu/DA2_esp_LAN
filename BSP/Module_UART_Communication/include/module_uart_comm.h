/**
 * @file module_uart_comm.h
 * @brief Generic UART Communication Driver for Modules
 */

#ifndef MODULE_UART_COMM_H
#define MODULE_UART_COMM_H

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Lane-ingress benchmark A/B toggle ===== */
/* 0 = Build A (production-faithful): module_uart_comm_receive() is byte-for-byte
 *     identical to what the Zigbee/LoRa handler tasks run. The UART event queue
 *     is left undrained — exactly as production leaves it — so ISR timing is not
 *     perturbed. Use this build to measure the TRUE per-lane ceiling; detect
 *     saturation by cross-checking rig sent-bytes vs LAN consumed-bytes.
 * 1 = Build B (instrumented): receive() also drains the event queue to tally
 *     UART_FIFO_OVF / UART_BUFFER_FULL so `drop`/`drv_buf_full` can register.
 *     This changes ISR behaviour (observer effect) and is NOT production-faithful.
 * Flip this define and reflash to run the A/B comparison. */
#ifndef MODULE_UART_OVF_DETECT
#define MODULE_UART_OVF_DETECT 0
#endif

/* ===== Hardware Pin Definitions (Hardcoded) ===== */
#define STACK0_UART_PORT    UART_NUM_2
#define STACK0_UART_TX_PIN  17
#define STACK0_UART_RX_PIN  18

#define STACK1_UART_PORT    UART_NUM_1
#define STACK1_UART_TX_PIN  8
#define STACK1_UART_RX_PIN  21

/* ===== Type Definitions ===== */

/**
 * @brief UART communication handle structure (opaque)
 */
typedef struct module_uart_comm_s *module_uart_comm_handle_t;

/**
 * @brief UART configuration structure
 */
typedef struct {
  uint8_t stack_id;           ///< Stack ID (0 or 1) - determines pins/port
  uint32_t baudrate;          ///< Baud rate (e.g., 9600, 115200)
  uart_parity_t parity;       ///< Parity mode
  uart_stop_bits_t stop_bits; ///< Stop bits
  size_t rx_buffer_size;      ///< RX buffer size in bytes
  size_t tx_buffer_size;      ///< TX buffer size in bytes
} module_uart_config_t;

/* ===== Public APIs ===== */

/**
 * @brief Initialize UART communication driver
 *
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NO_MEM: Out of memory
 *         - ESP_FAIL: UART driver installation failed
 */
esp_err_t module_uart_comm_init(const module_uart_config_t *config,
                                module_uart_comm_handle_t *handle);

/**
 * @brief Send data via UART
 *
 * @param handle UART handle
 * @param data Data buffer to send
 * @param len Data length in bytes
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_TIMEOUT: Send timeout
 */
esp_err_t module_uart_comm_send(module_uart_comm_handle_t handle,
                                const uint8_t *data, size_t len,
                                uint32_t timeout_ms);

/**
 * @brief Receive data from UART
 *
 * @param handle UART handle
 * @param buffer Buffer to store received data
 * @param max_len Maximum buffer size
 * @param received Output: actual bytes received
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_OK: Success (even if 0 bytes received)
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_TIMEOUT: Receive timeout
 */
esp_err_t module_uart_comm_receive(module_uart_comm_handle_t handle,
                                   uint8_t *buffer, size_t max_len,
                                   size_t *received, uint32_t timeout_ms);

/**
 * @brief Flush UART RX buffer
 *
 * @param handle UART handle
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid handle
 */
esp_err_t module_uart_comm_flush(module_uart_comm_handle_t handle);

/**
 * @brief Get number of bytes available in RX buffer
 *
 * @param handle UART handle
 * @return size_t Number of bytes available, 0 if handle is invalid
 */
size_t module_uart_comm_available(module_uart_comm_handle_t handle);

/**
 * @brief Read and clear the driver overflow counters since the last call.
 *
 * The UART driver is installed with an event queue, but production code never
 * consumes it. module_uart_comm_receive() drains that queue and tallies HW FIFO
 * overflows and SW ring-buffer-full events. This getter returns those tallies
 * (delta since last call) and resets them — used by the lane-ingress benchmark
 * to detect bus saturation. Either output pointer may be NULL.
 *
 * @param handle    UART handle
 * @param fifo_ovf  Output: UART_FIFO_OVF events (HW FIFO overflow), or NULL
 * @param buf_full  Output: UART_BUFFER_FULL events (SW ring full), or NULL
 */
void module_uart_comm_take_overflow(module_uart_comm_handle_t handle,
                                    uint32_t *fifo_ovf, uint32_t *buf_full);

/**
 * @brief Deinitialize UART communication driver
 *
 * @param handle UART handle
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid handle
 */
esp_err_t module_uart_comm_deinit(module_uart_comm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // MODULE_UART_COMM_H
