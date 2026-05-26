/**
 * @file module_config_controller.h
 * @brief Module configuration controller - Helper wrapper layer for BSP access
 */

#ifndef MODULE_CONFIG_CONTROLLER_H
#define MODULE_CONFIG_CONTROLLER_H

#include "json_config_parser.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize module config controller
 *
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_init(void);

/**
 * @brief Initialize UART communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @param params UART parameters (baudrate, parity, etc.)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_init_uart(uint8_t stack_id,
                                              const uart_params_t *params);

/**
 * @brief Initialize SPI communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @param params SPI parameters (clock_speed, mode)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_init_spi(uint8_t stack_id,
                                             const spi_params_t *params);

/**
 * @brief Initialize I2C communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @param params I2C parameters (address, clock_speed)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_init_i2c(uint8_t stack_id,
                                             const i2c_params_t *params);

/**
 * @brief Initialize USB communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @param params USB CDC parameters (bit_rate, stop_bits, parity, data_bits)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_init_usb(uint8_t stack_id,
                                             const usb_params_t *params);

/**
 * @brief Deinitialize UART communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_deinit_uart(uint8_t stack_id);

/**
 * @brief Deinitialize SPI communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_deinit_spi(uint8_t stack_id);

/**
 * @brief Deinitialize I2C communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_deinit_i2c(uint8_t stack_id);

/**
 * @brief Deinitialize USB communication for stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t module_config_controller_deinit_usb(uint8_t stack_id);

/**
 * @brief Write data to module via configured bus
 *
 * @param stack_id Stack ID (0 or 1)
 * @param port_type Communication port type
 * @param data Data to write
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t module_bus_write(uint8_t stack_id, comm_port_type_t port_type,
                           const uint8_t *data, size_t len);

/**
 * @brief Flush the RX buffer of the configured bus for a stack
 *
 * Currently only effective for UART. Discards any buffered incoming bytes
 * so that a subsequent read returns only fresh data.
 *
 * @param stack_id   Stack ID (0 or 1)
 * @param port_type  Communication port type
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if bus not initialized
 */
esp_err_t module_bus_flush(uint8_t stack_id, comm_port_type_t port_type);

/**
 * @brief Drain the RX buffer into buf over a time window
 *
 * Accumulates all data arriving on the bus within `window_ms` milliseconds.
 * Uses repeated small reads (20 ms each) until the window expires or the
 * buffer is full.  Null-terminates the result.
 *
 * @param stack_id   Stack ID (0 or 1)
 * @param port_type  Communication port type
 * @param buf        Destination buffer
 * @param max        Buffer size (result is capped at max-1 bytes + NUL)
 * @param window_ms  Total observation window in milliseconds
 * @return Number of bytes accumulated
 */
size_t module_bus_drain(uint8_t stack_id, comm_port_type_t port_type,
                        uint8_t *buf, size_t max, uint32_t window_ms);

/**
 * @brief Read data from module via configured bus
 *
 * @param stack_id Stack ID (0 or 1)
 * @param port_type Communication port type
 * @param buffer Buffer to store received data
 * @param max_len Maximum buffer length
 * @param timeout_ms Timeout in milliseconds
 * @param received_len Output: actual received length
 * @return ESP_OK on success
 */
esp_err_t module_bus_read(uint8_t stack_id, comm_port_type_t port_type,
                          uint8_t *buffer, size_t max_len, uint32_t timeout_ms,
                          size_t *received_len);

/**
 * @brief Get the underlying UART handle for a stack (used by the lane
 *        ingress bench to drain UART driver overflow events).
 *
 * @param stack_id Stack ID (0 or 1)
 * @return Handle if UART is initialized for that stack, NULL otherwise.
 */
struct module_uart_comm_s;
typedef struct module_uart_comm_s *module_uart_comm_handle_t;
module_uart_comm_handle_t module_config_controller_get_uart_handle(uint8_t stack_id);

/**
 * @brief Write single GPIO pin
 *
 * @param stack_id Stack ID (0 or 1)
 * @param pin Pin ID string ("01", "02", etc.)
 * @param state true=HIGH, false=LOW
 * @return ESP_OK on success
 */
esp_err_t module_gpio_write(uint8_t stack_id, const char *pin, bool state);

/**
 * @brief Write multiple GPIO pins
 *
 * @param stack_id Stack ID (0 or 1)
 * @param gpio_actions Array of GPIO actions
 * @param count Number of actions
 * @return ESP_OK on success
 */
esp_err_t module_gpio_write_multi(uint8_t stack_id,
                                  const gpio_control_t *gpio_actions,
                                  size_t count);

#ifdef __cplusplus
}
#endif

#endif // MODULE_CONFIG_CONTROLLER_H
