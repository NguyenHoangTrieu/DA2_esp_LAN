/**
 * @file tca_handler.h
 * @brief TCA6424A 24-bit I2C I/O Expander Handler
 */

#ifndef TCA_HANDLER_H
#define TCA_HANDLER_H

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// TCA6424A Configuration
#define TCA6424A_I2C_ADDR 0x22      // ADDR pin = GND
#define TCA6424A_I2C_FREQ_HZ 100000 // 100kHz

// GPIO Pins
#define TCA6424A_INT_PIN 21   // Interrupt pin
#define TCA6424A_RESET_PIN 47 // Reset pin

// Port definitions
typedef enum { TCA_PORT_0 = 0, TCA_PORT_1 = 1, TCA_PORT_2 = 2 } tca_port_t;

// Register addresses
#define TCA6424A_INPUT_PORT0 0x00
#define TCA6424A_INPUT_PORT1 0x01
#define TCA6424A_INPUT_PORT2 0x02
#define TCA6424A_OUTPUT_PORT0 0x04
#define TCA6424A_OUTPUT_PORT1 0x05
#define TCA6424A_OUTPUT_PORT2 0x06
#define TCA6424A_POLARITY_PORT0 0x08
#define TCA6424A_POLARITY_PORT1 0x09
#define TCA6424A_POLARITY_PORT2 0x0A
#define TCA6424A_CONFIG_PORT0 0x0C
#define TCA6424A_CONFIG_PORT1 0x0D
#define TCA6424A_CONFIG_PORT2 0x0E

/**
 * @brief TCA6424A interrupt callback function type
 * @param port Port number that triggered interrupt
 * @param pin_state Current state of port pins
 */
typedef void (*tca_interrupt_callback_t)(tca_port_t port, uint8_t pin_state);

/**
 * @brief Initialize TCA6424A (requires i2c_dev_support to be initialized first)
 * @return ESP_OK on success
 */
esp_err_t tca_init(void);

/**
 * @brief Test if TCA6424A is responding
 * @return ESP_OK if device responds
 */
esp_err_t tca_test_connection(void);

/**
 * @brief Deinitialize TCA6424A
 * @return ESP_OK on success
 */
esp_err_t tca_deinit(void);

/**
 * @brief Hardware reset TCA6424A
 * @return ESP_OK on success
 */
esp_err_t tca_reset(void);

/**
 * @brief Configure port pins as input or output
 * @param port Port number (0-2)
 * @param config Configuration byte (1=input, 0=output)
 * @return ESP_OK on success
 */
esp_err_t tca_configure_port(tca_port_t port, uint8_t config);

/**
 * @brief Read input port
 * @param port Port number (0-2)
 * @param value Pointer to store port value
 * @return ESP_OK on success
 */
esp_err_t tca_read_port(tca_port_t port, uint8_t *value);

/**
 * @brief Write output port
 * @param port Port number (0-2)
 * @param value Value to write
 * @return ESP_OK on success
 */
esp_err_t tca_write_port(tca_port_t port, uint8_t value);

/**
 * @brief Read output port current state
 * @param port Port number (0-2)
 * @param value Pointer to store value
 * @return ESP_OK on success
 */
esp_err_t tca_read_output_port(tca_port_t port, uint8_t *value);

/**
 * @brief Set polarity inversion
 * @param port Port number (0-2)
 * @param polarity Polarity byte (1=inverted, 0=normal)
 * @return ESP_OK on success
 */
esp_err_t tca_set_polarity(tca_port_t port, uint8_t polarity);

/**
 * @brief Set pin with verification
 * @param port Port number (0-2)
 * @param pin Pin number (0-7)
 * @param level Pin level (true=high, false=low)
 * @param verify Enable verification (true=verify, false=no verify)
 * @return ESP_OK on success
 */
esp_err_t tca_set_pin_verified(tca_port_t port, uint8_t pin, bool level,
                               bool verify);

/**
 * @brief Read single pin
 * @param port Port number (0-2)
 * @param pin Pin number (0-7)
 * @param level Pointer to store pin level
 * @return ESP_OK on success
 */
esp_err_t tca_read_pin(tca_port_t port, uint8_t pin, bool *level);

/**
 * @brief Register interrupt callback
 * @param callback Callback function
 * @return ESP_OK on success
 */
esp_err_t tca_register_interrupt_callback(tca_interrupt_callback_t callback);

/**
 * @brief Read configuration register of a port
 * @param port Port number (0-2)
 * @param value Output value pointer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tca_read_config_register(tca_port_t port, uint8_t *value);

/**
 * @brief Read output register of a port
 * @param port Port number (0-2)
 * @param value Output value pointer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tca_read_output_register(tca_port_t port, uint8_t *value);

/**
 * @brief Write output register of a port
 * @param port Port number (0-2)
 * @param value Value to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tca_write_output_register(tca_port_t port, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* TCA_HANDLER_H */
