/**
 * @file module_i2c_comm.h
 * @brief Generic I2C Communication Driver for Modules
 */

#ifndef MODULE_I2C_COMM_H
#define MODULE_I2C_COMM_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Hardware Pin Definitions (Hardcoded) ===== */

// Stack 0 I2C pins (adjust according to hardware schematic)
#define STACK0_I2C_PORT I2C_NUM_0
#define STACK0_I2C_SDA_PIN 21
#define STACK0_I2C_SCL_PIN 22

// Stack 1 I2C pins (adjust according to hardware schematic)
#define STACK1_I2C_PORT I2C_NUM_1
#define STACK1_I2C_SDA_PIN 26
#define STACK1_I2C_SCL_PIN 27

/* ===== Type Definitions ===== */

/**
 * @brief I2C communication handle structure (opaque)
 */
typedef struct module_i2c_comm_s *module_i2c_comm_handle_t;

/**
 * @brief I2C configuration structure
 */
typedef struct {
  uint8_t stack_id;        ///< Stack ID (0 or 1) - determines pins/port
  uint8_t device_address;  ///< I2C device address (7-bit)
  uint32_t clock_speed_hz; ///< Clock speed in Hz (e.g., 100000 for 100kHz)
  bool pullup_enable;      ///< Enable internal pull-up resistors
} module_i2c_config_t;

/* ===== Public APIs ===== */

/**
 * @brief Initialize I2C communication driver
 *
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NO_MEM: Out of memory
 */
esp_err_t module_i2c_comm_init(const module_i2c_config_t *config,
                               module_i2c_comm_handle_t *handle);

/**
 * @brief Write data to I2C device
 *
 * @param handle I2C handle
 * @param data Data buffer to write
 * @param len Data length in bytes
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_TIMEOUT: Transaction timeout
 */
esp_err_t module_i2c_comm_write(module_i2c_comm_handle_t handle,
                                const uint8_t *data, size_t len,
                                uint32_t timeout_ms);

/**
 * @brief Read data from I2C device
 *
 * @param handle I2C handle
 * @param buffer Buffer to store read data
 * @param len Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_TIMEOUT: Transaction timeout
 */
esp_err_t module_i2c_comm_read(module_i2c_comm_handle_t handle, uint8_t *buffer,
                               size_t len, uint32_t timeout_ms);

/**
 * @brief Write to a register then read from I2C device
 *
 * @param handle I2C handle
 * @param reg_addr Register address
 * @param buffer Buffer to store read data
 * @param len Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t module_i2c_comm_write_read(module_i2c_comm_handle_t handle,
                                     uint8_t reg_addr, uint8_t *buffer,
                                     size_t len, uint32_t timeout_ms);

/**
 * @brief Deinitialize I2C communication driver
 *
 * @param handle I2C handle
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid handle
 */
esp_err_t module_i2c_comm_deinit(module_i2c_comm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // MODULE_I2C_COMM_H
