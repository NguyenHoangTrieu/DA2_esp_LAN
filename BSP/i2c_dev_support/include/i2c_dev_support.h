/**
 * @file i2c_dev_support.h
 * @brief Common I2C Device Support Layer for ESP-IDF 6.0
 */

#ifndef I2C_DEV_SUPPORT_H
#define I2C_DEV_SUPPORT_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C Pin Configuration
#define I2C_MASTER_SDA_IO       2
#define I2C_MASTER_SCL_IO       1
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000  // 100kHz default

/**
 * @brief Initialize I2C master bus
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_init(void);

/**
 * @brief Deinitialize I2C master bus
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_deinit(void);

/**
 * @brief Check if I2C bus is initialized
 * @return true if initialized, false otherwise
 */
bool i2c_dev_support_is_initialized(void);

/**
 * @brief Get I2C master bus handle
 * @return I2C bus handle or NULL if not initialized
 */
i2c_master_bus_handle_t i2c_dev_support_get_bus_handle(void);

/**
 * @brief Add I2C device to the bus
 * @param device_addr 7-bit I2C device address
 * @param scl_speed_hz SCL speed in Hz
 * @param dev_handle Pointer to store device handle
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_add_device(uint8_t device_addr, uint32_t scl_speed_hz, 
                                     i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Remove I2C device from the bus
 * @param dev_handle Device handle to remove
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_remove_device(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Write data to I2C device
 * @param dev_handle Device handle
 * @param data Pointer to data buffer
 * @param len Length of data
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_write(i2c_master_dev_handle_t dev_handle, 
                                const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Read data from I2C device
 * @param dev_handle Device handle
 * @param data Pointer to data buffer
 * @param len Length of data to read
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_read(i2c_master_dev_handle_t dev_handle, 
                               uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Write then read data from I2C device
 * @param dev_handle Device handle
 * @param write_data Pointer to write data buffer
 * @param write_len Length of write data
 * @param read_data Pointer to read data buffer
 * @param read_len Length of read data
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_support_write_read(i2c_master_dev_handle_t dev_handle,
                                     const uint8_t *write_data, size_t write_len,
                                     uint8_t *read_data, size_t read_len, 
                                     uint32_t timeout_ms);
                                    
/**
 * @brief Scan I2C bus and print found devices (debug only)
 */
void i2c_dev_support_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_DEV_SUPPORT_H */
