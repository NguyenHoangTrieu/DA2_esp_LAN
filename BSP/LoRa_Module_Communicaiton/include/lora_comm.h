/**
 * @file lora_comm.h
 * @brief E32 LoRa Module Communication Driver (Hardware Abstraction)
 *
 * Supports multiple communication interfaces (UART, SPI, etc.)
 * Currently implements UART only
 * Pure API - no tasks, callbacks only for user application
 */

#ifndef LORA_COMM_H
#define LORA_COMM_H

#include "e32_config.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Status Codes =====
typedef enum {
  LORA_COMM_OK = 0,
  LORA_COMM_ERR_INVALID_ARG,
  LORA_COMM_ERR_NO_MEM,
  LORA_COMM_ERR_TIMEOUT,
  LORA_COMM_ERR_BUSY,
  LORA_COMM_ERR_NOT_INITIALIZED,
  LORA_COMM_ERR_MODE_SWITCH,
  LORA_COMM_ERR_CONFIG_FAILED,
  LORA_COMM_ERR_COMM_FAILED
} lora_comm_status_t;

// ===== Communication Interface Types =====
typedef enum {
  LORA_COMM_TYPE_UART = 0,
  LORA_COMM_TYPE_SPI, // Reserved for future
  LORA_COMM_TYPE_I2C  // Reserved for future
} lora_comm_type_t;

// ===== Forward Declaration =====
typedef struct lora_comm_handle_s *lora_comm_handle_t;

// ===== Communication Interface (HAL) =====
typedef struct {
  void *user_ctx; // User context pointer (e.g., UART port number)

  /**
   * @brief Initialize communication interface
   * @param config_ptr Pointer to interface-specific configuration
   * @param user_ctx Pointer to store user context
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t (*init)(void *config_ptr, void **user_ctx);

  /**
   * @brief Deinitialize communication interface
   * @param user_ctx User context pointer
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t (*deinit)(void *user_ctx);

  /**
   * @brief Write data to module
   * @param user_ctx User context pointer
   * @param data Data buffer
   * @param length Data length
   * @param timeout_ms Timeout in milliseconds
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t (*write)(void *user_ctx, const uint8_t *data, size_t length,
                     uint32_t timeout_ms);

  /**
   * @brief Read data from module
   * @param user_ctx User context pointer
   * @param data Data buffer
   * @param length Maximum length to read
   * @param actual_length Pointer to store actual bytes read
   * @param timeout_ms Timeout in milliseconds
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t (*read)(void *user_ctx, uint8_t *data, size_t length,
                    size_t *actual_length, uint32_t timeout_ms);

  /**
   * @brief Flush RX buffer
   * @param user_ctx User context pointer
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t (*flush)(void *user_ctx);

  /**
   * @brief Get number of bytes available in RX buffer
   * @param user_ctx User context pointer
   * @return size_t Number of bytes available
   */
  size_t (*available)(void *user_ctx);
} lora_comm_interface_t;

// ===== GPIO Configuration =====
typedef struct {
  int m0_pin;  // M0 mode control pin (-1 if not used)
  int m1_pin;  // M1 mode control pin (-1 if not used)
  int aux_pin; // AUX status pin (-1 if not used)
} lora_gpio_config_t;

// ===== UART Configuration =====
typedef struct {
  int uart_port;      // UART port number
  int tx_pin;         // UART TX pin
  int rx_pin;         // UART RX pin
  int baud_rate;      // UART baud rate (9600 for config mode)
  int rx_buffer_size; // RX buffer size
  int tx_buffer_size; // TX buffer size
} lora_uart_config_t;

// ===== Main Configuration =====
typedef struct {
  lora_comm_type_t comm_type;      // Communication type
  lora_gpio_config_t gpio_config;  // GPIO configuration
  lora_comm_interface_t interface; // Communication interface
  void *interface_config;          // Interface-specific config
  e32_params_t module_params;      // E32 module parameters
} lora_comm_config_t;

// ===== API Functions =====

/**
 * @brief Initialize LoRa communication driver
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_init(const lora_comm_config_t *config,
                                  lora_comm_handle_t *handle);

/**
 * @brief Deinitialize LoRa communication driver
 * @param handle Communication handle
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_deinit(lora_comm_handle_t handle);

/**
 * @brief Set operating mode
 * @param handle Communication handle
 * @param mode Operating mode
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_set_mode(lora_comm_handle_t handle,
                                      e32_mode_t mode);

/**
 * @brief Get current operating mode
 * @param handle Communication handle
 * @param mode Output mode pointer
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_get_mode(lora_comm_handle_t handle,
                                      e32_mode_t *mode);

/**
 * @brief Wait for AUX pin to go high (module ready)
 * @param handle Communication handle
 * @param timeout_ms Timeout in milliseconds
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_wait_aux_high(lora_comm_handle_t handle,
                                           uint32_t timeout_ms);

/**
 * @brief Check if AUX pin is high (module ready)
 * @param handle Communication handle
 * @return true if AUX is high, false otherwise
 */
bool lora_comm_is_aux_high(lora_comm_handle_t handle);

/**
 * @brief Send data in transparent mode
 * @param handle Communication handle
 * @param data Data buffer
 * @param length Data length
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_send_transparent(lora_comm_handle_t handle,
                                              const uint8_t *data,
                                              size_t length);

/**
 * @brief Send data in fixed transmission mode
 * @param handle Communication handle
 * @param target_addr Target address (high 8 bits | low 8 bits)
 * @param target_channel Target channel
 * @param data Data buffer
 * @param length Data length
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_send_fixed(lora_comm_handle_t handle,
                                        uint16_t target_addr,
                                        uint8_t target_channel,
                                        const uint8_t *data, size_t length);

/**
 * @brief Receive data (blocking)
 * @param handle Communication handle
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @param actual_length Pointer to store actual bytes received
 * @param timeout_ms Timeout in milliseconds
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_receive(lora_comm_handle_t handle, uint8_t *buffer,
                                     size_t buffer_size, size_t *actual_length,
                                     uint32_t timeout_ms);

/**
 * @brief Check available bytes in RX buffer
 * @param handle Communication handle
 * @return size_t Number of bytes available
 */
size_t lora_comm_available(lora_comm_handle_t handle);

/**
 * @brief Read module parameters
 * @param handle Communication handle
 * @param params Output parameters structure
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_read_params(lora_comm_handle_t handle,
                                         e32_params_t *params);

/**
 * @brief Write module parameters (save to flash)
 * @param handle Communication handle
 * @param params Parameters structure
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_write_params(lora_comm_handle_t handle,
                                          const e32_params_t *params);

/**
 * @brief Write module parameters (temporary, no save)
 * @param handle Communication handle
 * @param params Parameters structure
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_write_params_temp(lora_comm_handle_t handle,
                                               const e32_params_t *params);

/**
 * @brief Read module version information
 * @param handle Communication handle
 * @param version Output version structure
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_read_version(lora_comm_handle_t handle,
                                          e32_version_t *version);

/**
 * @brief Reset module
 * @param handle Communication handle
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_reset(lora_comm_handle_t handle);

/**
 * @brief Flush RX buffer
 * @param handle Communication handle
 * @return lora_comm_status_t Status code
 */
lora_comm_status_t lora_comm_flush(lora_comm_handle_t handle);

// ===== UART Interface (Pre-defined for convenience) =====
extern lora_comm_interface_t lora_uart_interface;

/**
 * @brief Create UART interface configuration
 * @param uart_config UART configuration
 * @return lora_comm_interface_t Configured interface
 */
lora_comm_interface_t lora_create_uart_interface(void);

#ifdef __cplusplus
}
#endif

#endif // LORA_COMM_H
