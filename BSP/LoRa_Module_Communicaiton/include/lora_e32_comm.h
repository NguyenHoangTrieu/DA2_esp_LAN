/**
 * @file lora_e32_comm.h
 * @brief E32 LoRa Module Communication Driver (Broadcast only, HW abstraction)
 *
 * Supports multiple communication interfaces (UART, SPI, etc.)
 * Currently implements UART only.
 * Pure API - no tasks, callbacks only for user application.
 *
 * NOTE:
 *  - All data transmission is done in broadcast mode.
 *  - Configure E32 address to 0xFFFF and the same channel on all modules
 *    to make every node receive the same frame.
 */

#ifndef LORA_E32_COMM_H
#define LORA_E32_COMM_H

#include "e32_config.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Default hardware configuration (override these at compile time if needed) ===== */

/* GPIO pins for E32 mode control and AUX. 
 * Set to -1 to disable a pin if your hardware does not use it. 
 */
#ifndef LORA_E32_M0_GPIO
#define LORA_E32_M0_GPIO   (18)
#endif

#ifndef LORA_E32_M1_GPIO
#define LORA_E32_M1_GPIO   (17)
#endif

#ifndef LORA_E32_AUX_GPIO
#define LORA_E32_AUX_GPIO  (7)
#endif

/* UART hardware mapping used by the E32 module. 
 * These are board-specific and can be overridden from Kconfig or build flags. 
 */
#ifndef LORA_E32_UART_PORT
#define LORA_E32_UART_PORT (1)
#endif

#ifndef LORA_E32_UART_TX_PIN
#define LORA_E32_UART_TX_PIN (16)
#endif

#ifndef LORA_E32_UART_RX_PIN
#define LORA_E32_UART_RX_PIN (15)
#endif

/* Default UART baud rate for the E32 driver (config / normal mode). */
#ifndef LORA_E32_DEFAULT_BAUD_RATE
#define LORA_E32_DEFAULT_BAUD_RATE (9600)
#endif

// ===== Status Codes =====
typedef enum {
  LORA_E32_COMM_OK = 0,
  LORA_E32_COMM_ERR_INVALID_ARG,
  LORA_E32_COMM_ERR_NO_MEM,
  LORA_E32_COMM_ERR_TIMEOUT,
  LORA_E32_COMM_ERR_BUSY,
  LORA_E32_COMM_ERR_NOT_INITIALIZED,
  LORA_E32_COMM_ERR_MODE_SWITCH,
  LORA_E32_COMM_ERR_CONFIG_FAILED,
  LORA_E32_COMM_ERR_COMM_FAILED
} lora_e32_comm_status_t;

// ===== Communication Interface Types =====
typedef enum {
  LORA_E32_COMM_TYPE_UART = 0,
  LORA_E32_COMM_TYPE_SPI, // Reserved for future
  LORA_E32_COMM_TYPE_I2C  // Reserved for future
} lora_e32_comm_type_t;

// ===== Forward Declaration =====
typedef struct lora_e32_comm_handle_s *lora_e32_comm_handle_t;

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
} lora_e32_comm_interface_t;

// ===== UART Configuration =====
typedef struct {
  int baud_rate;      // UART baud rate (9600 for config mode)
  int rx_buffer_size; // RX buffer size
  int tx_buffer_size; // TX buffer size
} lora_e32_comm_uart_config_t;

// ===== Main Configuration =====
typedef struct {
  lora_e32_comm_type_t comm_type;            // Communication type
  lora_e32_comm_interface_t interface;       // Communication interface
  void *interface_config;                    // Interface-specific config
  e32_params_t module_params;                // E32 module parameters
} lora_e32_comm_config_t;

/* ===== Global E32 configuration context ===== */
/* 
 * These globals hold the current E32 module parameters and UART baud rate. 
 * They are initialized with default values in the driver and can be updated 
 * at runtime by the application, and persisted to NVS if desired. 
 */
extern e32_params_t g_lora_e32_params;
extern int g_lora_e32_baud_rate;

// ===== API Functions =====

//First call this function to auto-initialize the default E32 driver
esp_err_t lora_e32_auto_init_default(void);

/**
 * @brief Initialize LoRa E32 broadcast communication driver
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_init(
    const lora_e32_comm_config_t *config, lora_e32_comm_handle_t *handle);

/**
 * @brief Deinitialize LoRa communication driver
 * @param handle Communication handle
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_deinit(lora_e32_comm_handle_t handle);

/**
 * @brief Set operating mode (Normal, Sleep, Wakeup...)
 * @param handle Communication handle
 * @param mode Operating mode
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_set_mode(lora_e32_comm_handle_t handle,
                                              e32_mode_t mode);

/**
 * @brief Get current operating mode
 * @param handle Communication handle
 * @param mode Output mode pointer
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_get_mode(lora_e32_comm_handle_t handle,
                                              e32_mode_t *mode);
                                              
/**
 * @brief Send data in broadcast mode
 *
 * All modules on the same RF channel will receive this frame.
 * To make it work as broadcast physical layer:
 *  - Configure module address to 0xFFFF
 *  - Use same channel for all nodes
 *
 * @param handle Communication handle
 * @param data Data buffer
 * @param length Data length
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_send_broadcast(
    lora_e32_comm_handle_t handle, const uint8_t *data, size_t length);

/**
 * @brief Receive data (blocking)
 * @param handle Communication handle
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @param actual_length Pointer to store actual bytes received
 * @param timeout_ms Timeout in milliseconds
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t
lora_e32_comm_receive(lora_e32_comm_handle_t handle, uint8_t *buffer,
                      size_t buffer_size, size_t *actual_length,
                      uint32_t timeout_ms);

/**
 * @brief Check available bytes in RX buffer
 * @param handle Communication handle
 * @return size_t Number of bytes available
 */
size_t lora_e32_comm_available(lora_e32_comm_handle_t handle);

/**
 * @brief Read module parameters
 * @param handle Communication handle
 * @param params Output parameters structure
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t
lora_e32_comm_read_params(lora_e32_comm_handle_t handle, e32_params_t *params);

/**
 * @brief Write module parameters (save to flash)
 * @param handle Communication handle
 * @param params Parameters structure
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_write_params(
    lora_e32_comm_handle_t handle, const e32_params_t *params);

/**
 * @brief Write module parameters (temporary, no save)
 * @param handle Communication handle
 * @param params Parameters structure
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_write_params_temp(
    lora_e32_comm_handle_t handle, const e32_params_t *params);

/**
 * @brief Read module version information
 * @param handle Communication handle
 * @param version Output version structure
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_read_version(
    lora_e32_comm_handle_t handle, e32_version_t *version);

/**
 * @brief Reset module
 * @param handle Communication handle
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_reset(lora_e32_comm_handle_t handle);

/**
 * @brief Flush RX buffer
 * @param handle Communication handle
 * @return lora_e32_comm_status_t Status code
 */
lora_e32_comm_status_t lora_e32_comm_flush(lora_e32_comm_handle_t handle);

// ===== UART Interface (Pre-defined for convenience) =====
extern lora_e32_comm_interface_t lora_e32_comm_uart_interface;

/**
 * @brief Create UART interface configuration
 * @return lora_e32_comm_interface_t Configured interface
 */
lora_e32_comm_interface_t lora_e32_comm_create_uart_interface(void);

#ifdef __cplusplus
}
#endif

#endif // LORA_E32_COMM_H
