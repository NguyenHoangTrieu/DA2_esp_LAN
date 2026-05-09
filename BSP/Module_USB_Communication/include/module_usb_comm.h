/**
 * @file module_usb_comm.h
 * @brief Generic USB CDC Communication Driver for Modules
 */

#ifndef MODULE_USB_COMM_H
#define MODULE_USB_COMM_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Hardware Definitions (Hardcoded) ===== */

// Stack 0 USB configuration
#define STACK0_USB_VID        0x303A  // Espressif VID
#define STACK0_USB_PID        0x1001  // Custom PID

// Stack 1 USB configuration  
#define STACK1_USB_VID        0x303A  // Espressif VID
#define STACK1_USB_PID        0x1002  // Custom PID

/* ===== Type Definitions ===== */

/**
 * @brief USB communication handle structure (opaque)
 */
typedef struct module_usb_comm_s *module_usb_comm_handle_t;

/**
 * @brief USB CDC line coding (for compatibility)
 */
typedef struct {
  uint32_t bit_rate;      ///< Bit rate (bps)
  uint8_t stop_bits;      ///< Stop bits (0=1bit, 1=1.5bits, 2=2bits)
  uint8_t parity;         ///< Parity (0=None, 1=Odd, 2=Even)
  uint8_t data_bits;      ///< Data bits (5,6,7,8,16)
} usb_cdc_line_coding_t;

/**
 * @brief USB configuration structure
 */
typedef struct {
  uint8_t stack_id;          ///< Stack ID (0 or 1) - determines USB instance
  usb_cdc_line_coding_t line_coding;  ///< CDC line coding (optional)
  size_t rx_buffer_size;     ///< RX buffer size in bytes
  size_t tx_buffer_size;     ///< TX buffer size in bytes
} module_usb_config_t;

/* ===== Public APIs ===== */

/**
 * @brief Initialize USB CDC communication driver
 *
 * Uses ESP32's built-in USB Serial/JTAG peripheral.
 * No pin configuration needed (USB D+/D- are hardwired).
 *
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NO_MEM: Out of memory
 *         - ESP_FAIL: USB initialization failed
 */
esp_err_t module_usb_comm_init(const module_usb_config_t *config,
                               module_usb_comm_handle_t *handle);

/**
 * @brief Send data via USB
 *
 * @param handle USB handle
 * @param data Data to send
 * @param len Data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_ERR_NOT_SUPPORTED: Not yet implemented
 */
esp_err_t module_usb_comm_send(module_usb_comm_handle_t handle,
                               const uint8_t *data, size_t len,
                               uint32_t timeout_ms);

/**
 * @brief Receive data from USB
 *
 * @param handle USB handle
 * @param buffer Buffer to store data
 * @param max_len Maximum buffer size
 * @param received Output: bytes received
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t
 *         - ESP_ERR_NOT_SUPPORTED: Not yet implemented
 */
esp_err_t module_usb_comm_receive(module_usb_comm_handle_t handle,
                                  uint8_t *buffer, size_t max_len,
                                  size_t *received, uint32_t timeout_ms);

/**
 * @brief Flush USB RX buffer
 *
 * @param handle USB handle
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid handle
 */
esp_err_t module_usb_comm_flush(module_usb_comm_handle_t handle);

/**
 * @brief Deinitialize USB communication driver
 *
 * @param handle USB handle
 * @return esp_err_t
 *         - ESP_ERR_NOT_SUPPORTED: Not yet implemented
 */
esp_err_t module_usb_comm_deinit(module_usb_comm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // MODULE_USB_COMM_H
