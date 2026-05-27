/**
 * @file module_usb_comm.h
 * @brief Generic USB CDC-ACM Communication Driver for Modules — HOST mode.
 *
 * The LAN MCU acts as the USB HOST; the plugged-in module is always the
 * USB DEVICE (CDC-ACM). The host binds to the first CDC-ACM device that
 * enumerates (any VID/PID), so no per-device IDs are configured here.
 *
 * HARDWARE: ESP32-S3 has a single USB-OTG controller -> exactly ONE USB
 * host port (not per-stack). Only one handle may be active at a time.
 */

#ifndef MODULE_USB_COMM_H
#define MODULE_USB_COMM_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief Initialize the USB host (CDC-ACM) for a stack.
 *
 * Installs the USB Host Library + CDC-ACM class driver and starts a
 * background task that opens any CDC-ACM device that enumerates.
 * D+/D- are hardwired (GPIO19/20 on ESP32-S3); no pin config needed.
 * Receive() returns data only after a device is plugged in and opened.
 *
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_INVALID_STATE: A USB host is already active (one OTG)
 *         - ESP_ERR_NO_MEM: Out of memory
 *         - ESP_FAIL: USB host/CDC driver install failed
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
 * @brief Read-and-reset the RX overflow event counter.
 *
 * Each event = one inbound CDC transfer whose surplus was dropped because
 * the RX stream buffer was full (consumer too slow). This is the USB lane
 * saturation signal, mirroring the UART overflow counter.
 *
 * @param handle USB handle
 * @param rx_overflow_evt Output: events since last call (may be NULL)
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid handle
 */
esp_err_t module_usb_comm_take_overflow(module_usb_comm_handle_t handle,
                                        uint32_t *rx_overflow_evt);

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
