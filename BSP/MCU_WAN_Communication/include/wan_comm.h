/**
 * @file wan_comm.h
 * @brief WAN Communication Library for LAN MCU (SPI Master)
 */

#ifndef WAN_COMM_H
#define WAN_COMM_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Protocol headers
 */
#define WAN_COMM_HEADER_CF 0x4346 // "CF" - Command Frame
#define WAN_COMM_HEADER_DT 0x4454 // "DT" - Data Frame
#define WAN_COMM_HEADER_SIZE 2

/**
 * @brief Default configuration values
 */
#define WAN_COMM_DEFAULT_CLOCK_HZ (10 * 1000 * 1000) // 10 MHz
#define WAN_COMM_DEFAULT_QUEUE_SIZE 7
#define WAN_COMM_MAX_TRANSFER_SIZE 8192 // SPI DMA limitation
#define WAN_COMM_TIMEOUT_MS 1000

/**
 * @brief Status codes
 */
typedef enum {
  WAN_COMM_OK = 0,
  WAN_COMM_ERR_INVALID_ARG,
  WAN_COMM_ERR_TIMEOUT,
  WAN_COMM_ERR_INVALID_STATE,
  WAN_COMM_ERR_NO_MEM,
  WAN_COMM_ERR_INVALID_HEADER,
  WAN_COMM_ERR_BUS_BUSY,
  WAN_COMM_ERR_NOT_INITIALIZED
} wan_comm_status_t;

/**
 * @brief Configuration structure
 */
typedef struct {
  // GPIO pins
  int gpio_sck; // SPI Clock
  int gpio_cs;  // Chip Select
  int gpio_io0; // MOSI / IO0
  int gpio_io1; // MISO / IO1
  int gpio_io2; // WP / IO2 (for Quad mode)
  int gpio_io3; // HD / IO3 (for Quad mode)

  // SPI configuration
  uint32_t clock_speed_hz;   // SPI clock frequency
  uint8_t mode;              // SPI mode (0-3)
  spi_host_device_t host_id; // SPI peripheral (SPI2_HOST or SPI3_HOST)

  // DMA configuration
  int dma_channel;     // DMA channel (SPI_DMA_CH_AUTO recommended)
  uint16_t queue_size; // Transaction queue size

  // Options
  bool enable_quad_mode; // Enable QSPI (4-bit) mode
} wan_comm_config_t;

/**
 * @brief Handle structure (opaque)
 */
typedef struct wan_comm_handle_s *wan_comm_handle_t;

/**
 * @brief Initialize WAN communication library (Master mode)
 *
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_init(const wan_comm_config_t *config,
                                wan_comm_handle_t *handle);

/**
 * @brief Deinitialize WAN communication library
 *
 * @param handle Handle to deinitialize
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_deinit(wan_comm_handle_t handle);

/**
 * @brief Send command packet to slave (WAN MCU)
 * Automatically prepends CF header to payload
 *
 * @param handle Communication handle
 * @param command_payload Command data
 * @param length Payload length (excluding header)
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_send_command(wan_comm_handle_t handle,
                                        const uint8_t *command_payload,
                                        uint16_t length);

/**
 * @brief Send data packet to slave (WAN MCU)
 * Automatically prepends DT header to payload
 *
 * @param handle Communication handle
 * @param data_payload Data to send
 * @param length Payload length (excluding header)
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_send_data(wan_comm_handle_t handle,
                                     const uint8_t *data_payload,
                                     uint16_t length);

/**
 * @brief Request data from slave (WAN MCU)
 * Performs a read transaction (master clocks, slave sends data)
 *
 * @param handle Communication handle
 * @param rx_buffer Buffer to receive data
 * @param length_to_read Number of bytes to read
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_request_data(wan_comm_handle_t handle,
                                        uint8_t *rx_buffer,
                                        uint16_t length_to_read);

/**
 * @brief Get last error status
 *
 * @param handle Communication handle
 * @return wan_comm_status_t Last error code
 */
wan_comm_status_t wan_comm_get_last_error(wan_comm_handle_t handle);

/**
 * @brief Get error count
 *
 * @param handle Communication handle
 * @return uint32_t Number of errors occurred
 */
uint32_t wan_comm_get_error_count(wan_comm_handle_t handle);

/**
 * @brief Clear error count
 *
 * @param handle Communication handle
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_clear_error_count(wan_comm_handle_t handle);

/**
 * @brief Get header size constant
 *
 * @return uint16_t Header size in bytes
 */
static inline uint16_t wan_comm_get_header_size(void) {
  return WAN_COMM_HEADER_SIZE;
}

#ifdef __cplusplus
}
#endif

#endif // WAN_COMM_H
