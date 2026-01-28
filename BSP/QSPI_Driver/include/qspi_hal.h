// BSP/QSPI_Driver/include/qspi_hal.h

#ifndef QSPI_HAL_H
#define QSPI_HAL_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct qspi_hal_s *qspi_hal_handle_t;

typedef enum { QSPI_MODE_MASTER = 0, QSPI_MODE_SLAVE } qspi_mode_t;

// Protocol constants
#define QSPI_HEADER_SIZE 6
#define QSPI_FRAME_SYNC 0xA5C3
#define QSPI_CMD_WRITE 0xAA
#define QSPI_CMD_READ 0x55

// Frame types
#define QSPI_FRAME_DATA 0x01
#define QSPI_FRAME_CMD 0x02
#define QSPI_FRAME_ACK 0x03

// Buffer config
#define QSPI_DMA_BUFFER_SIZE 4096
#define QSPI_TIMEOUT_MS 100
#define QSPI_SLAVE_TX_TIMEOUT_MS 1000

typedef struct {
  uint16_t sync;
  uint8_t type;
  uint16_t length;
  uint8_t *payload;
  uint8_t header_crc;
  uint16_t payload_crc;
} qspi_frame_t;

typedef void (*qspi_rx_callback_t)(qspi_frame_t *frame, void *user_data);

typedef struct {
  qspi_mode_t mode;
  int clock_speed_hz;

  // GPIO override (0 = use defaults)
  int gpio_clk;
  int gpio_cs;
  int gpio_d0;
  int gpio_d1;
  int gpio_d2;
  int gpio_d3;
  int gpio_dr;

  // Callbacks
  qspi_rx_callback_t rx_callback;
  void *user_data;

  // Options
  bool enable_crc;
  bool enable_stats;
  bool enable_dr_handshake; // Use GPIO DR for flow control
} qspi_hal_config_t;

typedef struct {
  uint32_t tx_bytes;
  uint32_t rx_bytes;
  uint32_t tx_frames;
  uint32_t rx_frames;
  uint32_t crc_errors;
  uint32_t timeout_errors;
  uint32_t dma_errors;
  uint32_t invalid_length_errors;
} qspi_hal_stats_t;

// Core API
esp_err_t qspi_hal_init(const qspi_hal_config_t *config,
                        qspi_hal_handle_t *handle);
esp_err_t qspi_hal_deinit(qspi_hal_handle_t handle);
esp_err_t qspi_hal_start(qspi_hal_handle_t handle);
esp_err_t qspi_hal_stop(qspi_hal_handle_t handle);

// Streaming API
esp_err_t qspi_hal_stream_write(qspi_hal_handle_t handle, const uint8_t *data,
                                size_t len, uint8_t frame_type);
esp_err_t qspi_hal_stream_read(qspi_hal_handle_t handle, uint8_t *buffer,
                               size_t *len, uint32_t timeout_ms);

// Status & Control
esp_err_t qspi_hal_get_stats(qspi_hal_handle_t handle, qspi_hal_stats_t *stats);
esp_err_t qspi_hal_clear_stats(qspi_hal_handle_t handle);
bool qspi_hal_is_ready(qspi_hal_handle_t handle);

// Utilities (in qspi_hal_common.c)
uint8_t qspi_crc8(const uint8_t *data, size_t len);
uint16_t qspi_crc16_ccitt(const uint8_t *data, size_t len);
bool qspi_validate_frame(const qspi_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif // QSPI_HAL_H
