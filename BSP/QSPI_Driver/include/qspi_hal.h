// BSP/QSPI_Driver/include/qspi_hal.h

#ifndef QSPI_HAL_H
#define QSPI_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"

#define QSPI_HAL_BUFFER_SIZE        (4 * 1024)     // 4KB per buffer
#define QSPI_HAL_NUM_BUFFERS        2              // Ping-pong
#define QSPI_HAL_MAX_FREQ_MHZ       40             // 40MHz max
#define QSPI_HAL_CMD_WRITE          0xAA           // Master write command
#define QSPI_HAL_CMD_READ           0x55           // Master read command
#define QSPI_HAL_DUMMY_BITS         8              // ESP32-S3 requirement

typedef struct {
    int gpio_clk;
    int gpio_cs;
    int gpio_d0;
    int gpio_d1;
    int gpio_d2;
    int gpio_d3;
    int gpio_dr;                // Data-ready GPIO
} qspi_hal_pins_t;

typedef struct {
    qspi_hal_pins_t pins;
    spi_host_device_t host;     // SPI2_HOST or SPI3_HOST
    uint32_t freq_mhz;          // Clock frequency
    bool is_master;             // true=master, false=slave
} qspi_hal_config_t;

typedef void (*qspi_hal_rx_callback_t)(uint8_t *data, size_t len, void *arg);

typedef struct {
    uint64_t bytes_tx;
    uint64_t bytes_rx;
    uint32_t dma_errors;
} qspi_hal_stats_t;

// Init/deinit
esp_err_t qspi_hal_init(const qspi_hal_config_t *cfg);
esp_err_t qspi_hal_deinit(void);

// Streaming API
esp_err_t qspi_hal_stream_write(const uint8_t *data, size_t len, uint32_t timeout_ms);
esp_err_t qspi_hal_stream_read(uint8_t *buffer, size_t *len, uint32_t timeout_ms);

// Callback-based RX
esp_err_t qspi_hal_register_rx_callback(qspi_hal_rx_callback_t callback, void *arg);

// GPIO handshake
esp_err_t qspi_hal_assert_data_ready(void);
esp_err_t qspi_hal_deassert_data_ready(void);
bool qspi_hal_is_peer_ready(void);

// Statistics
esp_err_t qspi_hal_get_stats(qspi_hal_stats_t *stats);

#endif // QSPI_HAL_H
