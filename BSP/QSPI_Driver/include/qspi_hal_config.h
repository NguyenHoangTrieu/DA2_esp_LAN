// BSP/QSPI_Driver/include/qspi_hal_config.h

#ifndef QSPI_HAL_CONFIG_H
#define QSPI_HAL_CONFIG_H

#include "driver/spi_common.h"

// Hardware Configuration
#define QSPI_HOST_ID SPI2_HOST
#define QSPI_CLOCK_SPEED_HZ (40 * 1000 * 1000)
#define QSPI_DMA_CHAN SPI_DMA_CH_AUTO
#define QSPI_QUEUE_SIZE 3

// GPIO Pins (ESP32-S3)
#define QSPI_PIN_CLK 12
#define QSPI_PIN_CS 10
#define QSPI_PIN_D0 11 // MOSI
#define QSPI_PIN_D1 13 // MISO
#define QSPI_PIN_D2 14 // WP
#define QSPI_PIN_D3 15 // HOLD

// Data Ready Handshake
#define QSPI_PIN_DR_MASTER 8 // LAN MCU output
#define QSPI_PIN_DR_SLAVE 46 // WAN MCU input

// Performance tuning
#define QSPI_MASTER_POLL_INTERVAL_MS 5 // Master RX poll rate
#define QSPI_DR_POLL_TIMEOUT_MS 50     // Max wait for DR signal

#endif // QSPI_HAL_CONFIG_H
