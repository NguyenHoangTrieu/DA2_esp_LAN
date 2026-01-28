// BSP/QSPI_Driver/include/qspi_hal_config.h

#ifndef QSPI_HAL_CONFIG_H
#define QSPI_HAL_CONFIG_H

#include "qspi_hal.h"

// Default pin mappings for ESP32-S3
#define QSPI_HAL_DEFAULT_GPIO_CLK       12
#define QSPI_HAL_DEFAULT_GPIO_CS        10
#define QSPI_HAL_DEFAULT_GPIO_D0        11
#define QSPI_HAL_DEFAULT_GPIO_D1        13
#define QSPI_HAL_DEFAULT_GPIO_D2        14
#define QSPI_HAL_DEFAULT_GPIO_D3        15
#define QSPI_HAL_DEFAULT_GPIO_DR_MASTER 46
#define QSPI_HAL_DEFAULT_GPIO_DR_SLAVE  46

// Default config for LAN MCU (Master)
#define QSPI_HAL_CONFIG_LAN_DEFAULT() {                     \
    .pins = {                                                \
        .gpio_clk = QSPI_HAL_DEFAULT_GPIO_CLK,              \
        .gpio_cs  = QSPI_HAL_DEFAULT_GPIO_CS,               \
        .gpio_d0  = QSPI_HAL_DEFAULT_GPIO_D0,               \
        .gpio_d1  = QSPI_HAL_DEFAULT_GPIO_D1,               \
        .gpio_d2  = QSPI_HAL_DEFAULT_GPIO_D2,               \
        .gpio_d3  = QSPI_HAL_DEFAULT_GPIO_D3,               \
        .gpio_dr  = QSPI_HAL_DEFAULT_GPIO_DR_MASTER,        \
    },                                                       \
    .host = SPI2_HOST,                                       \
    .freq_mhz = 40,                                          \
    .is_master = true                                        \
}

// Default config for WAN MCU (Slave)
#define QSPI_HAL_CONFIG_WAN_DEFAULT() {                     \
    .pins = {                                                \
        .gpio_clk = QSPI_HAL_DEFAULT_GPIO_CLK,              \
        .gpio_cs  = QSPI_HAL_DEFAULT_GPIO_CS,               \
        .gpio_d0  = QSPI_HAL_DEFAULT_GPIO_D0,               \
        .gpio_d1  = QSPI_HAL_DEFAULT_GPIO_D1,               \
        .gpio_d2  = QSPI_HAL_DEFAULT_GPIO_D2,               \
        .gpio_d3  = QSPI_HAL_DEFAULT_GPIO_D3,               \
        .gpio_dr  = QSPI_HAL_DEFAULT_GPIO_DR_SLAVE,         \
    },                                                       \
    .host = SPI2_HOST,                                       \
    .freq_mhz = 40,                                          \
    .is_master = false                                       \
}

#endif // QSPI_HAL_CONFIG_H
