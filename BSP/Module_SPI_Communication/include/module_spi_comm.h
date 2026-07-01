/**
 * @file module_spi_comm.h
 * @brief Generic SPI Communication Driver for Modules
 */

#ifndef MODULE_SPI_COMM_H
#define MODULE_SPI_COMM_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Hardware Pin Definitions (Hardcoded) ===== */

// Stack 0 SPI pins (adjust according to hardware schematic)
#define STACK0_SPI_HOST       SPI3_HOST
#define STACK0_SPI_MOSI_PIN   40
#define STACK0_SPI_MISO_PIN   42
#define STACK0_SPI_SCLK_PIN   41
#define STACK0_SPI_CS_PIN     38

// Stack 1 SPI pins (adjust according to hardware schematic)
#define STACK1_SPI_HOST       SPI3_HOST
#define STACK1_SPI_MOSI_PIN   40
#define STACK1_SPI_MISO_PIN   42
#define STACK1_SPI_SCLK_PIN   41
#define STACK1_SPI_CS_PIN     39

/* ===== Type Definitions ===== */

/**
 * @brief SPI communication handle structure (opaque)
 */
typedef struct module_spi_comm_s *module_spi_comm_handle_t;

/**
 * @brief SPI configuration structure
 */
typedef struct {
  uint8_t stack_id;            ///< Stack ID (0 or 1) - determines pins/host
  uint32_t clock_speed_hz;     ///< Clock speed in Hz
  uint8_t mode;                ///< SPI mode (0-3)
  uint8_t queue_size;          ///< Transaction queue size (default: 1)
} module_spi_config_t;

/* ===== Public APIs ===== */

/**
 * @brief Initialize SPI communication driver
 *
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NO_MEM: Out of memory
 */
esp_err_t module_spi_comm_init(const module_spi_config_t *config,
                               module_spi_comm_handle_t *handle);

/**
 * @brief SPI transfer (full-duplex)
 *
 * @param handle SPI handle
 * @param tx_data Data to transmit (can be NULL for read-only)
 * @param rx_data Buffer for received data (can be NULL for write-only)
 * @param len Transaction length in bytes
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t module_spi_comm_transfer(module_spi_comm_handle_t handle,
                                   const uint8_t *tx_data, uint8_t *rx_data,
                                   size_t len);

/**
 * @brief Deinitialize SPI communication driver
 *
 * @param handle SPI handle
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid handle
 */
esp_err_t module_spi_comm_deinit(module_spi_comm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // MODULE_SPI_COMM_H
