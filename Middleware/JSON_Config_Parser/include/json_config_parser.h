/**
 * @file json_config_parser.h
 * @brief Common JSON configuration parser for module metadata and communication
 * config
 */

#ifndef JSON_CONFIG_PARSER_H
#define JSON_CONFIG_PARSER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ========================================================================== */

#define MAX_MODULE_ID_LEN 4
#define MAX_MODULE_TYPE_LEN 32
#define MAX_MODULE_NAME_LEN 64
#define MAX_GPIO_ACTIONS 5
#define MAX_PIN_ID_LEN 4 // "01", "11", etc.

/* ============================================================================
 * Enums
 * ========================================================================== */

/**
 * @brief Communication port types
 */
typedef enum {
  COMM_PORT_UART = 0,
  COMM_PORT_SPI,
  COMM_PORT_I2C,
  COMM_PORT_USB,
  COMM_PORT_MAX
} comm_port_type_t;

/**
 * @brief UART parity options (module-specific to avoid ESP-IDF conflict)
 */
typedef enum {
  MODULE_UART_PARITY_NONE = 0,
  MODULE_UART_PARITY_EVEN,
  MODULE_UART_PARITY_ODD
} module_uart_parity_t;

/* ============================================================================
 * Structures
 * ========================================================================== */

/**
 * @brief GPIO control action (reusable across all modules)
 */
typedef struct {
  char pin[MAX_PIN_ID_LEN]; // Pin ID: "01", "02", etc.
  bool state;               // true = HIGH, false = LOW
} gpio_control_t;

/**
 * @brief UART communication parameters
 */
typedef struct {
  uint32_t baudrate;
  module_uart_parity_t parity;
  uint8_t stopbit;
} uart_params_t;

/**
 * @brief SPI communication parameters
 */
typedef struct {
  uint32_t clock_speed;
  uint8_t mode;
  char bit_order[8]; // "msb" or "lsb"
} spi_params_t;

/**
 * @brief I2C communication parameters
 */
typedef struct {
  uint8_t address;
  uint32_t clock_speed;
} i2c_params_t;

/**
 * @brief USB CDC communication parameters
 */
typedef struct {
  uint32_t bit_rate; // Bit rate (bps) - e.g., 115200
  uint8_t stop_bits; // 0=1bit, 1=1.5bits, 2=2bits
  uint8_t parity;    // 0=None, 1=Odd, 2=Even
  uint8_t data_bits; // 5, 6, 7, 8, or 16
} usb_params_t;

/**
 * @brief Communication configuration (union for different port types)
 */
typedef struct {
  comm_port_type_t port_type;
  union {
    uart_params_t uart;
    spi_params_t spi;
    i2c_params_t i2c;
    usb_params_t usb;
  } params;
} comm_config_t;

/**
 * @brief Module metadata (common for all module types)
 */
typedef struct {
  char module_id[MAX_MODULE_ID_LEN];
  char module_type[MAX_MODULE_TYPE_LEN];
  char module_name[MAX_MODULE_NAME_LEN];
  bool crlf_terminated;           ///< true = append \r\n to ASCII commands
  comm_config_t communication;
} module_metadata_t;

/**
 * @brief Parse module metadata from JSON string
 *
 * Parses complete metadata including:
 * - module_id, module_type, module_name
 * - module_communication (port_type + parameters for UART/SPI/I2C)
 *
 * Does NOT parse functions array (delegate to module-specific parsers)
 *
 * @param json_str JSON string to parse
 * @param metadata Output metadata structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid JSON or fields
 */
esp_err_t json_config_parse_metadata(const char *json_str,
                                     module_metadata_t *metadata);

#ifdef __cplusplus
}
#endif

#endif // JSON_CONFIG_PARSER_H
