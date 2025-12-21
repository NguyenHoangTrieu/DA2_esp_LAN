/**
 * @file stack_handler.h
 * @brief Communication Stack Manager with GPIO Port Management
 *
 * Manages 2 communication stacks with dedicated GPIO ports from TCA6424A.
 * Each stack has access to 9 GPIO pins mapped to specific TCA ports.
 */

#ifndef STACK_HANDLER_H
#define STACK_HANDLER_H

#include "esp_err.h"
#include "tca_handler.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Constants ===== */
#define STACK_HANDLER_MAX_STACKS 2
#define STACK_GPIO_PIN_COUNT 9

/* ===== Stack Port Definitions ===== */
typedef enum {
  STACK_PORT_1 = TCA_PORT_0,
  STACK_PORT_2 = TCA_PORT_1
} stack_port_t;

/* ===== GPIO Pin Numbers ===== */
typedef enum {
  STACK_GPIO_PIN_1 = 0,
  STACK_GPIO_PIN_2 = 1,
  STACK_GPIO_PIN_3 = 2,
  STACK_GPIO_PIN_4 = 3,
  STACK_GPIO_PIN_5 = 4,
  STACK_GPIO_PIN_6 = 5,
  STACK_GPIO_PIN_7 = 6,
  STACK_GPIO_PIN_8 = 7,
  STACK_GPIO_PIN_9 = 8
} stack_gpio_pin_num_t;

/* ===== Communication Types ===== */
typedef enum {
  STACK_COMM_TYPE_NONE = 0,
  STACK_COMM_TYPE_LORA,
  STACK_COMM_TYPE_RS485,
  STACK_COMM_TYPE_ZIGBEE,
  STACK_COMM_TYPE_CAN
} stack_comm_type_t;

/* ===== Stack Configuration ===== */
typedef struct {
  stack_comm_type_t comm_type;
  stack_port_t gpio_port;
  uint8_t uart_port;
  int tx_pin;
  int rx_pin;
  bool enabled;
} stack_config_t;

/* ===== Global Variables ===== */
extern stack_comm_type_t g_stack_1_type;
extern stack_comm_type_t g_stack_2_type;

/* ===== API Functions ===== */

/**
 * @brief Initialize stack handler and GPIO ports
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_init(void);

/**
 * @brief Configure a communication stack
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @param config Stack configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_set_config(uint8_t stack_id,
                                   const stack_config_t *config);

/**
 * @brief Write a value to a GPIO pin on a stack port
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @param pin GPIO pin number (STACK_GPIO_PIN_1 to STACK_GPIO_PIN_9)
 * @param level Pin level (true=HIGH, false=LOW)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_gpio_write(uint8_t stack_id, stack_gpio_pin_num_t pin,
                                   bool level);

/**
 * @brief Read a value from a GPIO pin on a stack port
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @param pin GPIO pin number (STACK_GPIO_PIN_1 to STACK_GPIO_PIN_9)
 * @param level Output pin level pointer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_gpio_read(uint8_t stack_id, stack_gpio_pin_num_t pin,
                                  bool *level);

/**
 * @brief Configure GPIO pin direction (input/output)
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @param pin GPIO pin number (STACK_GPIO_PIN_1 to STACK_GPIO_PIN_9)
 * @param is_output true for output, false for input
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_gpio_set_direction(uint8_t stack_id,
                                           stack_gpio_pin_num_t pin,
                                           bool is_output);

/**
 * @brief Convert stack communication type to string
 * @param type Communication type
 * @return const char* String representation
 */
const char *stack_handler_type_to_string(stack_comm_type_t type);

#ifdef __cplusplus
}
#endif

#endif // STACK_HANDLER_H
