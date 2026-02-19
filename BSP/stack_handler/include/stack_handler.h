/**
 * @file stack_handler.h
 * @brief Communication Stack Manager with GPIO Port Management
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
  STACK_PORT_1 = 0,
  STACK_PORT_2 = 1
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

/* Note: stack_comm_type_t removed - Module Base Setting uses JSON config instead */

/* ===== API Functions ===== */

/**
 * @brief Initialize stack handler and GPIO ports
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_init(void);



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



/* ===== New APIs for Module Controller Support ===== */

/**
 * @brief GPIO action structure for batch operations
 */
typedef struct {
  stack_gpio_pin_num_t pin;
  bool level;
} gpio_action_t;

/**
 * @brief Write multiple GPIO pins at once (batched operation)
 *
 * Optimizes I2C transactions by grouping GPIO writes by TCA port.
 *
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @param actions Array of GPIO actions
 * @param count Number of actions
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_gpio_write_multi(uint8_t stack_id,
                                         const gpio_action_t *actions,
                                         size_t count);

/**
 * @brief Get current state of a GPIO pin
 *
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @param pin GPIO pin number
 * @param state Output: current pin state
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_gpio_get_state(uint8_t stack_id,
                                       stack_gpio_pin_num_t pin, bool *state);

/**
 * @brief Lock stack for exclusive access (thread-safe)
 *
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex not acquired
 */
esp_err_t stack_handler_lock(uint8_t stack_id);

/**
 * @brief Unlock stack after exclusive access
 *
 * @param stack_id Stack ID (0 = Stack 1, 1 = Stack 2)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t stack_handler_unlock(uint8_t stack_id);

/**
 * @brief Get stack module ID (for Module Base Setting architecture)
 *
 * Returns module ID for configured stack:
 * - Stack 0: "002" (BLE STM32WB module - trial version)
 * - Stack 1: "000" (no module)
 *
 * @param stack_id Stack ID (0 or 1)
 * @return Pointer to module ID string ("002", "000", etc.)
 */
const char* stack_handler_get_module_id(uint8_t stack_id);

#ifdef __cplusplus
}
#endif

#endif // STACK_HANDLER_H
