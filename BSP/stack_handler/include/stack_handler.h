/**
 * @file stack_handler.h
 * @brief LAN Communication Stack Manager — two adapter slots (LAN1 & LAN2)
 *
 * Each slot has a dedicated TCA6416A. Pin mapping is flat:
 *   P00-P07 → PORT_0 bits 0-7 (enum 0-7)
 *   P10-P17 → PORT_1 bits 0-7 (enum 8-15)
 *
 * Special pins on every adapter board:
 *   P00-P03 : 4-bit adapter module ID (input, factory-programmed)
 *   P17     : IOX_SLOTDET — 0=LAN1 slot, 1=LAN2 slot (input)
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
#define STACK_HANDLER_MAX_STACKS 2     /**< LAN MCU supports 2 adapter slots       */
#define STACK_GPIO_PIN_COUNT     16    /**< Full TCA6416A 16-pin direct mapping     */
#define STACK_GPIO_PIN_NONE      0xFF  /**< Sentinel: no pin assigned               */

/* ===== GPIO Pin Identifiers ===== */
typedef enum {
    STACK_GPIO_PIN_00 = 0,   /* P00 — adapter ID bit 0 (input) */
    STACK_GPIO_PIN_01 = 1,   /* P01 — adapter ID bit 1 (input) */
    STACK_GPIO_PIN_02 = 2,   /* P02 — adapter ID bit 2 (input) */
    STACK_GPIO_PIN_03 = 3,   /* P03 — adapter ID bit 3 (input) */
    STACK_GPIO_PIN_04 = 4,   /* P04                            */
    STACK_GPIO_PIN_05 = 5,   /* P05                            */
    STACK_GPIO_PIN_06 = 6,   /* P06                            */
    STACK_GPIO_PIN_07 = 7,   /* P07                            */
    STACK_GPIO_PIN_10 = 8,   /* P10                            */
    STACK_GPIO_PIN_11 = 9,   /* P11                            */
    STACK_GPIO_PIN_12 = 10,  /* P12                            */
    STACK_GPIO_PIN_13 = 11,  /* P13                            */
    STACK_GPIO_PIN_14 = 12,  /* P14                            */
    STACK_GPIO_PIN_15 = 13,  /* P15                            */
    STACK_GPIO_PIN_16 = 14,  /* P16                            */
    STACK_GPIO_PIN_17 = 15,  /* P17 — IOX_SLOTDET (input)     */
} stack_gpio_pin_num_t;

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
 * @brief Get adapter module ID detected during init.
 *
 * The ID is read from P00-P03 of the adapter's TCA6416A at boot.
 * Returns "000" if the slot is empty (no TCA6416A responded).
 *
 * @param stack_id  0 = LAN1 adapter slot, 1 = LAN2 adapter slot.
 * @return Null-terminated string, e.g. "002", "006", "000".
 */
const char* stack_handler_get_module_id(uint8_t stack_id);

#ifdef __cplusplus
}
#endif

#endif // STACK_HANDLER_H
