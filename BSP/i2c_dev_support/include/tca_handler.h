/**
 * @file tca_handler.h
 * @brief TCA6416A 16-bit I2C I/O Expander — multi-instance (LAN MCU)
 *
 * The LAN MCU supports two adapter slots, each with its own TCA6416A.
 * All API calls take a tca6416a_inst_t pointer so both devices can be
 * driven independently.
 *
 * Address assignment (hardware-wired per slot):
 *   Slot 0 (LAN1): ADDR pin = GND → 0x20
 *   Slot 1 (LAN2): ADDR pin = VCC → 0x21
 */

#ifndef TCA_HANDLER_H
#define TCA_HANDLER_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "i2c_dev_support.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C addresses
#define TCA6416A_I2C_ADDR_0    0x20    // ADDR = GND  (LAN1 adapter slot)
#define TCA6416A_I2C_ADDR_1    0x21    // ADDR = VCC  (LAN2 adapter slot)
#define TCA6416A_I2C_FREQ_HZ   400000  // 400kHz

// Per-slot interrupt GPIO pins (LAN MCU new board)
#define TCA6416A_INT_PIN_LAN1  47      // INT from LAN1 adapter
#define TCA6416A_INT_PIN_LAN2  48      // INT from LAN2 adapter

// Port definitions — TCA6416A has 2 ports only
typedef enum {
    TCA_PORT_0 = 0,
    TCA_PORT_1 = 1,
} tca_port_t;

// TCA6416A register map
#define TCA6416A_INPUT_PORT0    0x00
#define TCA6416A_INPUT_PORT1    0x01
#define TCA6416A_OUTPUT_PORT0   0x02  // TCA6424A was 0x04
#define TCA6416A_OUTPUT_PORT1   0x03  // TCA6424A was 0x05
#define TCA6416A_POLARITY_PORT0 0x04  // TCA6424A was 0x08
#define TCA6416A_POLARITY_PORT1 0x05  // TCA6424A was 0x09
#define TCA6416A_CONFIG_PORT0   0x06  // TCA6424A was 0x0C
#define TCA6416A_CONFIG_PORT1   0x07  // TCA6424A was 0x0D

/**
 * @brief Per-instance context for one TCA6416A device.
 *
 * Allocate one of these per adapter slot and pass its pointer to every API call.
 * Zero-initialise before calling tca_init_inst().
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
    int                     int_gpio;
    uint8_t                 i2c_addr;     /**< I2C address (0x20 or 0x21) for logging */
    bool                    initialized;
} tca6416a_inst_t;

typedef void (*tca_interrupt_callback_t)(tca_port_t port, uint8_t pin_state);

/**
 * @brief Initialize both TCA6416A instances (requires i2c_dev_support to be initialized first).
 *
 * This function initializes both adapter slots:
 *   - Slot 0 (LAN1): address 0x20, INT on GPIO47
 *   - Slot 1 (LAN2): address 0x21, INT on GPIO48
 *
 * @return ESP_OK on success; may return warnings if one slot is absent but the other is OK.
 */
esp_err_t tca_init(void);

 /**
 * Configures the INT GPIO, adds the I2C device, probes it, and sets all
 * pins to input. Requires i2c_dev_support to be initialised first.
 *
 * @param inst      Caller-allocated instance (zero-init before call).
 * @param i2c_addr  TCA6416A_I2C_ADDR_0 or TCA6416A_I2C_ADDR_1.
 * @param int_gpio  GPIO wired to this adapter's INT pin.
 * @return ESP_OK on success; ESP_ERR_NOT_FOUND if no device at addr.
 */
esp_err_t tca_init_inst(tca6416a_inst_t *inst, uint8_t i2c_addr, int int_gpio);

/** @brief Deinitialise an instance and release the I2C device handle. */
esp_err_t tca_deinit_inst(tca6416a_inst_t *inst);

/** @brief Probe — returns ESP_OK if the device ACKs. */
esp_err_t tca_test_connection_inst(tca6416a_inst_t *inst);

/**
 * @brief Configure port direction.
 * @param config  Byte: bit=1 → input, bit=0 → output.
 */
esp_err_t tca_configure_port_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t config);

/** @brief Read input register of a port. */
esp_err_t tca_read_port_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t *value);

/** @brief Write output register of a port. */
esp_err_t tca_write_port_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t value);

/** @brief Read-modify-write a single output pin with optional read-back verify. */
esp_err_t tca_set_pin_verified_inst(tca6416a_inst_t *inst, tca_port_t port,
                                    uint8_t pin, bool level, bool verify);

/** @brief Read a single input pin. */
esp_err_t tca_read_pin_inst(tca6416a_inst_t *inst, tca_port_t port,
                            uint8_t pin, bool *level);

/** @brief Read configuration (direction) register. */
esp_err_t tca_read_config_register_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t *value);

/** @brief Read output register (not input). */
esp_err_t tca_read_output_register_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t *value);

/** @brief Write output register. */
esp_err_t tca_write_output_register_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t value);

/**
 * @brief Probe an I2C address and read P17 (IOX_SLOTDET) without full initialisation.
 *
 * Does NOT configure any GPIO or register an ISR.  Used during boot to discover
 * which adapter is plugged into which physical slot before committing to a full
 * tca_init_inst() call with the correct INT GPIO.
 *
 * Internally:
 *   1. Temporarily adds the device to the I2C bus.
 *   2. Reads CONFIG_PORT0 to confirm the device is present (NACK = not found).
 *   3. Reads INPUT_PORT1 to obtain P17 (bit 7 = SLOTDET).
 *   4. Removes the device handle.
 *
 * @param i2c_addr  Address to probe (TCA6416A_I2C_ADDR_0 or TCA6416A_I2C_ADDR_1).
 * @param slotdet   Output: P17 value — 0 = LAN1 (slot 0), 1 = LAN2 (slot 1).
 *                  Only valid when ESP_OK is returned.
 * @return ESP_OK if a TCA6416A is present; ESP_ERR_NOT_FOUND otherwise.
 */
esp_err_t tca_probe_slotdet(uint8_t i2c_addr, bool *slotdet);

#ifdef __cplusplus
}
#endif

#endif /* TCA_HANDLER_H */
