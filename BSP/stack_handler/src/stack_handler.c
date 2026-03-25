/**
 * @file stack_handler.c
 * @brief LAN Communication Stack Manager â€” two TCA6416A instances
 *
 * Each adapter slot has its own TCA6416A (I2C 0x20 or 0x21).
 * Flat pin mapping: enum 0-7 â†’ PORT_0, enum 8-15 â†’ PORT_1.
 * Module ID is read from P00-P03 at boot; slot identity verified via P17.
 */

#include "stack_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tca_handler.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "STACK_HANDLER";

/* ===== Per-slot state ===== */

static tca6416a_inst_t   g_tca[STACK_HANDLER_MAX_STACKS];
static char              g_module_id[STACK_HANDLER_MAX_STACKS][4]; /* "000"-"015" */
static bool              g_slot_present[STACK_HANDLER_MAX_STACKS];
static SemaphoreHandle_t g_stack_mutex[STACK_HANDLER_MAX_STACKS];
static bool              g_initialized = false;

/* Fixed I2C address and INT GPIO per slot */
static const uint8_t k_i2c_addr[STACK_HANDLER_MAX_STACKS] = {
    TCA6416A_I2C_ADDR_0,   /* Slot 0 (LAN1): ADDR=GND â†’ 0x20 */
    TCA6416A_I2C_ADDR_1,   /* Slot 1 (LAN2): ADDR=VCC â†’ 0x21 */
};
static const int k_int_gpio[STACK_HANDLER_MAX_STACKS] = {
    TCA6416A_INT_PIN_LAN1, /* Slot 0 INT: GPIO47 */
    TCA6416A_INT_PIN_LAN2, /* Slot 1 INT: GPIO48 */
};

/* ===== Helpers ===== */

static inline bool is_valid_stack_id(uint8_t id)        { return id < STACK_HANDLER_MAX_STACKS; }
static inline bool is_valid_pin(stack_gpio_pin_num_t p)  { return (uint8_t)p < STACK_GPIO_PIN_COUNT; }

/* Flat mapping: enum 0-7 â†’ PORT_0 bit 0-7, enum 8-15 â†’ PORT_1 bit 0-7 */
static void get_tca_mapping(stack_gpio_pin_num_t pin, tca_port_t *port, uint8_t *pin_num) {
    *port    = ((uint8_t)pin < 8) ? TCA_PORT_0 : TCA_PORT_1;
    *pin_num = (uint8_t)((uint8_t)pin % 8);
}

/* Read 4-bit module ID from P00-P03 into g_module_id[slot] */
static void read_module_id(uint8_t slot) {
    uint8_t port0_val = 0;
    esp_err_t ret = tca_read_port_inst(&g_tca[slot], TCA_PORT_0, &port0_val);
    if (ret != ESP_OK) {
        snprintf(g_module_id[slot], sizeof(g_module_id[slot]), "000");
        return;
    }
    uint8_t id = port0_val & 0x0F; /* bits 0-3 = P00-P03 */
    snprintf(g_module_id[slot], sizeof(g_module_id[slot]), "%03u", id);
}

/* ===== API ===== */

esp_err_t stack_handler_init(void) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    memset(g_tca,          0, sizeof(g_tca));
    memset(g_slot_present, 0, sizeof(g_slot_present));
    for (int i = 0; i < STACK_HANDLER_MAX_STACKS; i++)
        snprintf(g_module_id[i], sizeof(g_module_id[i]), "000");

    for (int slot = 0; slot < STACK_HANDLER_MAX_STACKS; slot++) {
        g_stack_mutex[slot] = xSemaphoreCreateMutex();
        if (!g_stack_mutex[slot]) {
            ESP_LOGE(TAG, "Failed to create mutex for slot %d", slot);
            for (int j = 0; j < slot; j++) vSemaphoreDelete(g_stack_mutex[j]);
            return ESP_ERR_NO_MEM;
        }
    }

    /* Probe each adapter slot */
    for (int slot = 0; slot < STACK_HANDLER_MAX_STACKS; slot++) {
        esp_err_t ret = tca_init_inst(&g_tca[slot], k_i2c_addr[slot], k_int_gpio[slot]);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "Slot %d: no adapter (0x%02X not found)", slot, k_i2c_addr[slot]);
            continue;
        }

        g_slot_present[slot] = true;

        /* Sanity-check P17 (IOX_SLOTDET) */
        bool slotdet = false;
        tca_read_pin_inst(&g_tca[slot], TCA_PORT_1, 7 /* P17 = PORT_1 bit 7 */, &slotdet);
        int expected = (slot == 1) ? 1 : 0;
        if ((int)slotdet != expected)
            ESP_LOGW(TAG, "Slot%d SLOTDET=%d, expected %d (wiring check?)",
                     slot, (int)slotdet, expected);

        /* Read module ID from P00-P03 */
        read_module_id(slot);

        ESP_LOGI(TAG, "Slot %d: TCA6416A@0x%02X present, module_id=%s SLOTDET=%d",
                 slot, k_i2c_addr[slot], g_module_id[slot], (int)slotdet);
    }

    g_initialized = true;
    return ESP_OK;
}

esp_err_t stack_handler_gpio_write(uint8_t stack_id, stack_gpio_pin_num_t pin,
                                   bool level) {
    if (!g_initialized)             return ESP_ERR_INVALID_STATE;
    if (!is_valid_stack_id(stack_id)) return ESP_ERR_INVALID_ARG;
    if (!g_slot_present[stack_id])  return ESP_ERR_NOT_FOUND;
    if (!is_valid_pin(pin))         return ESP_ERR_INVALID_ARG;

    tca_port_t port; uint8_t pin_num;
    get_tca_mapping(pin, &port, &pin_num);
    return tca_set_pin_verified_inst(&g_tca[stack_id], port, pin_num, level, true);
}

esp_err_t stack_handler_gpio_read(uint8_t stack_id, stack_gpio_pin_num_t pin,
                                  bool *level) {
    if (!g_initialized)                       return ESP_ERR_INVALID_STATE;
    if (!is_valid_stack_id(stack_id) || !level) return ESP_ERR_INVALID_ARG;
    if (!g_slot_present[stack_id])            return ESP_ERR_NOT_FOUND;
    if (!is_valid_pin(pin))                   return ESP_ERR_INVALID_ARG;

    tca_port_t port; uint8_t pin_num;
    get_tca_mapping(pin, &port, &pin_num);
    return tca_read_pin_inst(&g_tca[stack_id], port, pin_num, level);
}

esp_err_t stack_handler_gpio_set_direction(uint8_t stack_id,
                                           stack_gpio_pin_num_t pin,
                                           bool is_output) {
    if (!g_initialized)             return ESP_ERR_INVALID_STATE;
    if (!is_valid_stack_id(stack_id)) return ESP_ERR_INVALID_ARG;
    if (!g_slot_present[stack_id])  return ESP_ERR_NOT_FOUND;
    if (!is_valid_pin(pin))         return ESP_ERR_INVALID_ARG;

    tca_port_t port; uint8_t pin_num;
    get_tca_mapping(pin, &port, &pin_num);

    uint8_t cfg;
    esp_err_t ret = tca_read_config_register_inst(&g_tca[stack_id], port, &cfg);
    if (ret != ESP_OK) return ret;

    if (is_output)  cfg &= ~(uint8_t)(1u << pin_num);
    else            cfg |=  (uint8_t)(1u << pin_num);

    return tca_configure_port_inst(&g_tca[stack_id], port, cfg);
}

esp_err_t stack_handler_gpio_write_multi(uint8_t stack_id,
                                         const gpio_action_t *actions,
                                         size_t count) {
    if (!g_initialized)                                    return ESP_ERR_INVALID_STATE;
    if (!is_valid_stack_id(stack_id) || !actions || count == 0) return ESP_ERR_INVALID_ARG;
    if (!g_slot_present[stack_id])                         return ESP_ERR_NOT_FOUND;

    uint8_t port_masks[2]  = {0};
    uint8_t port_states[2] = {0};

    for (size_t i = 0; i < count; i++) {
        if (!is_valid_pin(actions[i].pin)) {
            ESP_LOGW(TAG, "Skipping invalid pin %d", actions[i].pin);
            continue;
        }
        tca_port_t port; uint8_t pin_num;
        get_tca_mapping(actions[i].pin, &port, &pin_num);
        port_masks[port]  |= (uint8_t)(1u << pin_num);
        if (actions[i].level)
            port_states[port] |= (uint8_t)(1u << pin_num);
    }

    esp_err_t ret = ESP_OK;
    for (int p = 0; p < 2 && ret == ESP_OK; p++) {
        if (port_masks[p] == 0) continue;

        /* Set affected pins as output */
        uint8_t cfg;
        ret = tca_read_config_register_inst(&g_tca[stack_id], (tca_port_t)p, &cfg);
        if (ret != ESP_OK) break;
        cfg &= ~port_masks[p];
        ret = tca_configure_port_inst(&g_tca[stack_id], (tca_port_t)p, cfg);
        if (ret != ESP_OK) break;

        /* Read-modify-write output register */
        uint8_t current;
        ret = tca_read_output_register_inst(&g_tca[stack_id], (tca_port_t)p, &current);
        if (ret != ESP_OK) break;
        current = (current & ~port_masks[p]) | (port_states[p] & port_masks[p]);
        ret = tca_write_output_register_inst(&g_tca[stack_id], (tca_port_t)p, current);
    }

    return ret;
}

esp_err_t stack_handler_gpio_get_state(uint8_t stack_id,
                                       stack_gpio_pin_num_t pin, bool *state) {
    return stack_handler_gpio_read(stack_id, pin, state);
}

esp_err_t stack_handler_lock(uint8_t stack_id) {
    if (!g_initialized || !is_valid_stack_id(stack_id)) return ESP_ERR_INVALID_ARG;
    return (xSemaphoreTake(g_stack_mutex[stack_id], pdMS_TO_TICKS(1000)) == pdTRUE)
           ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t stack_handler_unlock(uint8_t stack_id) {
    if (!g_initialized || !is_valid_stack_id(stack_id)) return ESP_ERR_INVALID_ARG;
    xSemaphoreGive(g_stack_mutex[stack_id]);
    return ESP_OK;
}

const char* stack_handler_get_module_id(uint8_t stack_id) {
    if (!is_valid_stack_id(stack_id)) return "000";
    return g_module_id[stack_id];
}
