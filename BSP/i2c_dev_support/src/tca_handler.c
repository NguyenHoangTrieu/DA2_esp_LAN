/**
 * @file tca_handler.c
 * @brief TCA6416A I/O Expander — multi-instance implementation (LAN MCU)
 *
 * Each function takes a tca6416a_inst_t* so two independent TCA6416A devices
 * (one per adapter slot) can be driven from the same driver.
 */

#include "tca_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_dev_support.h"

static const char *TAG = "TCA6416A";

/* ===== Internal per-instance register helpers ===== */

static esp_err_t write_reg(tca6416a_inst_t *inst, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_dev_support_write(inst->dev_handle, data, 2, 1000);
}

static esp_err_t read_reg(tca6416a_inst_t *inst, uint8_t reg, uint8_t *value) {
    return i2c_dev_support_write_read(inst->dev_handle, &reg, 1, value, 1, 1000);
}

static void IRAM_ATTR tca_isr_handler(void *arg) {
    (void)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ===== Public API ===== */

esp_err_t tca_init_inst(tca6416a_inst_t *inst, uint8_t i2c_addr, int int_gpio) {
    if (!inst) return ESP_ERR_INVALID_ARG;
    if (!i2c_dev_support_is_initialized()) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing TCA6416A at 0x%02X (INT=GPIO%d)", i2c_addr, int_gpio);

    inst->int_gpio    = int_gpio;
    inst->initialized = false;
    inst->dev_handle  = NULL;

    /* Configure INT GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << int_gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "gpio_install_isr_service: %s (non-fatal)", esp_err_to_name(isr_ret));
    gpio_isr_handler_add((gpio_num_t)int_gpio, tca_isr_handler, inst);

    /* Add device to I2C bus */
    esp_err_t ret = i2c_dev_support_add_device(i2c_addr, TCA6416A_I2C_FREQ_HZ,
                                               &inst->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", i2c_addr, esp_err_to_name(ret));
        return ret;
    }

    /* Probe */
    ret = tca_test_connection_inst(inst);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No TCA6416A at 0x%02X", i2c_addr);
        i2c_dev_support_remove_device(inst->dev_handle);
        inst->dev_handle = NULL;
        return ret;
    }

    /* Default: all pins input */
    tca_configure_port_inst(inst, TCA_PORT_0, 0xFF);
    tca_configure_port_inst(inst, TCA_PORT_1, 0xFF);

    inst->initialized = true;
    ESP_LOGI(TAG, "TCA6416A 0x%02X ready", i2c_addr);
    return ESP_OK;
}

esp_err_t tca_deinit_inst(tca6416a_inst_t *inst) {
    if (!inst || !inst->initialized) return ESP_OK;
    gpio_isr_handler_remove((gpio_num_t)inst->int_gpio);
    esp_err_t ret = i2c_dev_support_remove_device(inst->dev_handle);
    inst->dev_handle  = NULL;
    inst->initialized = false;
    ESP_LOGI(TAG, "TCA6416A deinitialized");
    return ret;
}

esp_err_t tca_test_connection_inst(tca6416a_inst_t *inst) {
    if (!inst || !inst->dev_handle) return ESP_ERR_INVALID_STATE;
    uint8_t val;
    esp_err_t ret = read_reg(inst, TCA6416A_CONFIG_PORT0, &val);
    if (ret == ESP_OK)
        ESP_LOGI(TAG, "TCA6416A OK - Config0: 0x%02X", val);
    else
        ESP_LOGE(TAG, "TCA6416A not responding: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t tca_configure_port_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t config) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1)
        return ESP_ERR_INVALID_ARG;
    uint8_t reg = TCA6416A_CONFIG_PORT0 + (uint8_t)port;
    esp_err_t ret = write_reg(inst, reg, config);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Failed to configure port %d: %s", port, esp_err_to_name(ret));
    return ret;
}

esp_err_t tca_read_port_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t *value) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1 || !value)
        return ESP_ERR_INVALID_ARG;
    uint8_t reg = TCA6416A_INPUT_PORT0 + (uint8_t)port;
    return read_reg(inst, reg, value);
}

esp_err_t tca_write_port_inst(tca6416a_inst_t *inst, tca_port_t port, uint8_t value) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1)
        return ESP_ERR_INVALID_ARG;
    uint8_t reg = TCA6416A_OUTPUT_PORT0 + (uint8_t)port;
    esp_err_t ret = write_reg(inst, reg, value);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Failed to write port %d: %s", port, esp_err_to_name(ret));
    return ret;
}

esp_err_t tca_set_pin_verified_inst(tca6416a_inst_t *inst, tca_port_t port,
                                     uint8_t pin, bool level, bool verify) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1 || pin > 7)
        return ESP_ERR_INVALID_ARG;

    uint8_t reg = TCA6416A_OUTPUT_PORT0 + (uint8_t)port;
    uint8_t current;
    esp_err_t ret = read_reg(inst, reg, &current);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read output port %d failed: %s", port, esp_err_to_name(ret));
        return ret;
    }

    if (level)  current |=  (uint8_t)(1u << pin);
    else        current &= ~(uint8_t)(1u << pin);

    ret = write_reg(inst, reg, current);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write output port %d failed: %s", port, esp_err_to_name(ret));
        return ret;
    }

    if (verify) {
        vTaskDelay(pdMS_TO_TICKS(1));
        uint8_t readback;
        ret = read_reg(inst, reg, &readback);
        if (ret != ESP_OK) return ret;
        bool got = (readback & (1u << pin)) != 0;
        if (got == level) {
            ESP_LOGI(TAG, "P%d_%d = %d VERIFIED (reg=0x%02X)", port, pin, level, readback);
        } else {
            ESP_LOGE(TAG, "P%d_%d VERIFY FAILED: expected %d got %d (reg=0x%02X)",
                     port, pin, level, (int)got, readback);
            return ESP_ERR_INVALID_RESPONSE;
        }
    } else {
        ESP_LOGD(TAG, "P%d_%d = %d (no verify)", port, pin, level);
    }
    return ESP_OK;
}

esp_err_t tca_read_pin_inst(tca6416a_inst_t *inst, tca_port_t port,
                             uint8_t pin, bool *level) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1 || pin > 7 || !level)
        return ESP_ERR_INVALID_ARG;
    uint8_t val;
    esp_err_t ret = read_reg(inst, TCA6416A_INPUT_PORT0 + (uint8_t)port, &val);
    if (ret == ESP_OK) *level = (val & (1u << pin)) != 0;
    return ret;
}

esp_err_t tca_read_config_register_inst(tca6416a_inst_t *inst, tca_port_t port,
                                         uint8_t *value) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1 || !value)
        return ESP_ERR_INVALID_ARG;
    return read_reg(inst, TCA6416A_CONFIG_PORT0 + (uint8_t)port, value);
}

esp_err_t tca_read_output_register_inst(tca6416a_inst_t *inst, tca_port_t port,
                                         uint8_t *value) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1 || !value)
        return ESP_ERR_INVALID_ARG;
    return read_reg(inst, TCA6416A_OUTPUT_PORT0 + (uint8_t)port, value);
}

esp_err_t tca_write_output_register_inst(tca6416a_inst_t *inst, tca_port_t port,
                                          uint8_t value) {
    if (!inst || !inst->dev_handle || port > TCA_PORT_1)
        return ESP_ERR_INVALID_ARG;
    return write_reg(inst, TCA6416A_OUTPUT_PORT0 + (uint8_t)port, value);
}

