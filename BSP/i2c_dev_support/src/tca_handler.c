/**
 * @file tca_handler.c
 * @brief TCA6424A I/O Expander Handler Implementation
 */

#include "tca_handler.h"
#include "i2c_dev_support.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TCA6424A";
static i2c_master_dev_handle_t tca_handle = NULL;
static tca_interrupt_callback_t interrupt_callback = NULL;

static esp_err_t tca_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_dev_support_write(tca_handle, data, 2, 1000);
}

static esp_err_t tca_read_register(uint8_t reg, uint8_t *value) {
    return i2c_dev_support_write_read(tca_handle, &reg, 1, value, 1, 1000);
}

static void IRAM_ATTR tca_interrupt_handler(void *arg) {
    // Handle interrupt in task context
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Notify task to read port status
    xHigherPriorityTaskWoken = pdTRUE;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t tca_init(void) {
    if (!i2c_dev_support_is_initialized()) {
        ESP_LOGE(TAG, "I2C not initialized. Call i2c_dev_support_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing TCA6424A at address 0x%02X", TCA6424A_I2C_ADDR);

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TCA6424A_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << TCA6424A_INT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TCA6424A_INT_PIN, tca_interrupt_handler, NULL);

    // Add TCA6424A device to bus
    esp_err_t ret = i2c_dev_support_add_device(TCA6424A_I2C_ADDR, TCA6424A_I2C_FREQ_HZ, &tca_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCA6424A device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Hardware reset
    tca_reset();

    // Configure all ports as inputs by default
    tca_configure_port(TCA_PORT_0, 0xFF);
    tca_configure_port(TCA_PORT_1, 0xFF);
    tca_configure_port(TCA_PORT_2, 0xFF);

    ESP_LOGI(TAG, "TCA6424A initialized successfully (INT=%d, RESET=%d)", 
             TCA6424A_INT_PIN, TCA6424A_RESET_PIN);
    return ESP_OK;
}

esp_err_t tca_deinit(void) {
    if (tca_handle) {
        gpio_isr_handler_remove(TCA6424A_INT_PIN);
        esp_err_t ret = i2c_dev_support_remove_device(tca_handle);
        tca_handle = NULL;
        ESP_LOGI(TAG, "TCA6424A deinitialized");
        return ret;
    }
    return ESP_OK;
}

esp_err_t tca_reset(void) {
    ESP_LOGI(TAG, "Performing hardware reset");
    gpio_set_level(TCA6424A_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TCA6424A_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t tca_configure_port(tca_port_t port, uint8_t config) {
    if (!tca_handle || port > TCA_PORT_2) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = TCA6424A_CONFIG_PORT0 + port;
    esp_err_t ret = tca_write_register(reg, config);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Port %d configured: 0x%02X", port, config);
    }
    return ret;
}

esp_err_t tca_read_port(tca_port_t port, uint8_t *value) {
    if (!tca_handle || port > TCA_PORT_2 || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = TCA6424A_INPUT_PORT0 + port;
    return tca_read_register(reg, value);
}

esp_err_t tca_write_port(tca_port_t port, uint8_t value) {
    if (!tca_handle || port > TCA_PORT_2) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = TCA6424A_OUTPUT_PORT0 + port;
    return tca_write_register(reg, value);
}

esp_err_t tca_set_polarity(tca_port_t port, uint8_t polarity) {
    if (!tca_handle || port > TCA_PORT_2) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = TCA6424A_POLARITY_PORT0 + port;
    return tca_write_register(reg, polarity);
}

esp_err_t tca_set_pin(tca_port_t port, uint8_t pin, bool level) {
    if (pin > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port_value;
    esp_err_t ret = tca_read_port(port, &port_value);
    if (ret != ESP_OK) {
        return ret;
    }

    if (level) {
        port_value |= (1 << pin);
    } else {
        port_value &= ~(1 << pin);
    }

    return tca_write_port(port, port_value);
}

esp_err_t tca_read_pin(tca_port_t port, uint8_t pin, bool *level) {
    if (pin > 7 || !level) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port_value;
    esp_err_t ret = tca_read_port(port, &port_value);
    if (ret == ESP_OK) {
        *level = (port_value & (1 << pin)) != 0;
    }
    return ret;
}

esp_err_t tca_register_interrupt_callback(tca_interrupt_callback_t callback) {
    interrupt_callback = callback;
    return ESP_OK;
}
