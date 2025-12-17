/**
 * @file i2c_dev_support.c
 * @brief Common I2C Device Support Layer Implementation
 */

#include "i2c_dev_support.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "I2C_SUPPORT";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static bool is_initialized = false;

esp_err_t i2c_dev_support_init(void) {
    if (is_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C Master on port %d (SDA=%d, SCL=%d)", 
             I2C_MASTER_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    is_initialized = true;
    ESP_LOGI(TAG, "I2C Master initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_dev_support_deinit(void) {
    if (!is_initialized) {
        ESP_LOGW(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (i2c_bus_handle) {
        esp_err_t ret = i2c_del_master_bus(i2c_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
            return ret;
        }
        i2c_bus_handle = NULL;
    }

    is_initialized = false;
    ESP_LOGI(TAG, "I2C Master deinitialized");
    return ESP_OK;
}

bool i2c_dev_support_is_initialized(void) {
    return is_initialized;
}

i2c_master_bus_handle_t i2c_dev_support_get_bus_handle(void) {
    return i2c_bus_handle;
}

esp_err_t i2c_dev_support_add_device(uint8_t device_addr, uint32_t scl_speed_hz,
                                     i2c_master_dev_handle_t *dev_handle) {
    if (!is_initialized || !i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized. Call i2c_dev_support_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (!dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = scl_speed_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", device_addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Device 0x%02X added successfully", device_addr);
    return ESP_OK;
}

esp_err_t i2c_dev_support_remove_device(i2c_master_dev_handle_t dev_handle) {
    if (!dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Device removed successfully");
    return ESP_OK;
}

esp_err_t i2c_dev_support_write(i2c_master_dev_handle_t dev_handle,
                                const uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (!dev_handle || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit(dev_handle, data, len, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t i2c_dev_support_read(i2c_master_dev_handle_t dev_handle,
                               uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (!dev_handle || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_receive(dev_handle, data, len, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t i2c_dev_support_write_read(i2c_master_dev_handle_t dev_handle,
                                     const uint8_t *write_data, size_t write_len,
                                     uint8_t *read_data, size_t read_len,
                                     uint32_t timeout_ms) {
    if (!dev_handle || !write_data || !read_data) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(dev_handle, write_data, write_len,
                                      read_data, read_len, pdMS_TO_TICKS(timeout_ms));
}
