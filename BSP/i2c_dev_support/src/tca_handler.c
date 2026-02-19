/**
 * @file tca_handler.c
 * @brief TCA6424A I/O Expander Handler Implementation
 */

#include "tca_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_dev_support.h"

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
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Notify task to read port status
  xHigherPriorityTaskWoken = pdTRUE;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t tca_test_connection(void) {
  if (!tca_handle) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t test_val;
  esp_err_t ret = tca_read_register(TCA6424A_CONFIG_PORT0, &test_val);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "TCA6424A responding OK - Config0: 0x%02X", test_val);
  } else {
    ESP_LOGE(TAG, "TCA6424A not responding: %s", esp_err_to_name(ret));
  }

  return ret;
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

  // Install GPIO ISR service if not already installed, then add handler
  esp_err_t isr_ret = gpio_install_isr_service(0);
  if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "gpio_install_isr_service: %s (non-fatal)", esp_err_to_name(isr_ret));
  }
  gpio_isr_handler_add(TCA6424A_INT_PIN, tca_interrupt_handler, NULL);

  // Add TCA6424A device to bus
  esp_err_t ret = i2c_dev_support_add_device(TCA6424A_I2C_ADDR,
                                             TCA6424A_I2C_FREQ_HZ, &tca_handle);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add TCA6424A device: %s", esp_err_to_name(ret));
    return ret;
  }

  // Hardware reset
  tca_reset();

  // Test connection
  ret = tca_test_connection();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Device not responding at 0x%02X!", TCA6424A_I2C_ADDR);
    ESP_LOGE(TAG, "Check: 1) Wiring, 2) ADDR pin, 3) Power supply");
    return ret;
  }

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
  gpio_set_level(TCA6424A_RESET_PIN, 1);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(TCA6424A_RESET_PIN, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;
}

esp_err_t tca_configure_port(tca_port_t port, uint8_t config) {
  if (!tca_handle || port > TCA_PORT_2) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg = TCA6424A_CONFIG_PORT0 + port;
  esp_err_t ret = tca_write_register(reg, config);

  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Port %d configured: 0x%02X (0=output, 1=input)", port,
             config);
  } else {
    ESP_LOGE(TAG, "Failed to configure port %d: %s", port,
             esp_err_to_name(ret));
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
  esp_err_t ret = tca_write_register(reg, value);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write port %d: %s", port, esp_err_to_name(ret));
  }

  return ret;
}

esp_err_t tca_read_output_port(tca_port_t port, uint8_t *value) {
  if (!tca_handle || port > TCA_PORT_2 || !value) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg = TCA6424A_OUTPUT_PORT0 + port;
  return tca_read_register(reg, value);
}

esp_err_t tca_set_polarity(tca_port_t port, uint8_t polarity) {
  if (!tca_handle || port > TCA_PORT_2) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg = TCA6424A_POLARITY_PORT0 + port;
  return tca_write_register(reg, polarity);
}

esp_err_t tca_set_pin_verified(tca_port_t port, uint8_t pin, bool level,
                               bool verify) {
  if (pin > 7 || port > TCA_PORT_2) {
    return ESP_ERR_INVALID_ARG;
  }

  // Read OUTPUT register
  uint8_t port_value;
  esp_err_t ret = tca_read_output_port(port, &port_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read output port %d: %s", port,
             esp_err_to_name(ret));
    return ret;
  }

  // Modify bit
  if (level) {
    port_value |= (1 << pin);
  } else {
    port_value &= ~(1 << pin);
  }

  // Write back
  ret = tca_write_port(port, port_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write port %d: %s", port, esp_err_to_name(ret));
    return ret;
  }

  // Verify if requested
  if (verify) {
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for chip to update

    uint8_t verify_value;
    ret = tca_read_output_port(port, &verify_value);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to verify port %d: %s", port, esp_err_to_name(ret));
      return ret;
    }

    bool actual_state = (verify_value & (1 << pin)) != 0;
    if (actual_state == level) {
      ESP_LOGI(TAG, "Pin P%d_%d set to %d - VERIFIED ✓ (port=0x%02X)", port,
               pin, level, verify_value);
    } else {
      ESP_LOGE(TAG,
               "Pin P%d_%d VERIFY FAILED! Expected: %d, Got: %d (port=0x%02X)",
               port, pin, level, actual_state, verify_value);
      return ESP_ERR_INVALID_RESPONSE;
    }
  } else {
    ESP_LOGD(TAG, "Pin P%d_%d set to %d (no verify)", port, pin, level);
  }

  return ESP_OK;
}

esp_err_t tca_read_pin(tca_port_t port, uint8_t pin, bool *level) {
  if (pin > 7 || !level || port > TCA_PORT_2) {
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

esp_err_t tca_read_config_register(tca_port_t port, uint8_t *value) {
  if (!tca_handle || port > TCA_PORT_2 || !value) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t reg = TCA6424A_CONFIG_PORT0 + port;
  return tca_read_register(reg, value);
}

esp_err_t tca_read_output_register(tca_port_t port, uint8_t *value) {
  if (!tca_handle || port > TCA_PORT_2 || !value) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t reg = TCA6424A_OUTPUT_PORT0 + port;
  return tca_read_register(reg, value);
}

esp_err_t tca_write_output_register(tca_port_t port, uint8_t value) {
  if (!tca_handle || port > TCA_PORT_2) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t reg = TCA6424A_OUTPUT_PORT0 + port;
  return tca_write_register(reg, value);
}