/**
 * @file module_i2c_comm.c
 * @brief Generic I2C Communication Driver Implementation (ESP-IDF v6 API)
 */

#include "module_i2c_comm.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_I2C";

/* ===== Internal Handle Structure ===== */

struct module_i2c_comm_s {
  uint8_t stack_id;
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t dev_handle;
  uint8_t device_address;
  uint32_t clock_speed_hz;
  bool initialized;
};

/* ===== Helper Functions ===== */

static inline bool is_valid_handle(module_i2c_comm_handle_t handle) {
  return (handle != NULL && handle->initialized);
}

/* ===== Public API Implementation ===== */

esp_err_t module_i2c_comm_init(const module_i2c_config_t *config,
                               module_i2c_comm_handle_t *handle) {
  if (!config || !handle) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (config->stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d (must be 0 or 1)", config->stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  // Get hardcoded pins based on stack_id
  i2c_port_num_t i2c_port;
  int sda_pin, scl_pin;

  if (config->stack_id == 0) {
    i2c_port = STACK0_I2C_PORT;
    sda_pin = STACK0_I2C_SDA_PIN;
    scl_pin = STACK0_I2C_SCL_PIN;
  } else {
    i2c_port = STACK1_I2C_PORT;
    sda_pin = STACK1_I2C_SDA_PIN;
    scl_pin = STACK1_I2C_SCL_PIN;
  }

  ESP_LOGI(
      TAG, "Initializing I2C for Stack%d: port=%d, SDA=%d, SCL=%d, addr=0x%02X",
      config->stack_id, i2c_port, sda_pin, scl_pin, config->device_address);

  // Allocate handle
  module_i2c_comm_handle_t i2c_handle =
      (module_i2c_comm_handle_t)calloc(1, sizeof(struct module_i2c_comm_s));
  if (!i2c_handle) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ESP_ERR_NO_MEM;
  }

  // Configure I2C master bus
  i2c_master_bus_config_t bus_config = {
      .i2c_port = i2c_port,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = config->pullup_enable,
  };

  i2c_master_bus_handle_t bus_handle;
  esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    free(i2c_handle);
    return ret;
  }

  // Configure I2C device
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->device_address,
      .scl_speed_hz = config->clock_speed_hz,
  };

  i2c_master_dev_handle_t dev_handle;
  ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
    i2c_del_master_bus(bus_handle);
    free(i2c_handle);
    return ret;
  }

  // Store configuration
  i2c_handle->stack_id = config->stack_id;
  i2c_handle->bus_handle = bus_handle;
  i2c_handle->dev_handle = dev_handle;
  i2c_handle->device_address = config->device_address;
  i2c_handle->clock_speed_hz = config->clock_speed_hz;
  i2c_handle->initialized = true;

  *handle = i2c_handle;

  ESP_LOGI(TAG, "I2C initialized for Stack%d: addr=0x%02X, clock=%lu Hz",
           config->stack_id, config->device_address, config->clock_speed_hz);

  return ESP_OK;
}

esp_err_t module_i2c_comm_write(module_i2c_comm_handle_t handle,
                                const uint8_t *data, size_t len,
                                uint32_t timeout_ms) {
  if (!is_valid_handle(handle) || !data || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret =
      i2c_master_transmit(handle->dev_handle, data, len, timeout_ms);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "I2C wrote %d bytes to 0x%02X", len, handle->device_address);
  return ESP_OK;
}

esp_err_t module_i2c_comm_read(module_i2c_comm_handle_t handle, uint8_t *buffer,
                               size_t len, uint32_t timeout_ms) {
  if (!is_valid_handle(handle) || !buffer || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret =
      i2c_master_receive(handle->dev_handle, buffer, len, timeout_ms);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "I2C read %d bytes from 0x%02X", len, handle->device_address);
  return ESP_OK;
}

esp_err_t module_i2c_comm_write_read(module_i2c_comm_handle_t handle,
                                     uint8_t reg_addr, uint8_t *buffer,
                                     size_t len, uint32_t timeout_ms) {
  if (!is_valid_handle(handle) || !buffer || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = i2c_master_transmit_receive(handle->dev_handle, &reg_addr, 1,
                                              buffer, len, timeout_ms);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C write-read failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "I2C write-read %d bytes from reg 0x%02X", len, reg_addr);
  return ESP_OK;
}

esp_err_t module_i2c_comm_deinit(module_i2c_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing I2C for Stack%d", handle->stack_id);

  // Remove device from bus
  esp_err_t ret = i2c_master_bus_rm_device(handle->dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
  }

  // Delete bus
  ret = i2c_del_master_bus(handle->bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
  }

  // Mark as uninitialized
  handle->initialized = false;

  // Free handle
  free(handle);

  return ESP_OK;
}
