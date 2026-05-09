/**
 * @file module_i2c_comm.c
 * @brief Generic I2C Communication Driver Implementation (ESP-IDF v6 API)
 */

#include "module_i2c_comm.h"
#include "i2c_dev_support.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_I2C";

/* ===== Internal Handle Structure ===== */

struct module_i2c_comm_s {
  uint8_t stack_id;
  i2c_master_dev_handle_t dev_handle; // device registered on shared i2c_dev_support bus
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

  // Shared I2C bus must be initialized by i2c_dev_support before this call
  if (!i2c_dev_support_is_initialized()) {
    ESP_LOGE(TAG, "Shared I2C bus not initialized. Call i2c_dev_support_init() first");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Registering I2C device for Stack%d: addr=0x%02X, clock=%lu Hz on shared bus",
           config->stack_id, config->device_address, config->clock_speed_hz);

  // Allocate handle
  module_i2c_comm_handle_t i2c_handle =
      (module_i2c_comm_handle_t)calloc(1, sizeof(struct module_i2c_comm_s));
  if (!i2c_handle) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ESP_ERR_NO_MEM;
  }

  // Add device to the shared I2C bus managed by i2c_dev_support
  i2c_master_dev_handle_t dev_handle;
  esp_err_t ret = i2c_dev_support_add_device(config->device_address,
                                             config->clock_speed_hz,
                                             &dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add I2C device 0x%02X: %s",
             config->device_address, esp_err_to_name(ret));
    free(i2c_handle);
    return ret;
  }

  // Store configuration
  i2c_handle->stack_id = config->stack_id;
  i2c_handle->dev_handle = dev_handle;
  i2c_handle->device_address = config->device_address;
  i2c_handle->clock_speed_hz = config->clock_speed_hz;
  i2c_handle->initialized = true;

  *handle = i2c_handle;

  ESP_LOGI(TAG, "I2C device registered for Stack%d: addr=0x%02X on shared bus",
           config->stack_id, config->device_address);

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

  ESP_LOGI(TAG, "Deinitializing I2C device for Stack%d", handle->stack_id);

  // Remove device from shared bus (bus itself is owned by i2c_dev_support)
  esp_err_t ret = i2c_dev_support_remove_device(handle->dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
  }

  // Mark as uninitialized
  handle->initialized = false;

  // Free handle
  free(handle);

  return ESP_OK;
}
