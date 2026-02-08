/**
 * @file module_i2c_comm.c
 * @brief Generic I2C Communication Driver Implementation
 */

#include "module_i2c_comm.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_I2C";

/* ===== Internal Handle Structure ===== */

struct module_i2c_comm_s {
  uint8_t stack_id;
  i2c_port_t port;
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
  i2c_port_t i2c_port;
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

  ESP_LOGI(TAG, "Initializing I2C for Stack%d: port=%d, SDA=%d, SCL=%d, addr=0x%02X",
           config->stack_id, i2c_port, sda_pin, scl_pin, config->device_address);

  // Allocate handle
  module_i2c_comm_handle_t i2c_handle =
      (module_i2c_comm_handle_t)calloc(1, sizeof(struct module_i2c_comm_s));
  if (!i2c_handle) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ESP_ERR_NO_MEM;
  }

  // Configure I2C master
  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .sda_pullup_en = config->pullup_enable,
      .scl_pullup_en = config->pullup_enable,
      .master.clk_speed = config->clock_speed_hz,
  };

  // Configure I2C
  esp_err_t ret = i2c_param_config(i2c_port, &i2c_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
    free(i2c_handle);
    return ret;
  }

  // Install I2C driver
  ret = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    // ESP_ERR_INVALID_STATE means driver already installed
    ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
    free(i2c_handle);
    return ret;
  }

  // Store configuration
  i2c_handle->stack_id = config->stack_id;
  i2c_handle->port = i2c_port;
  i2c_handle->device_address = config->device_address;
  i2c_handle->clock_speed_hz = config->clock_speed_hz;
  i2c_handle->initialized = true;

  *handle = i2c_handle;

  ESP_LOGI(TAG, "I2C initialized for Stack%d: port=%d, SDA=%d, SCL=%d, addr=0x%02X, clock=%lu Hz",
           config->stack_id, i2c_port, sda_pin, scl_pin,
           config->device_address, config->clock_speed_hz);

  return ESP_OK;
}

esp_err_t module_i2c_comm_write(module_i2c_comm_handle_t handle,
                                const uint8_t *data, size_t len,
                                uint32_t timeout_ms) {
  if (!is_valid_handle(handle) || !data || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  // Create I2C command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Start + Device address + Write
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_WRITE,
                        true);

  // Write data
  i2c_master_write(cmd, data, len, true);

  // Stop
  i2c_master_stop(cmd);

  // Execute transaction
  esp_err_t ret =
      i2c_master_cmd_begin(handle->port, cmd, pdMS_TO_TICKS(timeout_ms));

  // Delete command link
  i2c_cmd_link_delete(cmd);

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

  // Create I2C command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Start + Device address + Read
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_READ,
                        true);

  // Read data
  if (len > 1) {
    i2c_master_read(cmd, buffer, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, buffer + len - 1, I2C_MASTER_NACK);

  // Stop
  i2c_master_stop(cmd);

  // Execute transaction
  esp_err_t ret =
      i2c_master_cmd_begin(handle->port, cmd, pdMS_TO_TICKS(timeout_ms));

  // Delete command link
  i2c_cmd_link_delete(cmd);

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

  // Create I2C command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Start + Device address + Write + Register address
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, reg_addr, true);

  // Repeated start + Device address + Read
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_READ,
                        true);

  // Read data
  if (len > 1) {
    i2c_master_read(cmd, buffer, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, buffer + len - 1, I2C_MASTER_NACK);

  // Stop
  i2c_master_stop(cmd);

  // Execute transaction
  esp_err_t ret =
      i2c_master_cmd_begin(handle->port, cmd, pdMS_TO_TICKS(timeout_ms));

  // Delete command link
  i2c_cmd_link_delete(cmd);

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

  ESP_LOGI(TAG, "Deinitializing I2C%d", handle->port);

  // Uninstall driver
  esp_err_t ret = i2c_driver_delete(handle->port);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to delete I2C driver: %s", esp_err_to_name(ret));
  }

  // Mark as uninitialized
  handle->initialized = false;

  // Free handle
  free(handle);

  return ESP_OK;
}
