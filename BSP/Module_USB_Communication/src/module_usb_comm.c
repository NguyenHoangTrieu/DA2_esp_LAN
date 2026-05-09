/**
 * @file module_usb_comm.c
 * @brief Generic USB CDC Communication Driver Implementation
 */

#include "module_usb_comm.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_USB";

/* ===== Internal Handle Structure ===== */

struct module_usb_comm_s {
  uint8_t stack_id;
  usb_cdc_line_coding_t line_coding;
  size_t rx_buffer_size;
  size_t tx_buffer_size;
  SemaphoreHandle_t mutex;
  bool initialized;
};

/* ===== Helper Functions ===== */

static inline bool is_valid_handle(module_usb_comm_handle_t handle) {
  return (handle != NULL && handle->initialized);
}

/* ===== Public API Implementation ===== */

esp_err_t module_usb_comm_init(const module_usb_config_t *config,
                                module_usb_comm_handle_t *handle) {
  if (!config || !handle) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (config->stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d (must be 0 or 1)", config->stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing USB CDC for Stack%d", config->stack_id);

  // Allocate handle
  module_usb_comm_handle_t usb_handle =
      (module_usb_comm_handle_t)calloc(1, sizeof(struct module_usb_comm_s));
  if (!usb_handle) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ESP_ERR_NO_MEM;
  }

  // Configure USB Serial/JTAG driver
  usb_serial_jtag_driver_config_t usb_config = {
      .rx_buffer_size = config->rx_buffer_size,
      .tx_buffer_size = config->tx_buffer_size,
  };

  // Install USB Serial/JTAG driver
  esp_err_t ret = usb_serial_jtag_driver_install(&usb_config);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    // ESP_ERR_INVALID_STATE means already installed
    ESP_LOGE(TAG, "Failed to install USB driver: %s", esp_err_to_name(ret));
    free(usb_handle);
    return ret;
  }

  // Create mutex for thread-safety
  usb_handle->mutex = xSemaphoreCreateMutex();
  if (!usb_handle->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex");
    usb_serial_jtag_driver_uninstall();
    free(usb_handle);
    return ESP_ERR_NO_MEM;
  }

  // Store configuration
  usb_handle->stack_id = config->stack_id;
  usb_handle->line_coding = config->line_coding;
  usb_handle->rx_buffer_size = config->rx_buffer_size;
  usb_handle->tx_buffer_size = config->tx_buffer_size;
  usb_handle->initialized = true;

  *handle = usb_handle;

  ESP_LOGI(TAG, "USB CDC initialized for Stack%d: RX_buf=%d, TX_buf=%d, bitrate=%lu",
           config->stack_id, config->rx_buffer_size, config->tx_buffer_size,
           config->line_coding.bit_rate);

  return ESP_OK;
}

esp_err_t module_usb_comm_send(module_usb_comm_handle_t handle,
                                const uint8_t *data, size_t len,
                                uint32_t timeout_ms) {
  if (!is_valid_handle(handle) || !data || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  // Lock mutex
  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire mutex for send");
    return ESP_ERR_TIMEOUT;
  }

  // Send data via USB Serial/JTAG
  int written = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(timeout_ms));

  // Unlock mutex
  xSemaphoreGive(handle->mutex);

  if (written < 0) {
    ESP_LOGE(TAG, "USB write failed");
    return ESP_FAIL;
  }

  if ((size_t)written != len) {
    ESP_LOGW(TAG, "USB write incomplete: %d/%d bytes", written, len);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "USB Stack%d sent %d bytes", handle->stack_id, written);
  return ESP_OK;
}

esp_err_t module_usb_comm_receive(module_usb_comm_handle_t handle,
                                   uint8_t *buffer, size_t max_len,
                                   size_t *received, uint32_t timeout_ms) {
  if (!is_valid_handle(handle) || !buffer || !received || max_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  *received = 0;

  // Lock mutex
  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire mutex for receive");
    return ESP_ERR_TIMEOUT;
  }

  // Read data from USB Serial/JTAG
  int length = usb_serial_jtag_read_bytes(buffer, max_len, pdMS_TO_TICKS(timeout_ms));

  // Unlock mutex
  xSemaphoreGive(handle->mutex);

  if (length < 0) {
    ESP_LOGE(TAG, "USB read failed");
    return ESP_FAIL;
  }

  *received = (size_t)length;

  if (length > 0) {
    ESP_LOGD(TAG, "USB Stack%d received %d bytes", handle->stack_id, length);
  }

  return ESP_OK;
}

esp_err_t module_usb_comm_flush(module_usb_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return ESP_ERR_INVALID_ARG;
  }

  // USB Serial/JTAG doesn't have explicit flush
  // Data is immediately transferred to USB buffer
  ESP_LOGD(TAG, "USB flush (no-op for USB Serial/JTAG)");
  
  return ESP_OK;
}

esp_err_t module_usb_comm_deinit(module_usb_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing USB Stack%d", handle->stack_id);

  // Delete mutex
  if (handle->mutex) {
    vSemaphoreDelete(handle->mutex);
  }

  // Uninstall driver
  esp_err_t ret = usb_serial_jtag_driver_uninstall();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to uninstall USB driver: %s", esp_err_to_name(ret));
  }

  // Mark as uninitialized
  handle->initialized = false;

  // Free handle
  free(handle);

  return ESP_OK;
}
