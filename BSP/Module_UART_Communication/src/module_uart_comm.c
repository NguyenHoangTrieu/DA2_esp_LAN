/**
 * @file module_uart_comm.c
 * @brief Generic UART Communication Driver Implementation
 */

#include "module_uart_comm.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_UART";

/* ===== Internal Handle Structure ===== */

struct module_uart_comm_s {
  uint8_t stack_id;
  uart_port_t port;
  uint32_t baudrate;
  QueueHandle_t uart_queue;
  SemaphoreHandle_t mutex;
  size_t rx_buffer_size;
  size_t tx_buffer_size;
  bool initialized;
};

/* ===== Helper Functions ===== */

static inline bool is_valid_handle(module_uart_comm_handle_t handle) {
  return (handle != NULL && handle->initialized);
}

/* ===== Public API Implementation ===== */

esp_err_t module_uart_comm_init(const module_uart_config_t *config,
                                module_uart_comm_handle_t *handle) {
  // Validate parameters
  if (!config || !handle) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (config->stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d (must be 0 or 1)", config->stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  // Get hardcoded pins and port based on stack_id
  uart_port_t uart_port;
  int tx_pin, rx_pin;
  
  if (config->stack_id == 0) {
    uart_port = STACK0_UART_PORT;
    tx_pin = STACK0_UART_TX_PIN;
    rx_pin = STACK0_UART_RX_PIN;
  } else {
    uart_port = STACK1_UART_PORT;
    tx_pin = STACK1_UART_TX_PIN;
    rx_pin = STACK1_UART_RX_PIN;
  }

  ESP_LOGI(TAG, "Initializing UART for Stack%d: port=%d, TX=%d, RX=%d",
           config->stack_id, uart_port, tx_pin, rx_pin);

  // Allocate handle
  module_uart_comm_handle_t uart_handle =
      (module_uart_comm_handle_t)calloc(1, sizeof(struct module_uart_comm_s));
  if (!uart_handle) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ESP_ERR_NO_MEM;
  }

  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = config->baudrate,
      .data_bits = UART_DATA_8_BITS,
      .parity = config->parity,
      .stop_bits = config->stop_bits,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // Install UART driver
  esp_err_t ret = uart_driver_install(uart_port, config->rx_buffer_size,
                                      config->tx_buffer_size, 10,
                                      &uart_handle->uart_queue, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
    free(uart_handle);
    return ret;
  }

  // Set UART parameters
  ret = uart_param_config(uart_port, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure UART params: %s", esp_err_to_name(ret));
    uart_driver_delete(uart_port);
    free(uart_handle);
    return ret;
  }

  // Set UART pins
  ret = uart_set_pin(uart_port, tx_pin, rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
    uart_driver_delete(uart_port);
    free(uart_handle);
    return ret;
  }

  // Create mutex for thread-safety
  uart_handle->mutex = xSemaphoreCreateMutex();
  if (!uart_handle->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex");
    uart_driver_delete(uart_port);
    free(uart_handle);
    return ESP_ERR_NO_MEM;
  }

  // Store configuration
  uart_handle->stack_id = config->stack_id;
  uart_handle->port = uart_port;
  uart_handle->baudrate = config->baudrate;
  uart_handle->rx_buffer_size = config->rx_buffer_size;
  uart_handle->tx_buffer_size = config->tx_buffer_size;
  uart_handle->initialized = true;

  *handle = uart_handle;

  ESP_LOGI(TAG, "UART%d initialized for Stack%d: TX=%d, RX=%d, baud=%lu",
           uart_port, config->stack_id, tx_pin, rx_pin, config->baudrate);

  return ESP_OK;
}

esp_err_t module_uart_comm_send(module_uart_comm_handle_t handle,
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

  // Send data
  int written = uart_write_bytes(handle->port, data, len);

  // Unlock mutex
  xSemaphoreGive(handle->mutex);

  if (written < 0) {
    ESP_LOGE(TAG, "UART write failed");
    return ESP_FAIL;
  }

  if ((size_t)written != len) {
    ESP_LOGW(TAG, "UART write incomplete: %d/%d bytes", written, len);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "UART%d sent %d bytes", handle->port, written);
  return ESP_OK;
}

esp_err_t module_uart_comm_receive(module_uart_comm_handle_t handle,
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

  // Read data with timeout
  int length =
      uart_read_bytes(handle->port, buffer, max_len, pdMS_TO_TICKS(timeout_ms));

  // Unlock mutex
  xSemaphoreGive(handle->mutex);

  if (length < 0) {
    ESP_LOGE(TAG, "UART read failed");
    return ESP_FAIL;
  }

  *received = (size_t)length;

  if (length > 0) {
    ESP_LOGD(TAG, "UART%d received %d bytes", handle->port, length);
  }

  return ESP_OK;
}

esp_err_t module_uart_comm_flush(module_uart_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return ESP_ERR_INVALID_ARG;
  }

  /* uart_flush_input clears the software RX ring buffer (stale received bytes).
   * This is distinct from uart_flush() which flushes the TX FIFO. */
  esp_err_t ret = uart_flush_input(handle->port);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART RX flush failed: %s", esp_err_to_name(ret));
  }

  return ret;
}

size_t module_uart_comm_available(module_uart_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return 0;
  }

  size_t available = 0;
  uart_get_buffered_data_len(handle->port, &available);

  return available;
}

esp_err_t module_uart_comm_deinit(module_uart_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing UART%d", handle->port);

  // Delete mutex
  if (handle->mutex) {
    vSemaphoreDelete(handle->mutex);
  }

  // Uninstall driver
  esp_err_t ret = uart_driver_delete(handle->port);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to delete UART driver: %s", esp_err_to_name(ret));
  }

  // Mark as uninitialized
  handle->initialized = false;

  // Free handle
  free(handle);

  return ESP_OK;
}
