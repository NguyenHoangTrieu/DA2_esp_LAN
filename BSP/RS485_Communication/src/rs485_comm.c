/**
 * @file rs485_comm.c
 * @brief RS485 Communication Driver Implementation
 */

#include "rs485_comm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stack_handler.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "RS485_COMM";

/* ===== Internal Handle Structure ===== */
struct rs485_comm_handle_s {
  uint8_t stack_id;
  int uart_port;
  int baud_rate;
  rs485_mode_t current_mode;
  bool is_initialized;
};

/* ===== Helper Functions ===== */

/**
 * @brief Get current active stack ID based on global stack types
 * @return uint8_t Stack ID (0 or 1), default to 0 if no RS485 stack found
 */
static uint8_t get_active_rs485_stack(void) {
  if (g_stack_1_type == STACK_COMM_TYPE_RS485) {
    return 0; // Stack 1
  } else if (g_stack_2_type == STACK_COMM_TYPE_RS485) {
    return 1; // Stack 2
  }
  return 0; // Default to Stack 1
}

/**
 * @brief Get UART port for current stack
 */
static int get_rs485_uart_port(void) {
  return (get_active_rs485_stack() == 0) ? RS485_UART_PORT_STACK_1
                                         : RS485_UART_PORT_STACK_2;
}

/**
 * @brief Get UART TX pin for current stack
 */
static int get_rs485_uart_tx_pin(void) {
  return (get_active_rs485_stack() == 0) ? RS485_UART_TX_PIN_STACK_1
                                         : RS485_UART_TX_PIN_STACK_2;
}

/**
 * @brief Get UART RX pin for current stack
 */
static int get_rs485_uart_rx_pin(void) {
  return (get_active_rs485_stack() == 0) ? RS485_UART_RX_PIN_STACK_1
                                         : RS485_UART_RX_PIN_STACK_2;
}

/**
 * @brief Get DE GPIO pin for current stack
 */
static stack_gpio_pin_num_t get_rs485_de_gpio(void) {
  return (get_active_rs485_stack() == 0) ? RS485_DE_GPIO_STACK_1
                                         : RS485_DE_GPIO_STACK_2;
}

/**
 * @brief Get RE GPIO pin for current stack
 */
static stack_gpio_pin_num_t get_rs485_re_gpio(void) {
  return (get_active_rs485_stack() == 0) ? RS485_RE_GPIO_STACK_1
                                         : RS485_RE_GPIO_STACK_2;
}

/* ===== Private Functions ===== */

/**
 * @brief Internal function to set RS485 mode via GPIO
 */
static esp_err_t rs485_set_mode_internal(rs485_comm_handle_t handle,
                                         rs485_mode_t mode) {
  if (!handle || !handle->is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret;
  uint8_t stack_id = handle->stack_id;
  
  // Logic: TX (1) -> Pin High. RX (0) -> Pin Low.
  bool level = (mode == RS485_MODE_ONLY_SEND) ? true : false; 

  // Set DE
  ret = stack_handler_gpio_write(stack_id, get_rs485_de_gpio(), level);
  if (ret != ESP_OK) return ret;

  // Set RE (Cùng mức logic với DE)
  ret = stack_handler_gpio_write(stack_id, get_rs485_re_gpio(), level);
  
  if (ret == ESP_OK) {
    handle->current_mode = mode;
  }

  return ret;
}

/* ===== Public API Implementation ===== */

esp_err_t rs485_comm_init(const rs485_comm_config_t *config,
                          rs485_comm_handle_t *handle) {
  if (!config || !handle) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing RS485 driver");

  // Allocate handle
  rs485_comm_handle_t h =
      (rs485_comm_handle_t)calloc(1, sizeof(struct rs485_comm_handle_s));
  if (!h) {
    ESP_LOGE(TAG, "Failed to allocate memory");
    return ESP_ERR_NO_MEM;
  }

  // Get stack configuration
  uint8_t stack_id = get_active_rs485_stack();
  int uart_port = get_rs485_uart_port();
  int tx_pin = get_rs485_uart_tx_pin();
  int rx_pin = get_rs485_uart_rx_pin();

  // Store configuration
  h->stack_id = stack_id;
  h->uart_port = uart_port;
  h->baud_rate =
      (config->baud_rate > 0) ? config->baud_rate : RS485_DEFAULT_BAUD_RATE;
  h->current_mode = RS485_MODE_ONLY_RECEIVE;

  // Configure UART
  uart_config_t uart_config = {
      .baud_rate = h->baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret = uart_param_config(uart_port, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
    free(h);
    return ret;
  }

  // Set UART pins
  ret = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
    free(h);
    return ret;
  }

  // Install UART driver
  int rx_buf = (config->rx_buffer_size > 0) ? config->rx_buffer_size
                                            : RS485_DEFAULT_RX_BUF_SIZE;
  int tx_buf = (config->tx_buffer_size > 0) ? config->tx_buffer_size
                                            : RS485_DEFAULT_TX_BUF_SIZE;

  ret = uart_driver_install(uart_port, rx_buf, tx_buf, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
    free(h);
    return ret;
  }

  // Configure GPIO pins for DE and RE via stack_handler
  ret = stack_handler_gpio_set_direction(stack_id, get_rs485_de_gpio(), true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure DE pin");
    uart_driver_delete(uart_port);
    free(h);
    return ret;
  }

  ret = stack_handler_gpio_set_direction(stack_id, get_rs485_re_gpio(), true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure RE pin");
    uart_driver_delete(uart_port);
    free(h);
    return ret;
  }

  h->is_initialized = true;
  *handle = h;

  // Set to receive mode by default
  rs485_set_mode_internal(h, RS485_MODE_ONLY_RECEIVE);

  uart_flush_input(uart_port);

  ESP_LOGI(TAG,
           "RS485 initialized: Stack%d, UART%d, baud=%d, TX=%d, RX=%d, "
           "DE=GPIO%d, RE=GPIO%d",
           stack_id + 1, uart_port, h->baud_rate, tx_pin, rx_pin,
           get_rs485_de_gpio() + 1, get_rs485_re_gpio() + 1);

  return ESP_OK;
}

esp_err_t rs485_comm_write(rs485_comm_handle_t handle, const uint8_t *data,
                           size_t length, uint32_t timeout_ms) {
  if (!handle || !handle->is_initialized) {
    ESP_LOGE(TAG, "Invalid handle or not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!data || length == 0) {
    ESP_LOGE(TAG, "Invalid data or length");
    return ESP_ERR_INVALID_ARG;
  }

  // Switch to transmit mode
  esp_err_t ret = rs485_set_mode_internal(handle, RS485_MODE_ONLY_SEND);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to switch to TX mode");
    return ret;
  }

  // Write data
  int written = uart_write_bytes(handle->uart_port, data, length);
  if (written < 0) {
    ESP_LOGE(TAG, "UART write failed");
    rs485_set_mode_internal(handle, RS485_MODE_ONLY_RECEIVE);
    return ESP_FAIL;
  }

  // Wait for transmission to complete
  ret = uart_wait_tx_done(handle->uart_port, pdMS_TO_TICKS(timeout_ms));
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "UART TX timeout");
    rs485_set_mode_internal(handle, RS485_MODE_ONLY_RECEIVE);
    return ESP_ERR_TIMEOUT;
  }

  // Small delay to ensure last bit is transmitted
  vTaskDelay(pdMS_TO_TICKS(2));

  // Switch back to receive mode
  ret = rs485_set_mode_internal(handle, RS485_MODE_ONLY_RECEIVE);

  ESP_LOGD(TAG, "Written %d bytes to RS485", written);
  return ESP_OK;
}

esp_err_t rs485_comm_read(rs485_comm_handle_t handle, uint8_t *buffer,
                          size_t length, size_t *actual_length,
                          uint32_t timeout_ms) {
  if (!handle || !handle->is_initialized) {
    ESP_LOGE(TAG, "Invalid handle or not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!buffer || !actual_length) {
    ESP_LOGE(TAG, "Invalid buffer or actual_length pointer");
    return ESP_ERR_INVALID_ARG;
  }

  // Ensure we're in receive mode
  if (handle->current_mode != RS485_MODE_ONLY_RECEIVE) {
    rs485_set_mode_internal(handle, RS485_MODE_ONLY_RECEIVE);
  }

  // Read data from UART
  int len = uart_read_bytes(handle->uart_port, buffer, length,
                            pdMS_TO_TICKS(timeout_ms));

  if (len < 0) {
    *actual_length = 0;
    ESP_LOGE(TAG, "UART read failed");
    return ESP_FAIL;
  }

  *actual_length = (size_t)len;

  if (len > 0) {
    ESP_LOGD(TAG, "Read %d bytes from RS485", len);
    return ESP_OK;
  } else {
    return ESP_ERR_TIMEOUT;
  }
}

size_t rs485_comm_available(rs485_comm_handle_t handle) {
  if (!handle || !handle->is_initialized) {
    ESP_LOGW(TAG, "Invalid handle or not initialized");
    return 0;
  }

  size_t available = 0;
  esp_err_t ret = uart_get_buffered_data_len(handle->uart_port, &available);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to get buffered data length");
    return 0;
  }

  return available;
}

esp_err_t rs485_comm_flush(rs485_comm_handle_t handle) {
  if (!handle || !handle->is_initialized) {
    ESP_LOGE(TAG, "Invalid handle or not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = uart_flush_input(handle->uart_port);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART flush failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "RS485 UART RX buffer flushed");
  return ESP_OK;
}

esp_err_t rs485_comm_set_mode(rs485_comm_handle_t handle, rs485_mode_t mode) {
  if (!handle || !handle->is_initialized) {
    ESP_LOGE(TAG, "Invalid handle or not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  return rs485_set_mode_internal(handle, mode);
}
