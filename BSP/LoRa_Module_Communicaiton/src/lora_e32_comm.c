/**
 * @file lora_e32_comm.c
 * @brief E32 LoRa Module Communication Driver Implementation (Broadcast API)
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_e32_comm.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "LORA_E32_COMM";

// ===== Internal Handle Structure =====
struct lora_e32_comm_handle_s {
  lora_e32_comm_config_t config;
  lora_e32_comm_interface_t interface;
  e32_mode_t current_mode;
  bool is_initialized;
};

// ===== UART Interface Implementation =====

static esp_err_t uart_init_impl(void *config_ptr, void **user_ctx) {
  if (config_ptr == NULL || user_ctx == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  lora_e32_comm_uart_config_t *uart_cfg =
      (lora_e32_comm_uart_config_t *)config_ptr;

  uart_config_t uart_config = {.baud_rate = uart_cfg->baud_rate,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .source_clk = UART_SCLK_DEFAULT};

  esp_err_t ret = uart_param_config(uart_cfg->uart_port, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_set_pin(uart_cfg->uart_port, uart_cfg->tx_pin, uart_cfg->rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_driver_install(uart_cfg->uart_port, uart_cfg->rx_buffer_size,
                            uart_cfg->tx_buffer_size, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Store UART port in user context
  *user_ctx = (void *)(intptr_t)uart_cfg->uart_port;

  ESP_LOGI(TAG, "UART initialized: port=%d, baud=%d, TX=%d, RX=%d",
           uart_cfg->uart_port, uart_cfg->baud_rate, uart_cfg->tx_pin,
           uart_cfg->rx_pin);

  return ESP_OK;
}

static esp_err_t uart_deinit_impl(void *user_ctx) {
  if (user_ctx == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  int uart_port = (int)(intptr_t)user_ctx;
  return uart_driver_delete(uart_port);
}

static esp_err_t uart_write_impl(void *user_ctx, const uint8_t *data,
                                 size_t length, uint32_t timeout_ms) {
  if (user_ctx == NULL || data == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  int uart_port = (int)(intptr_t)user_ctx;
  int written = uart_write_bytes(uart_port, data, length);

  if (written < 0) {
    ESP_LOGE(TAG, "UART write failed");
    return ESP_FAIL;
  }

  // Wait for TX done
  uart_wait_tx_done(uart_port, pdMS_TO_TICKS(timeout_ms));

  return ESP_OK;
}

static esp_err_t uart_read_impl(void *user_ctx, uint8_t *data, size_t length,
                                size_t *actual_length, uint32_t timeout_ms) {
  if (user_ctx == NULL || data == NULL || actual_length == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  int uart_port = (int)(intptr_t)user_ctx;
  int len = uart_read_bytes(uart_port, data, length, pdMS_TO_TICKS(timeout_ms));

  if (len < 0) {
    *actual_length = 0;
    return ESP_FAIL;
  }

  *actual_length = (size_t)len;
  return (len > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t uart_flush_impl(void *user_ctx) {
  if (user_ctx == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  int uart_port = (int)(intptr_t)user_ctx;
  return uart_flush_input(uart_port);
}

static size_t uart_available_impl(void *user_ctx) {
  if (user_ctx == NULL) {
    return 0;
  }

  int uart_port = (int)(intptr_t)user_ctx;
  size_t available = 0;
  uart_get_buffered_data_len(uart_port, &available);
  return available;
}

// ===== Create UART Interface =====
lora_e32_comm_interface_t lora_e32_comm_create_uart_interface(void) {
  lora_e32_comm_interface_t interface = {.user_ctx = NULL,
                                         .init = uart_init_impl,
                                         .deinit = uart_deinit_impl,
                                         .write = uart_write_impl,
                                         .read = uart_read_impl,
                                         .flush = uart_flush_impl,
                                         .available = uart_available_impl};
  return interface;
}

// Predefined instance (optional external reference)
lora_e32_comm_interface_t lora_e32_comm_uart_interface;

// ===== Internal Helper Functions =====

static esp_err_t set_gpio_mode_pins(lora_e32_comm_handle_t handle,
                                    e32_mode_t mode) {
  if (handle->config.gpio_config.m0_pin < 0 ||
      handle->config.gpio_config.m1_pin < 0) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t m0 = (mode & 0x01) ? 1 : 0;
  uint8_t m1 = (mode & 0x02) ? 1 : 0;

  gpio_set_level(handle->config.gpio_config.m0_pin, m0);
  gpio_set_level(handle->config.gpio_config.m1_pin, m1);

  return ESP_OK;
}

static bool is_aux_high_internal(lora_e32_comm_handle_t handle) {
  if (handle->config.gpio_config.aux_pin < 0) {
    return true; // Assume ready if no AUX pin
  }
  return gpio_get_level(handle->config.gpio_config.aux_pin) == 1;
}

// ===== API Implementation =====

lora_e32_comm_status_t lora_e32_comm_init(const lora_e32_comm_config_t *config,
                                          lora_e32_comm_handle_t *handle) {
  if (config == NULL || handle == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing LoRa E32 broadcast driver");

  // Allocate handle
  lora_e32_comm_handle_t h =
      (lora_e32_comm_handle_t)calloc(1, sizeof(struct lora_e32_comm_handle_s));
  if (h == NULL) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return LORA_E32_COMM_ERR_NO_MEM;
  }

  // Copy configuration
  memcpy(&h->config, config, sizeof(lora_e32_comm_config_t));
  h->interface = config->interface;
  h->current_mode = E32_MODE_SLEEP;
  h->is_initialized = false;

  // Initialize GPIO pins
  if (config->gpio_config.m0_pin >= 0) {
    gpio_config_t io_conf = {.pin_bit_mask =
                                 (1ULL << config->gpio_config.m0_pin),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
  }

  if (config->gpio_config.m1_pin >= 0) {
    gpio_config_t io_conf = {.pin_bit_mask =
                                 (1ULL << config->gpio_config.m1_pin),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
  }

  if (config->gpio_config.aux_pin >= 0) {
    gpio_config_t io_conf = {.pin_bit_mask =
                                 (1ULL << config->gpio_config.aux_pin),
                             .mode = GPIO_MODE_INPUT,
                             .pull_up_en = GPIO_PULLUP_ENABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
  }

  // Initialize communication interface
  esp_err_t ret =
      h->interface.init(config->interface_config, &h->interface.user_ctx);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize communication interface");
    free(h);
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  // Set initial mode to SLEEP for configuration
  set_gpio_mode_pins(h, E32_MODE_SLEEP);
  vTaskDelay(pdMS_TO_TICKS(E32_MODE_SWITCH_TIME_MS));

  h->is_initialized = true;
  *handle = h;

  ESP_LOGI(TAG, "LoRa E32 broadcast driver initialized");
  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_deinit(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing LoRa E32 driver");

  // Deinitialize interface
  handle->interface.deinit(handle->interface.user_ctx);

  // Free resources
  handle->is_initialized = false;
  free(handle);

  ESP_LOGI(TAG, "LoRa E32 driver deinitialized");
  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_set_mode(lora_e32_comm_handle_t handle,
                                              e32_mode_t mode) {
  if (handle == NULL || !handle->is_initialized) {
    return LORA_E32_COMM_ERR_NOT_INITIALIZED;
  }

  ESP_LOGI(TAG, "Setting mode: %d", mode);

  // Wait for AUX to go high before mode switch
  if (handle->config.gpio_config.aux_pin >= 0) {
    lora_e32_comm_status_t status = lora_e32_comm_wait_aux_high(handle, 1000);
    if (status != LORA_E32_COMM_OK) {
      ESP_LOGW(TAG, "AUX not high before mode switch");
    }
  }

  // Set mode pins
  set_gpio_mode_pins(handle, mode);
  handle->current_mode = mode;

  // Wait for mode switch to complete
  vTaskDelay(pdMS_TO_TICKS(E32_MODE_SWITCH_TIME_MS));

  // Wait for AUX high after mode switch
  if (handle->config.gpio_config.aux_pin >= 0) {
    lora_e32_comm_wait_aux_high(handle, 1000);
  }

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_get_mode(lora_e32_comm_handle_t handle,
                                              e32_mode_t *mode) {
  if (handle == NULL || !handle->is_initialized || mode == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  *mode = handle->current_mode;
  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t
lora_e32_comm_wait_aux_high(lora_e32_comm_handle_t handle,
                            uint32_t timeout_ms) {
  if (handle == NULL || handle->config.gpio_config.aux_pin < 0) {
    return LORA_E32_COMM_OK; // No AUX pin configured
  }

  uint32_t start = xTaskGetTickCount();
  while (!is_aux_high_internal(handle)) {
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
      ESP_LOGW(TAG, "AUX timeout");
      return LORA_E32_COMM_ERR_TIMEOUT;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // Wait additional 2ms after AUX goes high
  vTaskDelay(pdMS_TO_TICKS(E32_AUX_HIGH_TIME_MS));
  return LORA_E32_COMM_OK;
}

bool lora_e32_comm_is_aux_high(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return false;
  }
  return is_aux_high_internal(handle);
}

lora_e32_comm_status_t
lora_e32_comm_send_broadcast(lora_e32_comm_handle_t handle, const uint8_t *data,
                             size_t length) {
  if (handle == NULL || !handle->is_initialized || data == NULL ||
      length == 0) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  if (handle->current_mode != E32_MODE_NORMAL &&
      handle->current_mode != E32_MODE_WAKEUP) {
    ESP_LOGE(TAG, "Invalid mode for transmission: %d", handle->current_mode);
    return LORA_E32_COMM_ERR_MODE_SWITCH;
  }

  ESP_LOGI(TAG, "Sending %d bytes (broadcast)", (int)length);

  // At physical layer, module should be configured:
  //  - Address = 0xFFFF
  //  - Same channel on all nodes
  // so a normal transparent frame becomes broadcast to all modules.

  esp_err_t ret =
      handle->interface.write(handle->interface.user_ctx, data, length, 1000);

  return (ret == ESP_OK) ? LORA_E32_COMM_OK : LORA_E32_COMM_ERR_COMM_FAILED;
}

lora_e32_comm_status_t lora_e32_comm_receive(lora_e32_comm_handle_t handle,
                                             uint8_t *buffer,
                                             size_t buffer_size,
                                             size_t *actual_length,
                                             uint32_t timeout_ms) {
  if (handle == NULL || !handle->is_initialized || buffer == NULL ||
      actual_length == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  esp_err_t ret =
      handle->interface.read(handle->interface.user_ctx, buffer, buffer_size,
                             actual_length, timeout_ms);

  if (ret == ESP_OK && *actual_length > 0) {
    ESP_LOGD(TAG, "Received %d bytes", (int)*actual_length);
    return LORA_E32_COMM_OK;
  }

  return (ret == ESP_ERR_TIMEOUT) ? LORA_E32_COMM_ERR_TIMEOUT
                                  : LORA_E32_COMM_ERR_COMM_FAILED;
}

size_t lora_e32_comm_available(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return 0;
  }

  return handle->interface.available(handle->interface.user_ctx);
}

lora_e32_comm_status_t lora_e32_comm_read_params(lora_e32_comm_handle_t handle,
                                                 e32_params_t *params) {
  if (handle == NULL || !handle->is_initialized || params == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  // Ensure we're in sleep mode
  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }

  // Flush RX buffer
  handle->interface.flush(handle->interface.user_ctx);

  // Send read command: C1 C1 C1
  uint8_t cmd[3] = {E32_CMD_READ_PARAM, E32_CMD_READ_PARAM, E32_CMD_READ_PARAM};
  esp_err_t ret =
      handle->interface.write(handle->interface.user_ctx, cmd, 3, 1000);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send read params command");
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  // Read response: C0 + 5 bytes
  uint8_t response[6];
  size_t actual_len;
  ret = handle->interface.read(handle->interface.user_ctx, response, 6,
                               &actual_len, 1000);

  if (ret != ESP_OK || actual_len != 6 ||
      response[0] != E32_CMD_SET_PARAM_SAVE) {
    ESP_LOGE(TAG, "Failed to read parameters (got %d bytes, first=0x%02X)",
             (int)actual_len, response[0]);
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }

  memcpy(params, response, 6);
  ESP_LOGI(TAG, "Parameters read successfully");
  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_write_params(lora_e32_comm_handle_t handle,
                                                  const e32_params_t *params) {
  if (handle == NULL || !handle->is_initialized || params == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  // Ensure we're in sleep mode
  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }

  // Prepare command with C0 header (save to flash)
  e32_params_t cmd_params;
  memcpy(&cmd_params, params, sizeof(e32_params_t));
  cmd_params.head = E32_CMD_SET_PARAM_SAVE;

  esp_err_t ret = handle->interface.write(handle->interface.user_ctx,
                                          (uint8_t *)&cmd_params,
                                          sizeof(e32_params_t), 1000);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write parameters");
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for module to save

  ESP_LOGI(TAG, "Parameters written successfully");
  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t
lora_e32_comm_write_params_temp(lora_e32_comm_handle_t handle,
                                const e32_params_t *params) {
  if (handle == NULL || !handle->is_initialized || params == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }

  e32_params_t cmd_params;
  memcpy(&cmd_params, params, sizeof(e32_params_t));
  cmd_params.head = E32_CMD_SET_PARAM_TEMP;

  esp_err_t ret = handle->interface.write(handle->interface.user_ctx,
                                          (uint8_t *)&cmd_params,
                                          sizeof(e32_params_t), 1000);

  return (ret == ESP_OK) ? LORA_E32_COMM_OK : LORA_E32_COMM_ERR_CONFIG_FAILED;
}

lora_e32_comm_status_t lora_e32_comm_read_version(lora_e32_comm_handle_t handle,
                                                  e32_version_t *version) {
  if (handle == NULL || !handle->is_initialized || version == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }

  // Flush RX buffer
  handle->interface.flush(handle->interface.user_ctx);

  uint8_t cmd[3] = {E32_CMD_READ_VERSION, E32_CMD_READ_VERSION,
                    E32_CMD_READ_VERSION};
  esp_err_t ret =
      handle->interface.write(handle->interface.user_ctx, cmd, 3, 1000);
  if (ret != ESP_OK) {
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  uint8_t response[4];
  size_t actual_len;
  ret = handle->interface.read(handle->interface.user_ctx, response, 4,
                               &actual_len, 1000);

  if (ret != ESP_OK || actual_len != 4 || response[0] != E32_CMD_READ_VERSION) {
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }

  version->model = response[1];
  version->version = response[2];
  version->features = response[3];

  ESP_LOGI(TAG, "Version: Model=0x%02X, Ver=0x%02X, Features=0x%02X",
           version->model, version->version, version->features);

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_reset(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return LORA_E32_COMM_ERR_NOT_INITIALIZED;
  }

  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }

  uint8_t cmd[3] = {E32_CMD_RESET, E32_CMD_RESET, E32_CMD_RESET};
  esp_err_t ret =
      handle->interface.write(handle->interface.user_ctx, cmd, 3, 1000);

  if (ret != ESP_OK) {
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  vTaskDelay(pdMS_TO_TICKS(E32_RESET_TIME_MS));
  ESP_LOGI(TAG, "Module reset");

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_flush(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return LORA_E32_COMM_ERR_NOT_INITIALIZED;
  }

  esp_err_t ret = handle->interface.flush(handle->interface.user_ctx);
  return (ret == ESP_OK) ? LORA_E32_COMM_OK : LORA_E32_COMM_ERR_COMM_FAILED;
}
