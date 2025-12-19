/**
 * @file lora_e32_comm.c
 * @brief E32 LoRa Module Communication Driver Implementation (Broadcast API)
 *
 * This driver provides a thin abstraction over the E32 module using a
 * pluggable communication interface (currently UART only). All data is
 * transmitted in transparent broadcast mode: the radio must be configured
 * with address 0xFFFF and the same RF channel on all nodes.
 */

#include "lora_e32_comm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stack_handler.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "LORA_E32_COMM";

/* ===== Default E32 parameter bytes (compile‑time constants) =====
 *
 * We cannot call inline helper functions (e32_build_sped / e32_build_option)
 * inside a global initializer because they are not constant expressions.
 *
 * Instead we re‑encode the logic here using only macros and bit‑operations,
 * which *are* allowed in constant initializers.
 */
enum {
  E32_DEFAULT_SPED_BYTE = ((E32_DEFAULT_UART_PARITY << E32_SPED_PARITY_SHIFT) &
                           E32_SPED_PARITY_MASK) |
                          ((E32_DEFAULT_UART_BAUD << E32_SPED_UART_BAUD_SHIFT) &
                           E32_SPED_UART_BAUD_MASK) |
                          ((E32_DEFAULT_AIR_RATE << E32_SPED_AIR_RATE_SHIFT) &
                           E32_SPED_AIR_RATE_MASK),

  E32_DEFAULT_OPTION_BYTE =
      ((E32_DEFAULT_TRANS_MODE << E32_OPTION_TRANS_MODE_SHIFT) &
       E32_OPTION_TRANS_MODE_MASK) |
      ((E32_DEFAULT_IO_DRIVE << E32_OPTION_IO_DRIVE_SHIFT) &
       E32_OPTION_IO_DRIVE_MASK) |
      ((E32_DEFAULT_WAKEUP_TIME << E32_OPTION_WAKEUP_SHIFT) &
       E32_OPTION_WAKEUP_MASK) |
      ((E32_DEFAULT_FEC << E32_OPTION_FEC_SHIFT) & E32_OPTION_FEC_MASK) |
      ((E32_DEFAULT_TX_POWER << E32_OPTION_POWER_SHIFT) &
       E32_OPTION_POWER_MASK),
};

/* Global E32 configuration (UART + module params).
 * The baud rate is used when configuring the UART interface.
 * The params structure is the desired radio configuration that
 * will be written to the module at initialization.
 *
 * Default configuration is:
 *  - Broadcast address 0xFFFF
 *  - 433 MHz channel (E32_DEFAULT_CHANNEL)
 *  - Transparent transmission mode
 *  - 9600 bps UART, 8N1
 *  - 2.4 kbps air data rate
 *  - FEC enabled, 30 dBm TX power
 */
e32_params_t g_lora_e32_params = {
    .head = E32_CMD_SET_PARAM_SAVE,
    .addh = E32_GET_ADDH(E32_ADDR_BROADCAST),
    .addl = E32_GET_ADDL(E32_ADDR_BROADCAST),
    .sped = E32_DEFAULT_SPED_BYTE,
    .chan = E32_DEFAULT_CHANNEL,
    .option = E32_DEFAULT_OPTION_BYTE,
};

int g_lora_e32_baud_rate = LORA_E32_DEFAULT_BAUD_RATE; // Host UART baud rate

// ===== Internal Handle Structure =====
struct lora_e32_comm_handle_s {
  lora_e32_comm_config_t config;
  lora_e32_comm_interface_t interface;
  e32_mode_t current_mode;
  bool is_initialized;
  uint8_t stack_id;
};
//==Pre declarations==
static lora_e32_comm_status_t
lora_e32_comm_wait_aux_high(lora_e32_comm_handle_t handle, uint32_t timeout_ms);

// ===== Stack‑based Pin Mapping Helpers =====
/**
 * @brief Get current active stack ID based on global stack types
 * @return uint8_t Stack ID (0 or 1), default to 0 if no LoRa stack found
 */
static uint8_t get_active_lora_stack(void) {
  if (g_stack_1_type == STACK_COMM_TYPE_LORA) {
    return 0; // Stack 1
  } else if (g_stack_2_type == STACK_COMM_TYPE_LORA) {
    return 1; // Stack 2
  }
  return 0; // Default to Stack 1 if not initialized
}

/**
 * @brief Get UART port for current stack
 */
static int get_lora_uart_port(void) {
  return (get_active_lora_stack() == 0) ? LORA_E32_UART_PORT_STACK_1
                                        : LORA_E32_UART_PORT_STACK_2;
}

/**
 * @brief Get UART TX pin for current stack
 */
static int get_lora_uart_tx_pin(void) {
  return (get_active_lora_stack() == 0) ? LORA_E32_UART_TX_PIN_STACK_1
                                        : LORA_E32_UART_TX_PIN_STACK_2;
}

/**
 * @brief Get UART RX pin for current stack
 */
static int get_lora_uart_rx_pin(void) {
  return (get_active_lora_stack() == 0) ? LORA_E32_UART_RX_PIN_STACK_1
                                        : LORA_E32_UART_RX_PIN_STACK_2;
}

/**
 * @brief Get M0 GPIO pin for current stack
 */
static stack_gpio_pin_num_t get_lora_m0_gpio(void) {
  return (get_active_lora_stack() == 0) ? LORA_E32_M0_GPIO_STACK_1
                                        : LORA_E32_M0_GPIO_STACK_2;
}

/**
 * @brief Get M1 GPIO pin for current stack
 */
static stack_gpio_pin_num_t get_lora_m1_gpio(void) {
  return (get_active_lora_stack() == 0) ? LORA_E32_M1_GPIO_STACK_1
                                        : LORA_E32_M1_GPIO_STACK_2;
}

/**
 * @brief Get AUX GPIO pin for current stack
 */
static stack_gpio_pin_num_t get_lora_aux_gpio(void) {
  return (get_active_lora_stack() == 0) ? LORA_E32_AUX_GPIO_STACK_1
                                        : LORA_E32_AUX_GPIO_STACK_2;
}

// ===== UART Interface Implementation =====

static esp_err_t uart_init_impl(void *config_ptr, void **user_ctx) {
  if (config_ptr == NULL || user_ctx == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  lora_e32_comm_uart_config_t *uart_cfg =
      (lora_e32_comm_uart_config_t *)config_ptr;

  // Get UART config based on active stack
  int uart_port = get_lora_uart_port();
  int tx_pin = get_lora_uart_tx_pin();
  int rx_pin = get_lora_uart_rx_pin();
  int baud =
      (uart_cfg->baud_rate > 0) ? uart_cfg->baud_rate : g_lora_e32_baud_rate;

  uart_config_t uart_config = {.baud_rate = baud,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .source_clk = UART_SCLK_DEFAULT};

  esp_err_t ret = uart_param_config(uart_port, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_driver_install(uart_port, uart_cfg->rx_buffer_size,
                            uart_cfg->tx_buffer_size, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  *user_ctx = (void *)(intptr_t)uart_port;
  ESP_LOGI(TAG, "UART initialized: Stack%d, port=%d, baud=%d, TX=%d, RX=%d",
           get_active_lora_stack() + 1, uart_port, baud, tx_pin, rx_pin);
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
  lora_e32_comm_interface_t interface = {
      .user_ctx = (void *)(intptr_t)get_lora_uart_port(),
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
  (void)handle;

  uint8_t stack_id = get_active_lora_stack();
  uint8_t m0 = (mode & 0x01) ? 1 : 0;
  uint8_t m1 = (mode & 0x02) ? 1 : 0;

  esp_err_t ret = stack_handler_gpio_write(stack_id, get_lora_m0_gpio(), m0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set M0");
    return ret;
  }

  ret = stack_handler_gpio_write(stack_id, get_lora_m1_gpio(), m1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set M1");
    return ret;
  }

  ESP_LOGD(TAG, "Set mode pins: M0=%d, M1=%d (Stack %d)", m0, m1, stack_id + 1);
  return ESP_OK;
}

static bool is_aux_high_internal(lora_e32_comm_handle_t handle) {
  (void)handle;

  uint8_t stack_id = get_active_lora_stack();
  bool level = false;

  esp_err_t ret =
      stack_handler_gpio_read(stack_id, get_lora_aux_gpio(), &level);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read AUX");
    return true;
  }

  return level;
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

  // Copy configuration from application
  memcpy(&h->config, config, sizeof(lora_e32_comm_config_t));
  h->interface = config->interface;
  h->current_mode = E32_MODE_SLEEP;
  h->is_initialized = false;
  h->stack_id = get_active_lora_stack();
  esp_err_t gpio_ret;
  gpio_ret =
      stack_handler_gpio_set_direction(h->stack_id, get_lora_m0_gpio(), true);
  if (gpio_ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure M0");
    free(h);
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  gpio_ret =
      stack_handler_gpio_set_direction(h->stack_id, get_lora_m1_gpio(), true);
  if (gpio_ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure M1");
    free(h);
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  gpio_ret =
      stack_handler_gpio_set_direction(h->stack_id, get_lora_aux_gpio(), false);
  if (gpio_ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure AUX");
    free(h);
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  ESP_LOGI(TAG,
           "GPIO configured for Stack %d (M0=GPIO%d, M1=GPIO%d, AUX=GPIO%d)",
           h->stack_id + 1, get_lora_m0_gpio() + 1, get_lora_m1_gpio() + 1,
           get_lora_aux_gpio() + 1);

  // Initialize communication interface (UART or others)
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

  // Overwrite module_params with the global default configuration so that
  // the radio is always configured to the desired broadcast settings at
  // startup.
  h->config.module_params = g_lora_e32_params;

  // Write the initial parameters to the module (save to flash).
  lora_e32_comm_status_t st =
      lora_e32_comm_write_params(h, &h->config.module_params);
  if (st != LORA_E32_COMM_OK) {
    ESP_LOGW(TAG, "Failed to apply initial E32 parameters (status=%d)", st);
  } else {
    ESP_LOGI(TAG, "Initial E32 parameters applied to module");
  }

  h->is_initialized = true;
  h->stack_id = get_active_lora_stack();
  *handle = h;
  vTaskDelay(pdMS_TO_TICKS(50)); // Wait before flushing
  h->interface.flush(h->interface.user_ctx);
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
  lora_e32_comm_status_t status = lora_e32_comm_wait_aux_high(handle, 1000);
  if (status != LORA_E32_COMM_OK) {
    ESP_LOGW(TAG, "AUX not high before mode switch");
  }

  // Set mode pins
  set_gpio_mode_pins(handle, mode);
  handle->current_mode = mode;

  // Wait for mode switch to complete
  vTaskDelay(pdMS_TO_TICKS(E32_MODE_SWITCH_TIME_MS));

  // Wait for AUX high after mode switch
  lora_e32_comm_wait_aux_high(handle, 1000);

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

static lora_e32_comm_status_t
lora_e32_comm_wait_aux_high(lora_e32_comm_handle_t handle,
                            uint32_t timeout_ms) {
  if (handle == NULL) {
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

/**
 * @brief Temporarily change UART baudrate
 */
static esp_err_t uart_change_baudrate(lora_e32_comm_handle_t handle,
                                      int new_baud) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;

  int uart_port = (int)(intptr_t)handle->interface.user_ctx;

  uart_wait_tx_done(uart_port, pdMS_TO_TICKS(100));
  esp_err_t ret = uart_set_baudrate(uart_port, new_baud);
  if (ret != ESP_OK)
    return ret;

  uart_flush_input(uart_port);
  vTaskDelay(pdMS_TO_TICKS(10));

  // sync global
  g_lora_e32_baud_rate = new_baud;

  uint32_t b = 0;
  uart_get_baudrate(uart_port, &b);
  ESP_LOGI(TAG, "UART baudrate changed to %lu", (unsigned long)b);
  return ESP_OK;
}

lora_e32_comm_status_t lora_e32_comm_read_params(lora_e32_comm_handle_t handle,
                                                 e32_params_t *params) {
  if (handle == NULL || params == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  // Ensure we're in sleep mode
  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Wait before write

  int uart_port = (int)(intptr_t)handle->interface.user_ctx;
  uint32_t cur_baud_u32 = 0;
  uart_get_baudrate(uart_port, &cur_baud_u32);
  int original_baud = (int)cur_baud_u32;

  // Config mode requires 9600 bps
  if (original_baud != 9600) {
    if (uart_change_baudrate(handle, 9600) != ESP_OK) {
      return LORA_E32_COMM_ERR_COMM_FAILED;
    }
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
  size_t actual_len = 0;
  ret = handle->interface.read(handle->interface.user_ctx, response, 6,
                               &actual_len, 1000);

  if (ret != ESP_OK || actual_len != 6 ||
      response[0] != E32_CMD_SET_PARAM_SAVE) {
    ESP_LOGE(TAG, "Failed to read parameters (got %d bytes, first=0x%02X)",
             (int)actual_len, response[0]);
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }
  if (original_baud != 9600) {
    uart_change_baudrate(handle, original_baud);
  }
  lora_e32_comm_set_mode(handle, E32_MODE_NORMAL);
  memcpy(params, response, 6);
  ESP_LOGI(TAG, "Parameters read successfully");

  // Keep internal state in sync with the module
  memcpy(&handle->config.module_params, params, sizeof(e32_params_t));
  g_lora_e32_params = *params;

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_write_params(lora_e32_comm_handle_t handle,
                                                  const e32_params_t *params) {
  if (!handle || !params) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Wait before write

  // Get actual baudrate from driver
  int uart_port = (int)(intptr_t)handle->interface.user_ctx;
  uint32_t cur_baud_u32 = 0;
  uart_get_baudrate(uart_port, &cur_baud_u32);
  int original_baud = (int)cur_baud_u32;

  // Config mode requires 9600 bps
  if (original_baud != 9600) {
    if (uart_change_baudrate(handle, 9600) != ESP_OK) {
      return LORA_E32_COMM_ERR_COMM_FAILED;
    }
  }

  handle->interface.flush(handle->interface.user_ctx);

  // Send write command
  uint8_t cmd[6] = {E32_CMD_SET_PARAM_SAVE, params->addh, params->addl,
                    params->sped,           params->chan, params->option};

  if (handle->interface.write(handle->interface.user_ctx, cmd, sizeof(cmd),
                              1000) != ESP_OK) {
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }
  // Verify: read back actual params from module
  e32_params_t rb = {0};
  lora_e32_comm_status_t st = lora_e32_comm_read_params(handle, &rb);

  if (original_baud != 9600)
    uart_change_baudrate(handle, original_baud);
  lora_e32_comm_set_mode(handle, E32_MODE_NORMAL);
  if (st != LORA_E32_COMM_OK)
    return st;

  // Compare contents (ignore header)
  if (rb.addh != params->addh || rb.addl != params->addl ||
      rb.sped != params->sped || rb.chan != params->chan ||
      rb.option != params->option) {
    ESP_LOGE(TAG,
             "Temp write verify mismatch: "
             "W[ADDH=%02X ADDL=%02X SPED=%02X CHAN=%02X OPT=%02X] "
             "R[ADDH=%02X ADDL=%02X SPED=%02X CHAN=%02X OPT=%02X]",
             params->addh, params->addl, params->sped, params->chan,
             params->option, rb.addh, rb.addl, rb.sped, rb.chan, rb.option);
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }

  if (original_baud != 9600)
    uart_change_baudrate(handle, original_baud);
  lora_e32_comm_set_mode(handle, E32_MODE_NORMAL);
  // Update internal based on verified params
  handle->config.module_params = rb;
  g_lora_e32_params = rb;

  ESP_LOGI(TAG, "Parameters written & verified OK");
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
  vTaskDelay(pdMS_TO_TICKS(50)); // Wait before write

  int uart_port = (int)(intptr_t)handle->interface.user_ctx;
  uint32_t cur_baud_u32 = 0;
  uart_get_baudrate(uart_port, &cur_baud_u32);
  int original_baud = (int)cur_baud_u32;

  // Config mode requires 9600 bps
  if (original_baud != 9600) {
    if (uart_change_baudrate(handle, 9600) != ESP_OK) {
      return LORA_E32_COMM_ERR_COMM_FAILED;
    }
  }

  // Flush RX buffer
  handle->interface.flush(handle->interface.user_ctx);

  // Send temp write command
  uint8_t frame[6] = {E32_CMD_SET_PARAM_TEMP, params->addh, params->addl,
                      params->sped,           params->chan, params->option};

  esp_err_t ret = handle->interface.write(handle->interface.user_ctx, frame,
                                          sizeof(frame), 1000);
  if (ret != ESP_OK) {
    if (original_baud != 9600)
      uart_change_baudrate(handle, original_baud);
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }

  // Verify: read back actual params from module
  e32_params_t rb = {0};
  lora_e32_comm_status_t st = lora_e32_comm_read_params(handle, &rb);

  if (original_baud != 9600)
    uart_change_baudrate(handle, original_baud);
  lora_e32_comm_set_mode(handle, E32_MODE_NORMAL);
  if (st != LORA_E32_COMM_OK)
    return st;

  // Compare contents (ignore header)
  if (rb.addh != params->addh || rb.addl != params->addl ||
      rb.sped != params->sped || rb.chan != params->chan ||
      rb.option != params->option) {
    ESP_LOGE(TAG,
             "Temp write verify mismatch: "
             "W[ADDH=%02X ADDL=%02X SPED=%02X CHAN=%02X OPT=%02X] "
             "R[ADDH=%02X ADDL=%02X SPED=%02X CHAN=%02X OPT=%02X]",
             params->addh, params->addl, params->sped, params->chan,
             params->option, rb.addh, rb.addl, rb.sped, rb.chan, rb.option);
    return LORA_E32_COMM_ERR_CONFIG_FAILED;
  }

  // update internal state based on "verified" params
  handle->config.module_params = rb;
  g_lora_e32_params = rb;

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_read_version(lora_e32_comm_handle_t handle,
                                                  e32_version_t *version) {
  if (handle == NULL || !handle->is_initialized || version == NULL) {
    return LORA_E32_COMM_ERR_INVALID_ARG;
  }

  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Wait before write
  // Switch to 9600 for config mode
  int uart_port = (int)(intptr_t)handle->interface.user_ctx;
  uint32_t cur_baud_u32 = 0;
  uart_get_baudrate(uart_port, &cur_baud_u32);
  int original_baud = (int)cur_baud_u32;

  // Config mode requires 9600 bps
  if (original_baud != 9600) {
    if (uart_change_baudrate(handle, 9600) != ESP_OK) {
      return LORA_E32_COMM_ERR_COMM_FAILED;
    }
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
  size_t actual_len = 0;
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
  if (original_baud != 9600) {
    uart_change_baudrate(handle, original_baud);
  }
  lora_e32_comm_set_mode(handle, E32_MODE_NORMAL);

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_reset(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return LORA_E32_COMM_ERR_NOT_INITIALIZED;
  }

  if (handle->current_mode != E32_MODE_SLEEP) {
    lora_e32_comm_set_mode(handle, E32_MODE_SLEEP);
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Wait before write

  int uart_port = (int)(intptr_t)handle->interface.user_ctx;
  uint32_t cur_baud_u32 = 0;
  uart_get_baudrate(uart_port, &cur_baud_u32);
  int original_baud = (int)cur_baud_u32;

  // Config mode requires 9600 bps
  if (original_baud != 9600) {
    if (uart_change_baudrate(handle, 9600) != ESP_OK) {
      return LORA_E32_COMM_ERR_COMM_FAILED;
    }
  }

  uint8_t cmd[3] = {E32_CMD_RESET, E32_CMD_RESET, E32_CMD_RESET};
  esp_err_t ret =
      handle->interface.write(handle->interface.user_ctx, cmd, 3, 1000);

  if (ret != ESP_OK) {
    return LORA_E32_COMM_ERR_COMM_FAILED;
  }

  vTaskDelay(pdMS_TO_TICKS(E32_RESET_TIME_MS));
  ESP_LOGI(TAG, "Module reset");

  if (original_baud != 9600) {
    uart_change_baudrate(handle, original_baud);
  }
  lora_e32_comm_set_mode(handle, E32_MODE_NORMAL);

  return LORA_E32_COMM_OK;
}

lora_e32_comm_status_t lora_e32_comm_flush(lora_e32_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return LORA_E32_COMM_ERR_NOT_INITIALIZED;
  }

  esp_err_t ret = handle->interface.flush(handle->interface.user_ctx);
  return (ret == ESP_OK) ? LORA_E32_COMM_OK : LORA_E32_COMM_ERR_COMM_FAILED;
}

lora_e32_comm_handle_t g_lora_e32_handle = NULL;

static lora_e32_comm_uart_config_t g_default_uart_cfg = {
    .baud_rate = 9600, .rx_buffer_size = 512, .tx_buffer_size = 512};

static lora_e32_comm_config_t g_default_comm_cfg;

esp_err_t lora_e32_auto_init_default(void) {
  static bool s_inited = false;
  if (s_inited) {
    return ESP_OK;
  }

  g_default_comm_cfg.comm_type = LORA_E32_COMM_TYPE_UART;
  g_default_comm_cfg.interface = lora_e32_comm_create_uart_interface();
  g_default_comm_cfg.interface_config = &g_default_uart_cfg;
  g_default_comm_cfg.module_params = g_lora_e32_params;

  lora_e32_comm_status_t st =
      lora_e32_comm_init(&g_default_comm_cfg, &g_lora_e32_handle);

  if (st != LORA_E32_COMM_OK) {
    ESP_LOGE("LORA_E32", "Auto default E32 init FAILED, status=%d", st);
    g_lora_e32_handle = NULL;
    return ESP_FAIL;
  }

  // Optionally: set mode NORMAL
  lora_e32_comm_set_mode(g_lora_e32_handle, E32_MODE_NORMAL);

  s_inited = true;
  ESP_LOGI("LORA_E32", "Auto default E32 initialized");
  return ESP_OK;
}
