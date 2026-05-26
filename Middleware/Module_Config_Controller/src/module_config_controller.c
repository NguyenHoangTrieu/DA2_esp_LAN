/**
 * @file module_config_controller.c
 * @brief Module configuration controller implementation
 */

#include "module_config_controller.h"
#include "bench_lane_ingress.h"
#include "esp_log.h"
#include "module_i2c_comm.h"
#include "module_spi_comm.h"
#include "module_uart_comm.h"
#include "module_usb_comm.h"
#include "stack_handler.h"
#include <string.h>

static const char *TAG = "MOD_CTRL";

/* ============================================================================
 * Handle Storage
 * ========================================================================== */

typedef struct {
  module_uart_comm_handle_t uart;
  module_spi_comm_handle_t spi;
  module_i2c_comm_handle_t i2c;
  module_usb_comm_handle_t usb;
  bool uart_initialized;
  bool spi_initialized;
  bool i2c_initialized;
  bool usb_initialized;
} stack_handles_t;

static stack_handles_t g_stack_handles[2]; // Stack 0 and Stack 1

module_uart_comm_handle_t module_config_controller_get_uart_handle(uint8_t stack_id) {
  if (stack_id > 1) return NULL;
  if (!g_stack_handles[stack_id].uart_initialized) return NULL;
  return g_stack_handles[stack_id].uart;
}

/* ============================================================================
 * Public API Implementation
 * ========================================================================== */

esp_err_t module_config_controller_init(void) {
  ESP_LOGI(TAG, "Module config controller initialized");
  memset(g_stack_handles, 0, sizeof(g_stack_handles));
  return ESP_OK;
}

/* ============================================================================
 * Communication Initialization Functions
 * ========================================================================== */

esp_err_t module_config_controller_init_uart(uint8_t stack_id,
                                             const uart_params_t *params) {
  if (stack_id > 1 || !params) {
    ESP_LOGE(TAG, "Invalid arguments: stack_id=%d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (g_stack_handles[stack_id].uart_initialized) {
    ESP_LOGW(TAG, "UART already initialized for stack %d", stack_id);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Initializing UART for stack %d", stack_id);

  module_uart_config_t uart_config = {
      .stack_id = stack_id,
      .baudrate = params->baudrate,
      .parity = params->parity,
      .stop_bits = params->stopbit,
      /* RX buffer sized for 5 Mbps line rate × ~25 ms scheduler jitter.
       * 16 KB = ~26 ms at 625 KB/s. Production handlers use <1 KB so this
       * is conservative for normal use too. */
      .rx_buffer_size = 16384,
      .tx_buffer_size = 512,
  };

  esp_err_t ret =
      module_uart_comm_init(&uart_config, &g_stack_handles[stack_id].uart);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize UART: %s", esp_err_to_name(ret));
    return ret;
  }

  g_stack_handles[stack_id].uart_initialized = true;
  ESP_LOGI(TAG, "UART initialized for stack %d: baudrate=%ld, parity=%d",
           stack_id, params->baudrate, params->parity);

  return ESP_OK;
}

esp_err_t module_config_controller_init_spi(uint8_t stack_id,
                                            const spi_params_t *params) {
  if (stack_id > 1 || !params) {
    ESP_LOGE(TAG, "Invalid arguments: stack_id=%d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (g_stack_handles[stack_id].spi_initialized) {
    ESP_LOGW(TAG, "SPI already initialized for stack %d", stack_id);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Initializing SPI for stack %d", stack_id);

  module_spi_config_t spi_config = {
      .stack_id = stack_id,
      .clock_speed_hz = params->clock_speed,
      .mode = params->mode,
      .queue_size = 1,
  };

  esp_err_t ret =
      module_spi_comm_init(&spi_config, &g_stack_handles[stack_id].spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI: %s", esp_err_to_name(ret));
    return ret;
  }

  g_stack_handles[stack_id].spi_initialized = true;
  ESP_LOGI(TAG, "SPI initialized for stack %d: clock=%ld Hz, mode=%d", stack_id,
           params->clock_speed, params->mode);

  return ESP_OK;
}

esp_err_t module_config_controller_init_i2c(uint8_t stack_id,
                                            const i2c_params_t *params) {
  if (stack_id > 1 || !params) {
    ESP_LOGE(TAG, "Invalid arguments: stack_id=%d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (g_stack_handles[stack_id].i2c_initialized) {
    ESP_LOGW(TAG, "I2C already initialized for stack %d", stack_id);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Initializing I2C for stack %d", stack_id);

  module_i2c_config_t i2c_config = {
      .stack_id = stack_id,
      .device_address = params->address,
      .clock_speed_hz = params->clock_speed,
      .pullup_enable = true,
  };

  esp_err_t ret =
      module_i2c_comm_init(&i2c_config, &g_stack_handles[stack_id].i2c);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
    return ret;
  }

  g_stack_handles[stack_id].i2c_initialized = true;
  ESP_LOGI(TAG, "I2C initialized for stack %d: address=0x%02X, clock=%d",
           stack_id, params->address, params->clock_speed);

  return ESP_OK;
}

esp_err_t module_config_controller_init_usb(uint8_t stack_id,
                                            const usb_params_t *params) {
  if (stack_id > 1 || !params) {
    ESP_LOGE(TAG, "Invalid arguments: stack_id=%d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (g_stack_handles[stack_id].usb_initialized) {
    ESP_LOGW(TAG, "USB already initialized for stack %d", stack_id);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Initializing USB for stack %d", stack_id);

  module_usb_config_t usb_config = {
      .stack_id = stack_id,
      .line_coding =
          {
              .bit_rate = params->bit_rate,
              .stop_bits = params->stop_bits,
              .parity = params->parity,
              .data_bits = params->data_bits,
          },
      .rx_buffer_size = 256,
      .tx_buffer_size = 256,
  };

  esp_err_t ret =
      module_usb_comm_init(&usb_config, &g_stack_handles[stack_id].usb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize USB: %s", esp_err_to_name(ret));
    return ret;
  }

  g_stack_handles[stack_id].usb_initialized = true;
  ESP_LOGI(TAG,
           "USB initialized for stack %d: bitrate=%lu, stop_bits=%d, parity=%d",
           stack_id, params->bit_rate, params->stop_bits, params->parity);

  return ESP_OK;
}

/* ============================================================================
 * Communication Deinitialization Functions
 * ========================================================================== */

esp_err_t module_config_controller_deinit_uart(uint8_t stack_id) {
  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!g_stack_handles[stack_id].uart_initialized) {
    ESP_LOGW(TAG, "UART not initialized for stack %d", stack_id);
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Deinitializing UART for stack %d", stack_id);

  esp_err_t ret = module_uart_comm_deinit(g_stack_handles[stack_id].uart);
  if (ret == ESP_OK) {
    g_stack_handles[stack_id].uart_initialized = false;
    g_stack_handles[stack_id].uart = NULL;
  }

  return ret;
}

esp_err_t module_config_controller_deinit_spi(uint8_t stack_id) {
  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!g_stack_handles[stack_id].spi_initialized) {
    ESP_LOGW(TAG, "SPI not initialized for stack %d", stack_id);
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Deinitializing SPI for stack %d", stack_id);

  esp_err_t ret = module_spi_comm_deinit(g_stack_handles[stack_id].spi);
  if (ret == ESP_OK) {
    g_stack_handles[stack_id].spi_initialized = false;
    g_stack_handles[stack_id].spi = NULL;
  }

  return ret;
}

esp_err_t module_config_controller_deinit_i2c(uint8_t stack_id) {
  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!g_stack_handles[stack_id].i2c_initialized) {
    ESP_LOGW(TAG, "I2C not initialized for stack %d", stack_id);
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Deinitializing I2C for stack %d", stack_id);

  esp_err_t ret = module_i2c_comm_deinit(g_stack_handles[stack_id].i2c);
  if (ret == ESP_OK) {
    g_stack_handles[stack_id].i2c_initialized = false;
    g_stack_handles[stack_id].i2c = NULL;
  }

  return ret;
}

esp_err_t module_config_controller_deinit_usb(uint8_t stack_id) {
  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!g_stack_handles[stack_id].usb_initialized) {
    ESP_LOGW(TAG, "USB not initialized for stack %d", stack_id);
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Deinitializing USB for stack %d", stack_id);

  esp_err_t ret = module_usb_comm_deinit(g_stack_handles[stack_id].usb);
  if (ret == ESP_OK) {
    g_stack_handles[stack_id].usb_initialized = false;
    g_stack_handles[stack_id].usb = NULL;
  }

  return ret;
}

/* ============================================================================
 * Bus Communication Functions
 * ========================================================================== */

esp_err_t module_bus_write(uint8_t stack_id, comm_port_type_t port_type,
                           const uint8_t *data, size_t len) {
  if (data == NULL || len == 0) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD(TAG, "Bus write: stack=%d, port=%d, len=%d", stack_id, port_type,
           len);

  switch (port_type) {
  case COMM_PORT_UART:
    if (!g_stack_handles[stack_id].uart_initialized) {
      ESP_LOGE(TAG, "UART not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    return module_uart_comm_send(g_stack_handles[stack_id].uart, data, len,
                                 1000);

  case COMM_PORT_SPI:
    if (!g_stack_handles[stack_id].spi_initialized) {
      ESP_LOGE(TAG, "SPI not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    return module_spi_comm_transfer(g_stack_handles[stack_id].spi, data, NULL,
                                    len);

  case COMM_PORT_I2C:
    if (!g_stack_handles[stack_id].i2c_initialized) {
      ESP_LOGE(TAG, "I2C not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    return module_i2c_comm_write(g_stack_handles[stack_id].i2c, data, len,
                                 1000);
    break;

  case COMM_PORT_USB:
    if (!g_stack_handles[stack_id].usb_initialized) {
      ESP_LOGE(TAG, "USB not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    return module_usb_comm_send(g_stack_handles[stack_id].usb, data, len, 1000);

  default:
    ESP_LOGE(TAG, "Unsupported port type: %d", port_type);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t module_bus_read(uint8_t stack_id, comm_port_type_t port_type,
                          uint8_t *buffer, size_t max_len, uint32_t timeout_ms,
                          size_t *received_len) {
  if (buffer == NULL || received_len == NULL) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

// Validate timeout to prevent hangs or zero-wait bugs
#define MODULE_CTRL_MIN_TIMEOUT_MS 10
#define MODULE_CTRL_MAX_TIMEOUT_MS 60000

  if (timeout_ms < MODULE_CTRL_MIN_TIMEOUT_MS) {
    ESP_LOGW(TAG, "Timeout too small (%ld ms), clamping to minimum (%d ms)",
             timeout_ms, MODULE_CTRL_MIN_TIMEOUT_MS);
    timeout_ms = MODULE_CTRL_MIN_TIMEOUT_MS;
  }

  if (timeout_ms > MODULE_CTRL_MAX_TIMEOUT_MS) {
    ESP_LOGW(TAG, "Timeout too large (%ld ms), clamping to maximum (%d ms)",
             timeout_ms, MODULE_CTRL_MAX_TIMEOUT_MS);
    timeout_ms = MODULE_CTRL_MAX_TIMEOUT_MS;
  }

  *received_len = 0;

  ESP_LOGD(TAG, "Bus read: stack=%d, port=%d, timeout=%dms", stack_id,
           port_type, timeout_ms);

  switch (port_type) {
  case COMM_PORT_UART: {
    if (!g_stack_handles[stack_id].uart_initialized) {
      ESP_LOGE(TAG, "UART not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    esp_err_t r = module_uart_comm_receive(g_stack_handles[stack_id].uart, buffer,
                                           max_len, received_len, timeout_ms);
    if (r == ESP_OK && *received_len > 0)
      bench_lane_count_rx(stack_id, BENCH_LANE_UART, (uint32_t)*received_len);
    else
      bench_lane_count_miss(stack_id, BENCH_LANE_UART);
    return r;
  }

  case COMM_PORT_SPI: {
    if (!g_stack_handles[stack_id].spi_initialized) {
      ESP_LOGE(TAG, "SPI not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    *received_len = max_len;
    esp_err_t r = module_spi_comm_transfer(g_stack_handles[stack_id].spi, NULL, buffer,
                                           max_len);
    if (r == ESP_OK && *received_len > 0)
      bench_lane_count_rx(stack_id, BENCH_LANE_SPI, (uint32_t)*received_len);
    else
      bench_lane_count_miss(stack_id, BENCH_LANE_SPI);
    return r;
  }

  case COMM_PORT_I2C: {
    if (!g_stack_handles[stack_id].i2c_initialized) {
      ESP_LOGE(TAG, "I2C not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = module_i2c_comm_read(g_stack_handles[stack_id].i2c, buffer,
                                         max_len, timeout_ms);
    if (ret == ESP_OK) {
      *received_len = max_len;
      bench_lane_count_rx(stack_id, BENCH_LANE_I2C, (uint32_t)*received_len);
    } else {
      *received_len = 0;
      bench_lane_count_miss(stack_id, BENCH_LANE_I2C);
    }
    return ret;
  }

  case COMM_PORT_USB: {
    if (!g_stack_handles[stack_id].usb_initialized) {
      ESP_LOGE(TAG, "USB not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    esp_err_t r = module_usb_comm_receive(g_stack_handles[stack_id].usb, buffer,
                                          max_len, received_len, timeout_ms);
    if (r == ESP_OK && *received_len > 0)
      bench_lane_count_rx(stack_id, BENCH_LANE_USB, (uint32_t)*received_len);
    else
      bench_lane_count_miss(stack_id, BENCH_LANE_USB);
    return r;
  }

  default:
    ESP_LOGE(TAG, "Unsupported port type: %d", port_type);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t module_bus_flush(uint8_t stack_id, comm_port_type_t port_type) {
  if (stack_id > 1) {
    return ESP_ERR_INVALID_ARG;
  }

  switch (port_type) {
  case COMM_PORT_UART:
    if (!g_stack_handles[stack_id].uart_initialized) {
      ESP_LOGE(TAG, "UART not initialized for stack %d", stack_id);
      return ESP_ERR_INVALID_STATE;
    }
    return module_uart_comm_flush(g_stack_handles[stack_id].uart);

  default:
    /* Non-UART buses have no buffered-input concept; treat as no-op */
    return ESP_OK;
  }
}

size_t module_bus_drain(uint8_t stack_id, comm_port_type_t port_type,
                        uint8_t *buf, size_t max, uint32_t window_ms) {
  if (!buf || max == 0 || stack_id > 1) return 0;

  size_t     total  = 0;
  TickType_t t0     = xTaskGetTickCount();
  /* Use 20 ms slices so we don't miss back-to-back bytes */
  const uint32_t SLICE_MS = 20;

  while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(window_ms) &&
         total < max - 1) {
    size_t got = 0;
    module_bus_read(stack_id, port_type, buf + total, max - 1 - total,
                    SLICE_MS, &got);
    total += got;
  }
  buf[total] = '\0';
  return total;
}

esp_err_t module_gpio_write(uint8_t stack_id, const char *pin, bool state) {
  if (pin == NULL || strlen(pin) < 2) {
    ESP_LOGE(TAG, "Invalid pin format");
    return ESP_ERR_INVALID_ARG;
  }

  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  // Parse pin "XY": X = stack port (0=Stack1, 1=Stack2)
  //                  Y = GPIO pin number 00-17 (flat TCA6416A mapping: P00-P17)
  // New architecture: All 16 TCA pins directly accessible via numeric pin ID
  // No predefined meaning for pins (WAKE#/PERST#) — configured per use case
  uint8_t port = pin[0] - '0';

  if (port != stack_id) {
    ESP_LOGE(TAG, "Pin port %d does not match stack_id %d", port, stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  // Parse numeric pin ID 00-17 (supports both single digit 0-9 and two digits 10-17)
  stack_gpio_pin_num_t gpio_pin;
  int pin_num = -1;
  
  if (strlen(pin) == 2) {
    // Single digit: 0-9
    pin_num = pin[1] - '0';
  } else if (strlen(pin) == 3 && pin[1] >= '0' && pin[1] <= '9' && pin[2] >= '0' && pin[2] <= '9') {
    // Two digits: 00-17
    pin_num = (pin[1] - '0') * 10 + (pin[2] - '0');
  }
  
  if (pin_num < 0 || pin_num > 17 || pin_num == 8 || pin_num == 9) {
    ESP_LOGE(TAG, "Invalid GPIO pin: %s (must be 00-07 or 10-17)", pin);
    return ESP_ERR_INVALID_ARG;
  }

  // P10-P17 map to enum values 8-15 (STACK_GPIO_PIN_10=8 ... STACK_GPIO_PIN_17=15)
  if (pin_num >= 10) {
    pin_num -= 2;
  }

  gpio_pin = (stack_gpio_pin_num_t)pin_num;
  ESP_LOGD(TAG, "Parsed pin %s as GPIO pin %d (enum %d)", pin, pin_num, gpio_pin);

  ESP_LOGI(TAG, "GPIO write: stack=%d, pin=%s, state=%d", stack_id, pin, state);

  gpio_action_t action = {.pin = gpio_pin, .level = state};

  // Call stack handler
  return stack_handler_gpio_write_multi(stack_id, &action, 1);
}

esp_err_t module_gpio_write_multi(uint8_t stack_id,
                                  const gpio_control_t *gpio_actions,
                                  size_t count) {
  if (gpio_actions == NULL || count == 0) {
    return ESP_OK; // Nothing to do
  }

  if (stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (count > MAX_GPIO_ACTIONS) {
    ESP_LOGE(TAG, "Too many GPIO actions: %d (max %d)", count,
             MAX_GPIO_ACTIONS);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "GPIO write multi: stack=%d, count=%d", stack_id, count);

  // Convert to gpio_action_t array
  gpio_action_t actions[MAX_GPIO_ACTIONS];

  for (size_t i = 0; i < count; i++) {
    if (strlen(gpio_actions[i].pin) < 2) {
      ESP_LOGE(TAG, "Invalid pin format at index %d", i);
      return ESP_ERR_INVALID_ARG;
    }

    // Parse "XY": X=port (0=Stack1, 1=Stack2), Y=pin 00-17 (flat GPIO mapping)
    // New architecture: All 16 TCA pins accessible, no predefined W/P shortcuts
    uint8_t port = gpio_actions[i].pin[0] - '0';

    if (port != stack_id) {
      ESP_LOGE(TAG, "Pin port %d does not match stack_id %d at index %d", port,
               stack_id, i);
      return ESP_ERR_INVALID_ARG;
    }

    // Parse numeric pin ID 00-17
    int pin_num = -1;
    const char *pin_str = gpio_actions[i].pin;
    size_t pin_len = strlen(pin_str);
    
    if (pin_len == 2) {
      // Single digit 0-9
      pin_num = pin_str[1] - '0';
    } else if (pin_len == 3 && pin_str[1] >= '0' && pin_str[1] <= '9' && pin_str[2] >= '0' && pin_str[2] <= '9') {
      // Two digits 00-17
      pin_num = (pin_str[1] - '0') * 10 + (pin_str[2] - '0');
    }
    
    if (pin_num < 0 || pin_num > 17 || pin_num == 8 || pin_num == 9) {
      ESP_LOGE(TAG, "Invalid GPIO pin format at index %d: expected 00-07 or 10-17, got %s", i, pin_str);
      return ESP_ERR_INVALID_ARG;
    }

    // P10-P17 map to enum values 8-15 (STACK_GPIO_PIN_10=8 ... STACK_GPIO_PIN_17=15)
    if (pin_num >= 10) {
      pin_num -= 2;
    }

    actions[i].pin = (stack_gpio_pin_num_t)pin_num;
    actions[i].level = gpio_actions[i].state;
  }

  // Call stack handler
  return stack_handler_gpio_write_multi(stack_id, actions, count);
}
