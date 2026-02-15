/**
 * @file stack_handler.c
 * @brief Communication Stack Manager Implementation
 */

#include "stack_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tca_handler.h"
#include <string.h>

static const char *TAG = "STACK_HANDLER";

/* ===== GPIO Pin Mapping Tables ===== */

// Stack 1 GPIO mapping (LAN1): GPIO number -> {TCA_PORT, TCA_PIN}
static const struct {
  tca_port_t port;
  uint8_t pin;
} stack1_gpio_map[STACK_GPIO_PIN_COUNT] = {
    {TCA_PORT_2, 1}, // GPIO 1 -> P21
    {TCA_PORT_2, 2}, // GPIO 2 -> P22
    {TCA_PORT_2, 3}, // GPIO 3 -> P23
    {TCA_PORT_2, 4}, // GPIO 4 -> P24
    {TCA_PORT_2, 5}, // GPIO 5 -> P25
    {TCA_PORT_1, 3}, // GPIO 6 -> P13
    {TCA_PORT_1, 4}, // GPIO 7 -> P14
    {TCA_PORT_1, 5}, // GPIO 8 -> P15
    {TCA_PORT_1, 6}  // GPIO 9 -> P16
};

// Stack 2 GPIO mapping (LAN2): GPIO number -> {TCA_PORT, TCA_PIN}
static const struct {
  tca_port_t port;
  uint8_t pin;
} stack2_gpio_map[STACK_GPIO_PIN_COUNT] = {
    {TCA_PORT_0, 6}, // GPIO 1 -> P06
    {TCA_PORT_0, 7}, // GPIO 2 -> P07
    {TCA_PORT_1, 0}, // GPIO 3 -> P10
    {TCA_PORT_1, 1}, // GPIO 4 -> P11
    {TCA_PORT_1, 2}, // GPIO 5 -> P12
    {TCA_PORT_0, 0}, // GPIO 6 -> P00
    {TCA_PORT_0, 1}, // GPIO 7 -> P01
    {TCA_PORT_0, 2}, // GPIO 8 -> P02
    {TCA_PORT_0, 3}  // GPIO 9 -> P03
};

/* ===== Internal State ===== */

static bool g_initialized = false;
static SemaphoreHandle_t g_stack_mutex[STACK_HANDLER_MAX_STACKS];

/* ===== Helper Functions ===== */

static inline bool is_valid_stack_id(uint8_t stack_id) {
  return (stack_id < STACK_HANDLER_MAX_STACKS);
}

static inline bool is_valid_pin(stack_gpio_pin_num_t pin) {
  return (pin < STACK_GPIO_PIN_COUNT);
}

static void get_tca_mapping(uint8_t stack_id, stack_gpio_pin_num_t pin,
                            tca_port_t *port, uint8_t *pin_num) {
  if (stack_id == 0) {
    *port = stack1_gpio_map[pin].port;
    *pin_num = stack1_gpio_map[pin].pin;
  } else {
    *port = stack2_gpio_map[pin].port;
    *pin_num = stack2_gpio_map[pin].pin;
  }
}

/* ===== API Implementation ===== */

esp_err_t stack_handler_init(void) {
  if (g_initialized) {
    ESP_LOGW(TAG, "Already initialized");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing stack handler");

  if (tca_test_connection() != ESP_OK) {
    ESP_LOGE(TAG, "TCA6424A not available");
    return ESP_ERR_INVALID_STATE;
  }

  // Configure all TCA ports as inputs by default
  esp_err_t ret;

  ret = tca_configure_port(TCA_PORT_0, 0xFF);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure TCA Port 0");
    return ret;
  }

  ret = tca_configure_port(TCA_PORT_1, 0xFF);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure TCA Port 1");
    return ret;
  }

  ret = tca_configure_port(TCA_PORT_2, 0xFF);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure TCA Port 2");
    return ret;
  }

  // Initialize mutexes for each stack
  for (int i = 0; i < STACK_HANDLER_MAX_STACKS; i++) {
    g_stack_mutex[i] = xSemaphoreCreateMutex();
    if (!g_stack_mutex[i]) {
      ESP_LOGE(TAG, "Failed to create mutex for stack %d", i);
      // Clean up previously created mutexes
      for (int j = 0; j < i; j++) {
        vSemaphoreDelete(g_stack_mutex[j]);
      }
      return ESP_ERR_NO_MEM;
    }
  }

  g_initialized = true;

  ESP_LOGI(TAG, "Stack handler initialized");
  ESP_LOGI(TAG, "  Stack 1 GPIO mapping: P02-P07, P10-P12");
  ESP_LOGI(TAG, "  Stack 2 GPIO mapping: P15-P17, P20-P25");

  return ESP_OK;
}

esp_err_t stack_handler_gpio_write(uint8_t stack_id, stack_gpio_pin_num_t pin,
                                   bool level) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id)) {
    ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!is_valid_pin(pin)) {
    ESP_LOGE(TAG, "Invalid pin: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }

  tca_port_t port;
  uint8_t pin_num;
  get_tca_mapping(stack_id, pin, &port, &pin_num);

  esp_err_t ret = tca_set_pin_verified(port, pin_num, level, true);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write Stack%d GPIO%d (P%d%d)", stack_id + 1,
             pin + 1, port, pin_num);
  }

  return ret;
}

esp_err_t stack_handler_gpio_read(uint8_t stack_id, stack_gpio_pin_num_t pin,
                                  bool *level) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id) || !level) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (!is_valid_pin(pin)) {
    ESP_LOGE(TAG, "Invalid pin: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }

  tca_port_t port;
  uint8_t pin_num;
  get_tca_mapping(stack_id, pin, &port, &pin_num);

  esp_err_t ret = tca_read_pin(port, pin_num, level);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read Stack%d GPIO%d (P%d%d)", stack_id + 1,
             pin + 1, port, pin_num);
  }

  return ret;
}

esp_err_t stack_handler_gpio_set_direction(uint8_t stack_id,
                                           stack_gpio_pin_num_t pin,
                                           bool is_output) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id)) {
    ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!is_valid_pin(pin)) {
    ESP_LOGE(TAG, "Invalid pin: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }

  tca_port_t port;
  uint8_t pin_num;
  get_tca_mapping(stack_id, pin, &port, &pin_num);

  // Read current CONFIGURATION register
  uint8_t port_cfg;
  esp_err_t ret = tca_read_config_register(port, &port_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read config register P%d", port);
    return ret;
  }

  ESP_LOGI(TAG, "Before: Stack%d GPIO%d (P%d.%d) config=0x%02X", stack_id + 1,
           pin + 1, port, pin_num, port_cfg);

  // Modify pin direction (TCA6424A: 1=input, 0=output)
  if (is_output) {
    port_cfg &= ~(1 << pin_num); // Clear bit = output
  } else {
    port_cfg |= (1 << pin_num); // Set bit = input
  }

  // Write back configuration
  ret = tca_configure_port(port, port_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write config P%d", port);
    return ret;
  }

  // Verify
  uint8_t verify_cfg;
  ret = tca_read_config_register(port, &verify_cfg);
  if (ret == ESP_OK) {
    bool actual_is_output = !(verify_cfg & (1 << pin_num));
    ESP_LOGI(TAG, "After: Stack%d GPIO%d (P%d.%d) config=0x%02X - %s %s",
             stack_id + 1, pin + 1, port, pin_num, verify_cfg,
             actual_is_output ? "OUTPUT" : "INPUT",
             (actual_is_output == is_output) ? "OK" : "FAILED");
  }

  return ret;
}
/* ===== New APIs for Module Controller Support ===== */

esp_err_t stack_handler_gpio_write_multi(uint8_t stack_id,
                                         const gpio_action_t *actions,
                                         size_t count) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id) || !actions || count == 0) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  // Group actions by TCA port to minimize I2C transactions
  uint8_t port_masks[3] = {0};  // Bitmask of which pins to modify on each port
  uint8_t port_states[3] = {0}; // Desired state for modified pins

  // Build port masks and states
  for (size_t i = 0; i < count; i++) {
    if (!is_valid_pin(actions[i].pin)) {
      ESP_LOGW(TAG, "Skipping invalid pin %d", actions[i].pin);
      continue;
    }

    tca_port_t port;
    uint8_t pin_num;
    get_tca_mapping(stack_id, actions[i].pin, &port, &pin_num);

    port_masks[port] |= (1 << pin_num);
    if (actions[i].level) {
      port_states[port] |= (1 << pin_num);
    }
  }

  // Write to each port once (batched operation)
  esp_err_t ret = ESP_OK;
  for (int port = 0; port < 3; port++) {
    if (port_masks[port] != 0) {
      // Read current state
      uint8_t current;
      ret = tca_read_output_register(port, &current);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read P%d output register", port);
        break;
      }

      // Modify only target bits
      current = (current & ~port_masks[port]) |
                (port_states[port] & port_masks[port]);

      // Write back
      ret = tca_write_output_register(port, current);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write P%d output register", port);
        break;
      }
    }
  }

  ESP_LOGD(TAG, "Stack%d: Batch wrote %d GPIO actions", stack_id + 1, count);
  return ret;
}

esp_err_t stack_handler_gpio_get_state(uint8_t stack_id,
                                       stack_gpio_pin_num_t pin, bool *state) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id) || !is_valid_pin(pin) || !state) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  // Read current state (same as stack_handler_gpio_read)
  return stack_handler_gpio_read(stack_id, pin, state);
}

esp_err_t stack_handler_lock(uint8_t stack_id) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id)) {
    ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(g_stack_mutex[stack_id], pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire mutex for stack%d", stack_id + 1);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "Stack%d locked", stack_id + 1);
  return ESP_OK;
}

esp_err_t stack_handler_unlock(uint8_t stack_id) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id)) {
    ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreGive(g_stack_mutex[stack_id]);
  ESP_LOGD(TAG, "Stack%d unlocked", stack_id + 1);

  return ESP_OK;
}

/**
 * @brief Get stack module ID for Module Base Setting architecture (trial version)
 * 
 * For trial phase (BLE-only):
 * - Stack 0: Returns "002" (BLE STM32WB module)
 * - Stack 1: Returns "000" (no module)
 * 
 * This is a pseudo-implementation for testing Module Base Setting concept.
 * In production, this should detect actual module hardware.
 */
const char* stack_handler_get_module_id(uint8_t stack_id) {
  if (!is_valid_stack_id(stack_id)) {
    ESP_LOGW(TAG, "Invalid stack_id %d, returning '000'", stack_id);
    return "000";
  }
  
  // Trial version (BLE-only): Stack 0 has BLE STM32WB module
  if (stack_id == 0) {
    ESP_LOGI(TAG, "Stack %d module ID: 002 (BLE STM32WB)", stack_id);
    return "002";
  } else {
    ESP_LOGI(TAG, "Stack %d module ID: 000 (No module)", stack_id);
    return "000";
  }
}
