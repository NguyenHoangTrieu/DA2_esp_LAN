/**
 * @file stack_handler.c
 * @brief Communication Stack Manager Implementation
 *
 * GPIO Mapping according to schematic:
 * Stack 1: GPIO1=P02, GPIO2=P03, GPIO3=P04, GPIO4=P05, GPIO5=P06, GPIO6=P07,
 * GPIO7=P08, GPIO8=P12, GPIO9=P13 Stack 2: GPIO1=P15, GPIO2=P16, GPIO3=P17,
 * GPIO4=P20, GPIO5=P21, GPIO6=P22, GPIO7=P23, GPIO8=P24, GPIO9=P25
 */

#include "stack_handler.h"
#include "esp_log.h"
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

/* ===== Global Variables ===== */
stack_comm_type_t g_stack_1_type = STACK_COMM_TYPE_RS485;
stack_comm_type_t g_stack_2_type = STACK_COMM_TYPE_LORA;

/* ===== Internal State ===== */
static stack_config_t g_stack_configs[STACK_HANDLER_MAX_STACKS] = {
    {.comm_type = STACK_COMM_TYPE_NONE,
     .gpio_port = STACK_PORT_1,
     .uart_port = 2,
     .tx_pin = 17,
     .rx_pin = 18,
     .enabled = false},
    {.comm_type = STACK_COMM_TYPE_NONE,
     .gpio_port = STACK_PORT_2,
     .uart_port = 1,
     .tx_pin = 15,
     .rx_pin = 16,
     .enabled = false}};

static bool g_initialized = false;

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

  g_initialized = true;

  ESP_LOGI(TAG, "Stack handler initialized");
  ESP_LOGI(TAG, "  Stack 1 GPIO mapping: P02-P07, P10-P12");
  ESP_LOGI(TAG, "  Stack 2 GPIO mapping: P15-P17, P20-P25");

  return ESP_OK;
}

esp_err_t stack_handler_set_config(uint8_t stack_id,
                                   const stack_config_t *config) {
  if (!g_initialized) {
    ESP_LOGE(TAG, "Not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!is_valid_stack_id(stack_id) || !config) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  memcpy(&g_stack_configs[stack_id], config, sizeof(stack_config_t));

  if (stack_id == 0) {
    g_stack_1_type = config->comm_type;
  } else {
    g_stack_2_type = config->comm_type;
  }

  ESP_LOGI(TAG, "Stack %d configured: type=%d, UART%d", stack_id + 1,
           config->comm_type, config->uart_port);

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

const char *stack_handler_type_to_string(stack_comm_type_t type) {
  switch (type) {
  case STACK_COMM_TYPE_NONE:
    return "NONE";
  case STACK_COMM_TYPE_LORA:
    return "LORA";
  case STACK_COMM_TYPE_RS485:
    return "RS485";
  case STACK_COMM_TYPE_ZIGBEE:
    return "ZIGBEE";
  case STACK_COMM_TYPE_CAN:
    return "CAN";
  default:
    return "UNKNOWN";
  }
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
