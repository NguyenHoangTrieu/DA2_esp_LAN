/**
 * @file json_config_parser.c
 * @brief Common JSON configuration parser implementation
 */

#include "json_config_parser.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "JSON_PARSER";

/* ============================================================================
 * Internal Helper Functions
 * ========================================================================== */

/**
 * @brief Convert string to port type enum
 */
static comm_port_type_t string_to_port_type(const char *str) {
  if (strcmp(str, "uart") == 0)
    return COMM_PORT_UART;
  if (strcmp(str, "spi") == 0)
    return COMM_PORT_SPI;
  if (strcmp(str, "i2c") == 0)
    return COMM_PORT_I2C;
  if (strcmp(str, "usb") == 0)
    return COMM_PORT_USB;
  return COMM_PORT_MAX;
}

/**
 * @brief Convert string to UART parity enum
 */
static module_uart_parity_t string_to_parity(const char *str) {
  if (strcmp(str, "none") == 0)
    return MODULE_UART_PARITY_NONE;
  if (strcmp(str, "even") == 0)
    return MODULE_UART_PARITY_EVEN;
  if (strcmp(str, "odd") == 0)
    return MODULE_UART_PARITY_ODD;
  return MODULE_UART_PARITY_NONE;
}

/**
 * @brief Parse UART parameters from JSON
 */
static esp_err_t parse_uart_params(cJSON *params, uart_params_t *uart) {
  cJSON *baudrate = cJSON_GetObjectItem(params, "baudrate");
  cJSON *parity = cJSON_GetObjectItem(params, "parity");
  cJSON *stopbit = cJSON_GetObjectItem(params, "stopbit");

  if (!cJSON_IsNumber(baudrate)) {
    ESP_LOGE(TAG, "Missing or invalid 'baudrate' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsString(parity)) {
    ESP_LOGE(TAG, "Missing or invalid 'parity' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsNumber(stopbit)) {
    ESP_LOGE(TAG, "Missing or invalid 'stopbit' field");
    return ESP_ERR_INVALID_ARG;
  }

  uart->baudrate = baudrate->valueint;
  uart->parity = string_to_parity(parity->valuestring);
  uart->stopbit = stopbit->valueint;

  ESP_LOGI(TAG, "UART: baudrate=%d, parity=%d, stopbit=%d", uart->baudrate,
           uart->parity, uart->stopbit);

  return ESP_OK;
}

/**
 * @brief Parse SPI parameters from JSON
 */
static esp_err_t parse_spi_params(cJSON *params, spi_params_t *spi) {
  cJSON *clock_speed = cJSON_GetObjectItem(params, "clock_speed");
  cJSON *mode = cJSON_GetObjectItem(params, "mode");
  cJSON *bit_order = cJSON_GetObjectItem(params, "bit_order");

  if (!cJSON_IsNumber(clock_speed)) {
    ESP_LOGE(TAG, "Missing or invalid 'clock_speed' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsNumber(mode)) {
    ESP_LOGE(TAG, "Missing or invalid 'mode' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsString(bit_order)) {
    ESP_LOGE(TAG, "Missing or invalid 'bit_order' field");
    return ESP_ERR_INVALID_ARG;
  }

  spi->clock_speed = clock_speed->valueint;
  spi->mode = mode->valueint;
  strncpy(spi->bit_order, bit_order->valuestring, sizeof(spi->bit_order) - 1);

  ESP_LOGI(TAG, "SPI: clock=%d, mode=%d, bit_order=%s", spi->clock_speed,
           spi->mode, spi->bit_order);

  return ESP_OK;
}

/**
 * @brief Parse I2C parameters from JSON
 */
static esp_err_t parse_i2c_params(cJSON *params, i2c_params_t *i2c) {
  cJSON *address = cJSON_GetObjectItem(params, "address");
  cJSON *clock_speed = cJSON_GetObjectItem(params, "clock_speed");

  if (!cJSON_IsNumber(address)) {
    ESP_LOGE(TAG, "Missing or invalid 'address' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsNumber(clock_speed)) {
    ESP_LOGE(TAG, "Missing or invalid 'clock_speed' field");
    return ESP_ERR_INVALID_ARG;
  }

  i2c->address = address->valueint;
  i2c->clock_speed = clock_speed->valueint;

  ESP_LOGI(TAG, "I2C: address=0x%02X, clock=%d", i2c->address,
           i2c->clock_speed);

  return ESP_OK;
}

/**
 * @brief Parse USB CDC parameters from JSON
 */
static esp_err_t parse_usb_params(cJSON *params, usb_params_t *usb) {
  cJSON *bit_rate = cJSON_GetObjectItem(params, "bit_rate");
  cJSON *stop_bits = cJSON_GetObjectItem(params, "stop_bits");
  cJSON *parity = cJSON_GetObjectItem(params, "parity");
  cJSON *data_bits = cJSON_GetObjectItem(params, "data_bits");

  if (!cJSON_IsNumber(bit_rate)) {
    ESP_LOGE(TAG, "Missing or invalid 'bit_rate' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsNumber(stop_bits)) {
    ESP_LOGE(TAG, "Missing or invalid 'stop_bits' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsNumber(parity)) {
    ESP_LOGE(TAG, "Missing or invalid 'parity' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsNumber(data_bits)) {
    ESP_LOGE(TAG, "Missing or invalid 'data_bits' field");
    return ESP_ERR_INVALID_ARG;
  }

  usb->bit_rate = bit_rate->valueint;
  usb->stop_bits = stop_bits->valueint;
  usb->parity = parity->valueint;
  usb->data_bits = data_bits->valueint;

  ESP_LOGI(TAG, "USB: bit_rate=%lu, stop_bits=%d, parity=%d, data_bits=%d",
           usb->bit_rate, usb->stop_bits, usb->parity, usb->data_bits);

  return ESP_OK;
}

/**
 * @brief Parse communication configuration from JSON
 */
static esp_err_t parse_communication(cJSON *comm_json, comm_config_t *comm) {
  cJSON *port_type = cJSON_GetObjectItem(comm_json, "port_type");
  cJSON *parameters = cJSON_GetObjectItem(comm_json, "parameters");

  if (!cJSON_IsString(port_type)) {
    ESP_LOGE(TAG, "Missing or invalid 'port_type' field");
    return ESP_ERR_INVALID_ARG;
  }

  if (!cJSON_IsObject(parameters)) {
    ESP_LOGE(TAG, "Missing or invalid 'parameters' object");
    return ESP_ERR_INVALID_ARG;
  }

  comm->port_type = string_to_port_type(port_type->valuestring);
  if (comm->port_type == COMM_PORT_MAX) {
    ESP_LOGE(TAG, "Unsupported port_type: %s", port_type->valuestring);
    return ESP_ERR_NOT_SUPPORTED;
  }

  ESP_LOGI(TAG, "Port type: %s", port_type->valuestring);

  // Parse parameters based on port type
  esp_err_t ret = ESP_OK;
  switch (comm->port_type) {
  case COMM_PORT_UART:
    ret = parse_uart_params(parameters, &comm->params.uart);
    break;
  case COMM_PORT_SPI:
    ret = parse_spi_params(parameters, &comm->params.spi);
    break;
  case COMM_PORT_I2C:
    ret = parse_i2c_params(parameters, &comm->params.i2c);
    break;
  case COMM_PORT_USB:
    ret = parse_usb_params(parameters, &comm->params.usb);
    break;
  default:
    ret = ESP_ERR_NOT_SUPPORTED;
    break;
  }

  return ret;
}

/* ============================================================================
 * Public API Implementation
 * ========================================================================== */

esp_err_t json_config_parse_metadata(const char *json_str,
                                     module_metadata_t *metadata) {
  if (json_str == NULL || metadata == NULL) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  // Log JSON string details for debugging
  size_t json_len = strlen(json_str);
  ESP_LOGI(TAG, "Parsing JSON metadata: length=%zu", json_len);
  
  // Check null termination
  if (json_str[json_len] != '\0') {
    ESP_LOGE(TAG, "JSON string not null-terminated!");
    return ESP_ERR_INVALID_ARG;
  }
  
  // Log first 100 chars and last 100 chars
  if (json_len > 200) {
    ESP_LOGI(TAG, "JSON start: %.100s", json_str);
    ESP_LOGI(TAG, "JSON end: ...%s", json_str + json_len - 100);
  } else {
    ESP_LOGI(TAG, "JSON full: %s", json_str);
  }

  // Parse JSON string
  cJSON *root = cJSON_Parse(json_str);
  if (root == NULL) {
    const char *error_ptr = cJSON_GetErrorPtr();
    if (error_ptr != NULL) {
      // Find position in original string
      size_t error_pos = error_ptr - json_str;
      ESP_LOGE(TAG, "JSON parse error at position %zu", error_pos);
      ESP_LOGE(TAG, "JSON parse error before: %s", error_ptr);
      
      // Log context around error (50 chars before and after)
      if (error_pos > 50) {
        ESP_LOGE(TAG, "Context: ...%.50s >>> ERROR >>> %.50s...", 
                 json_str + error_pos - 50, error_ptr);
      } else {
        ESP_LOGE(TAG, "Context: %.50s >>> ERROR >>> %.50s...", 
                 json_str, error_ptr);
      }
    } else {
      ESP_LOGE(TAG, "JSON parse error (no error pointer)");
    }
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;

  // Extract module_id
  cJSON *module_id = cJSON_GetObjectItem(root, "module_id");
  if (!cJSON_IsString(module_id) || module_id->valuestring == NULL) {
    ESP_LOGE(TAG, "Missing or invalid 'module_id' field");
    ret = ESP_ERR_INVALID_ARG;
    goto cleanup;
  }
  strncpy(metadata->module_id, module_id->valuestring, MAX_MODULE_ID_LEN - 1);
  metadata->module_id[MAX_MODULE_ID_LEN - 1] = '\0';

  // Extract module_type
  cJSON *module_type = cJSON_GetObjectItem(root, "module_type");
  if (!cJSON_IsString(module_type) || module_type->valuestring == NULL) {
    ESP_LOGE(TAG, "Missing or invalid 'module_type' field");
    ret = ESP_ERR_INVALID_ARG;
    goto cleanup;
  }
  strncpy(metadata->module_type, module_type->valuestring,
          MAX_MODULE_TYPE_LEN - 1);
  metadata->module_type[MAX_MODULE_TYPE_LEN - 1] = '\0';

  // Extract module_name
  cJSON *module_name = cJSON_GetObjectItem(root, "module_name");
  if (!cJSON_IsString(module_name) || module_name->valuestring == NULL) {
    ESP_LOGE(TAG, "Missing or invalid 'module_name' field");
    ret = ESP_ERR_INVALID_ARG;
    goto cleanup;
  }
  strncpy(metadata->module_name, module_name->valuestring,
          MAX_MODULE_NAME_LEN - 1);
  metadata->module_name[MAX_MODULE_NAME_LEN - 1] = '\0';

  // Extract module_communication
  cJSON *module_communication =
      cJSON_GetObjectItem(root, "module_communication");
  if (!cJSON_IsObject(module_communication)) {
    ESP_LOGE(TAG, "Missing or invalid 'module_communication' object");
    ret = ESP_ERR_INVALID_ARG;
    goto cleanup;
  }

  ret = parse_communication(module_communication, &metadata->communication);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to parse communication config");
    goto cleanup;
  }

  ESP_LOGI(TAG, "Metadata parsed: id=%s, type=%s, name=%s", metadata->module_id,
           metadata->module_type, metadata->module_name);

cleanup:
  cJSON_Delete(root);
  return ret;
}
