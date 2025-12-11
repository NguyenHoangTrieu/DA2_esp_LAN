/**
 * @file wan_comm.c
 * @brief WAN Communication Library Implementation (Master - API only)
 */

#include "wan_comm.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WAN_COMM_MASTER";

/**
 * @brief Internal handle structure
 */
struct wan_comm_handle_s {
  // Configuration
  wan_comm_config_t config;

  // SPI handle
  spi_device_handle_t spi_device;

  // Buffers
  uint8_t *tx_buffer;
  uint8_t *rx_buffer;
  size_t buffer_size;

  // Synchronization
  SemaphoreHandle_t transfer_mutex;

  // State
  bool is_initialized;
  wan_comm_status_t last_error;

  // Error tracking
  uint32_t error_count;
};

// Forward declarations
static wan_comm_status_t wan_comm_validate_transaction(wan_comm_handle_t handle,
                                                       uint16_t length);
static void wan_comm_report_error(wan_comm_handle_t handle,
                                  wan_comm_status_t error, const char *context);

/**
 * @brief Initialize WAN communication library
 */
wan_comm_status_t wan_comm_init(const wan_comm_config_t *config,
                                wan_comm_handle_t *handle) {
  if (config == NULL || handle == NULL) {
    return WAN_COMM_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG,
           "Initializing WAN communication library (Master mode for LAN MCU)");

  // Allocate handle
  wan_comm_handle_t h =
      (wan_comm_handle_t)calloc(1, sizeof(struct wan_comm_handle_s));
  if (h == NULL) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return WAN_COMM_ERR_NO_MEM;
  }

  // Copy configuration
  memcpy(&h->config, config, sizeof(wan_comm_config_t));

  // Set defaults
  if (h->config.clock_speed_hz == 0) {
    h->config.clock_speed_hz = WAN_COMM_DEFAULT_CLOCK_HZ;
  }
  if (h->config.queue_size == 0) {
    h->config.queue_size = WAN_COMM_DEFAULT_QUEUE_SIZE;
  }
  if (h->config.dma_channel == 0) {
    h->config.dma_channel = SPI_DMA_CH_AUTO;
  }

  // Allocate buffers (DMA-capable memory)
  h->buffer_size = WAN_COMM_MAX_TRANSFER_SIZE + WAN_COMM_HEADER_SIZE;
  h->tx_buffer = (uint8_t *)heap_caps_malloc(h->buffer_size, MALLOC_CAP_DMA);
  h->rx_buffer = (uint8_t *)heap_caps_malloc(h->buffer_size, MALLOC_CAP_DMA);

  if (h->tx_buffer == NULL || h->rx_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate DMA buffers");
    free(h->tx_buffer);
    free(h->rx_buffer);
    free(h);
    return WAN_COMM_ERR_NO_MEM;
  }

  // Create mutex
  h->transfer_mutex = xSemaphoreCreateMutex();
  if (h->transfer_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex");
    free(h->tx_buffer);
    free(h->rx_buffer);
    free(h);
    return WAN_COMM_ERR_NO_MEM;
  }

  // Configure SPI bus
  spi_bus_config_t bus_cfg = {
      .mosi_io_num = config->gpio_io0,
      .miso_io_num = config->gpio_io1,
      .sclk_io_num = config->gpio_sck,
      .quadwp_io_num = config->enable_quad_mode ? config->gpio_io2 : -1,
      .quadhd_io_num = config->enable_quad_mode ? config->gpio_io3 : -1,
      .max_transfer_sz = h->buffer_size,
      .flags = SPICOMMON_BUSFLAG_MASTER};

  esp_err_t ret =
      spi_bus_initialize(config->host_id, &bus_cfg, config->dma_channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    free(h->tx_buffer);
    free(h->rx_buffer);
    vSemaphoreDelete(h->transfer_mutex);
    free(h);
    return WAN_COMM_ERR_INVALID_STATE;
  }

  // Configure SPI device
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = config->clock_speed_hz,
      .mode = config->mode,
      .spics_io_num = config->gpio_cs,
      .queue_size = config->queue_size,
      .flags = config->enable_quad_mode ? SPI_DEVICE_HALFDUPLEX : 0,
      .pre_cb = NULL,
      .post_cb = NULL};

  ret = spi_bus_add_device(config->host_id, &dev_cfg, &h->spi_device);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    spi_bus_free(config->host_id);
    free(h->tx_buffer);
    free(h->rx_buffer);
    vSemaphoreDelete(h->transfer_mutex);
    free(h);
    return WAN_COMM_ERR_INVALID_STATE;
  }

  // Initialize state
  h->is_initialized = true;
  h->last_error = WAN_COMM_OK;
  h->error_count = 0;

  *handle = h;
  ESP_LOGI(TAG, "WAN communication initialized successfully (LAN MCU Master)");
  ESP_LOGI(TAG, "Clock: %lu Hz, Mode: %d, Quad: %s", config->clock_speed_hz,
           config->mode, config->enable_quad_mode ? "Yes" : "No");

  return WAN_COMM_OK;
}

/**
 * @brief Deinitialize WAN communication library
 */
wan_comm_status_t wan_comm_deinit(wan_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return WAN_COMM_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing WAN communication library");

  // Remove device
  spi_bus_remove_device(handle->spi_device);

  // Free bus
  spi_bus_free(handle->config.host_id);

  // Free resources
  free(handle->tx_buffer);
  free(handle->rx_buffer);
  vSemaphoreDelete(handle->transfer_mutex);

  handle->is_initialized = false;
  free(handle);

  ESP_LOGI(TAG, "WAN communication deinitialized");
  return WAN_COMM_OK;
}

/**
 * @brief Send command packet
 */
wan_comm_status_t wan_comm_send_command(wan_comm_handle_t handle,
                                        const uint8_t *command_payload,
                                        uint16_t length) {
  if (handle == NULL || !handle->is_initialized) {
    return WAN_COMM_ERR_NOT_INITIALIZED;
  }

  if (command_payload == NULL || length == 0) {
    return WAN_COMM_ERR_INVALID_ARG;
  }

  wan_comm_status_t status = wan_comm_validate_transaction(handle, length);
  if (status != WAN_COMM_OK) {
    return status;
  }

  // Take mutex
  if (xSemaphoreTake(handle->transfer_mutex,
                     pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
    wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT,
                          "send_command mutex timeout");
    return WAN_COMM_ERR_TIMEOUT;
  }

  // Build packet: [CF header][payload]
  handle->tx_buffer[0] = (WAN_COMM_HEADER_CF >> 8) & 0xFF;
  handle->tx_buffer[1] = WAN_COMM_HEADER_CF & 0xFF;
  memcpy(&handle->tx_buffer[WAN_COMM_HEADER_SIZE], command_payload, length);

  // Prepare transaction
  spi_transaction_t trans = {.length =
                                 (length + WAN_COMM_HEADER_SIZE) * 8, // in bits
                             .tx_buffer = handle->tx_buffer,
                             .rx_buffer = NULL};

  // Transmit (blocking)
  esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);

  xSemaphoreGive(handle->transfer_mutex);

  if (ret != ESP_OK) {
    wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY,
                          "send_command SPI error");
    return WAN_COMM_ERR_BUS_BUSY;
  }

  ESP_LOGI(TAG, "Command sent: %d bytes", length);
  return WAN_COMM_OK;
}

/**
 * @brief Send data packet
 */
wan_comm_status_t wan_comm_send_data(wan_comm_handle_t handle,
                                     const uint8_t *data_payload,
                                     uint16_t length) {
  if (handle == NULL || !handle->is_initialized) {
    return WAN_COMM_ERR_NOT_INITIALIZED;
  }

  if (data_payload == NULL || length == 0) {
    return WAN_COMM_ERR_INVALID_ARG;
  }

  wan_comm_status_t status = wan_comm_validate_transaction(handle, length);
  if (status != WAN_COMM_OK) {
    return status;
  }

  // Take mutex
  if (xSemaphoreTake(handle->transfer_mutex,
                     pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
    wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT,
                          "send_data mutex timeout");
    return WAN_COMM_ERR_TIMEOUT;
  }

  // Build packet: [DT header][payload]
  handle->tx_buffer[0] = (WAN_COMM_HEADER_DT >> 8) & 0xFF;
  handle->tx_buffer[1] = WAN_COMM_HEADER_DT & 0xFF;
  memcpy(&handle->tx_buffer[WAN_COMM_HEADER_SIZE], data_payload, length);

  // Prepare transaction
  spi_transaction_t trans = {.length =
                                 (length + WAN_COMM_HEADER_SIZE) * 8, // in bits
                             .tx_buffer = handle->tx_buffer,
                             .rx_buffer = NULL};

  // Transmit (blocking)
  esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);

  xSemaphoreGive(handle->transfer_mutex);

  if (ret != ESP_OK) {
    wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "send_data SPI error");
    return WAN_COMM_ERR_BUS_BUSY;
  }

  ESP_LOGI(TAG, "Data sent: %d bytes", length);
  return WAN_COMM_OK;
}

/**
 * @brief Request data from slave
 */
wan_comm_status_t wan_comm_request_data(wan_comm_handle_t handle,
                                        uint8_t *rx_buffer,
                                        uint16_t length_to_read) {
    if (handle == NULL || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }

    if (rx_buffer == NULL || length_to_read == 0) {
        return WAN_COMM_ERR_INVALID_ARG;
    }

    wan_comm_status_t status =
        wan_comm_validate_transaction(handle, length_to_read);
    if (status != WAN_COMM_OK) {
        return status;
    }

    // Take mutex
    if (xSemaphoreTake(handle->transfer_mutex,
                      pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT,
                            "request_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }

    // BUILD DUMMY TX PACKET WITH CORRECT HEADER
    memset(handle->tx_buffer, 0, length_to_read + WAN_COMM_HEADER_SIZE);
    handle->tx_buffer[0] = (WAN_COMM_HEADER_CF >> 8) & 0xFF;  // 'C' = 0x43
    handle->tx_buffer[1] = WAN_COMM_HEADER_CF & 0xFF;         // 'F' = 0x46
    // Bytes 2+ are 0x00 (no payload, just polling)

    // PREPARE FULL-DUPLEX TRANSACTION
    spi_transaction_t trans = {
        .length = length_to_read * 8,       // Total bits to transfer
        .rxlength = length_to_read * 8,     // Bits to receive
        .tx_buffer = handle->tx_buffer,     // TX with correct header
        .rx_buffer = handle->rx_buffer
    };

    // Transmit (blocking)
    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    
    if (ret == ESP_OK) {
        // Copy to user buffer
        memcpy(rx_buffer, handle->rx_buffer, length_to_read);
    }

    xSemaphoreGive(handle->transfer_mutex);

    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY,
                            "request_data SPI error");
        return WAN_COMM_ERR_BUS_BUSY;
    }

    ESP_LOGI(TAG, "Data polled: %d bytes", length_to_read);
    return WAN_COMM_OK;
}

/**
 * @brief Get last error
 */
wan_comm_status_t wan_comm_get_last_error(wan_comm_handle_t handle) {
  if (handle == NULL) {
    return WAN_COMM_ERR_INVALID_ARG;
  }
  return handle->last_error;
}

/**
 * @brief Get error count
 */
uint32_t wan_comm_get_error_count(wan_comm_handle_t handle) {
  if (handle == NULL) {
    return 0;
  }
  return handle->error_count;
}

/**
 * @brief Clear error count
 */
wan_comm_status_t wan_comm_clear_error_count(wan_comm_handle_t handle) {
  if (handle == NULL || !handle->is_initialized) {
    return WAN_COMM_ERR_NOT_INITIALIZED;
  }

  handle->error_count = 0;
  return WAN_COMM_OK;
}

// ===== Internal Functions =====

/**
 * @brief Validate transaction parameters
 */
static wan_comm_status_t wan_comm_validate_transaction(wan_comm_handle_t handle,
                                                       uint16_t length) {
  if (length > WAN_COMM_MAX_TRANSFER_SIZE) {
    ESP_LOGE(TAG, "Transfer size %d exceeds maximum %d", length,
             WAN_COMM_MAX_TRANSFER_SIZE);
    return WAN_COMM_ERR_INVALID_ARG;
  }
  return WAN_COMM_OK;
}

/**
 * @brief Report error
 */
static void wan_comm_report_error(wan_comm_handle_t handle,
                                  wan_comm_status_t error,
                                  const char *context) {
  if (handle == NULL) {
    return;
  }

  handle->last_error = error;
  handle->error_count++;
  ESP_LOGE(TAG, "Error: %d, Context: %s", error, context);
}
