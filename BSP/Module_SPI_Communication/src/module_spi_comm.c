/**
 * @file module_spi_comm.c
 * @brief Generic SPI Communication Driver Implementation
 */

#include "module_spi_comm.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_SPI";

/* ===== Internal Handle Structure ===== */

struct module_spi_comm_s {
  uint8_t stack_id;
  spi_host_device_t host;
  spi_device_handle_t spi_device;
  uint32_t clock_speed_hz;
  uint8_t mode;
  bool initialized;
  /* True only if THIS handle called spi_bus_initialize() — i.e. it was the
   * first device on the bus.  Only the owner may call spi_bus_free().
   * The second device on a shared bus must NOT free the bus on deinit,
   * because the first device's handle is still using it. */
  bool bus_initialized_by_us;
};

/* ===== Helper Functions ===== */

static inline bool is_valid_handle(module_spi_comm_handle_t handle) {
  return (handle != NULL && handle->initialized);
}

/* ===== Public API Implementation ===== */

esp_err_t module_spi_comm_init(const module_spi_config_t *config,
                               module_spi_comm_handle_t *handle) {
  if (!config || !handle) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  if (config->stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d (must be 0 or 1)", config->stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  // Get hardcoded pins based on stack_id
  spi_host_device_t host;
  int mosi_pin, miso_pin, sclk_pin, cs_pin;
  
  if (config->stack_id == 0) {
    host = STACK0_SPI_HOST;
    mosi_pin = STACK0_SPI_MOSI_PIN;
    miso_pin = STACK0_SPI_MISO_PIN;
    sclk_pin = STACK0_SPI_SCLK_PIN;
    cs_pin = STACK0_SPI_CS_PIN;
  } else {
    host = STACK1_SPI_HOST;
    mosi_pin = STACK1_SPI_MOSI_PIN;
    miso_pin = STACK1_SPI_MISO_PIN;
    sclk_pin = STACK1_SPI_SCLK_PIN;
    cs_pin = STACK1_SPI_CS_PIN;
  }

  ESP_LOGI(TAG, "Initializing SPI for Stack%d: host=%d, MOSI=%d, MISO=%d, SCLK=%d, CS=%d",
           config->stack_id, host, mosi_pin, miso_pin, sclk_pin, cs_pin);

  // Allocate handle
  module_spi_comm_handle_t spi_handle =
      (module_spi_comm_handle_t)calloc(1, sizeof(struct module_spi_comm_s));
  if (!spi_handle) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ESP_ERR_NO_MEM;
  }

  // Configure SPI bus
  spi_bus_config_t bus_config = {
      .mosi_io_num = mosi_pin,
      .miso_io_num = miso_pin,
      .sclk_io_num = sclk_pin,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4096,
  };

  // Initialize SPI bus.
  // ESP_ERR_INVALID_STATE means the bus was already initialized by another
  // device (e.g. Stack 0 already called spi_bus_initialize for SPI3_HOST).
  // In that case we attach our device to the existing bus without taking
  // ownership — spi_bus_free() must NOT be called by this handle on deinit.
  esp_err_t ret = spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO);
  if (ret == ESP_OK) {
    spi_handle->bus_initialized_by_us = true;
  } else if (ret == ESP_ERR_INVALID_STATE) {
    /* Bus already up — attach-only, no ownership. */
    spi_handle->bus_initialized_by_us = false;
    ESP_LOGI(TAG, "SPI%d bus already initialized, attaching device only", host + 1);
  } else {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    free(spi_handle);
    return ret;
  }

  // Configure SPI device
  spi_device_interface_config_t dev_config = {
      .clock_speed_hz = config->clock_speed_hz,
      .mode = config->mode,
      .spics_io_num = cs_pin,
      .queue_size = (config->queue_size > 0) ? config->queue_size : 1,
      .flags = 0,
      .pre_cb = NULL,
      .post_cb = NULL,
  };

  // Add device to bus.
  // On failure: only free the bus if THIS handle initialized it.
  // If we attached to an existing bus, do NOT free it — that would destroy
  // the other stack's device that is still mounted on the bus.
  ret = spi_bus_add_device(host, &dev_config, &spi_handle->spi_device);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    if (spi_handle->bus_initialized_by_us) {
      spi_bus_free(host);
    }
    free(spi_handle);
    return ret;
  }

  // Store configuration
  spi_handle->stack_id = config->stack_id;
  spi_handle->host = host;
  spi_handle->clock_speed_hz = config->clock_speed_hz;
  spi_handle->mode = config->mode;
  spi_handle->initialized = true;

  *handle = spi_handle;

  ESP_LOGI(TAG,
           "SPI initialized for Stack%d: host=%d, MOSI=%d, MISO=%d, SCLK=%d, CS=%d, clock=%lu Hz, mode=%d",
           config->stack_id, host, mosi_pin, miso_pin, sclk_pin,
           cs_pin, config->clock_speed_hz, config->mode);

  return ESP_OK;
}

esp_err_t module_spi_comm_transfer(module_spi_comm_handle_t handle,
                                   const uint8_t *tx_data, uint8_t *rx_data,
                                   size_t len) {
  if (!is_valid_handle(handle) || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!tx_data && !rx_data) {
    ESP_LOGE(TAG, "Both TX and RX buffers are NULL");
    return ESP_ERR_INVALID_ARG;
  }

  // Prepare transaction
  spi_transaction_t trans = {
      .length = len * 8, // Length in bits
      .tx_buffer = tx_data,
      .rx_buffer = rx_data,
  };

  // Execute transaction
  esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGD(TAG, "SPI transferred %d bytes", len);
  return ESP_OK;
}

esp_err_t module_spi_comm_deinit(module_spi_comm_handle_t handle) {
  if (!is_valid_handle(handle)) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing SPI%d (stack_id=%d, bus_owner=%d)",
           handle->host + 1, handle->stack_id,
           (int)handle->bus_initialized_by_us);

  // Always remove our device from the bus.
  esp_err_t ret = spi_bus_remove_device(handle->spi_device);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
  }

  // Only free the SPI bus if this handle was the one that initialized it.
  // If another stack device is still attached to the same host, freeing the
  // bus here would corrupt that device's subsequent transactions.
  if (handle->bus_initialized_by_us) {
    ret = spi_bus_free(handle->host);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
    } else {
      ESP_LOGI(TAG, "SPI%d bus freed (was bus owner)", handle->host + 1);
    }
  } else {
    ESP_LOGI(TAG, "SPI%d bus NOT freed (not bus owner, other devices may remain)",
             handle->host + 1);
  }

  // Mark as uninitialized
  handle->initialized = false;

  // Free handle
  free(handle);

  return ESP_OK;
}
