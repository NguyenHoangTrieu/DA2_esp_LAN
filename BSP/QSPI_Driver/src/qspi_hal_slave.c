// BSP/QSPI_Driver/src/qspi_hal_slave.c

#include "driver/gpio.h"
#include "driver/spi_slave_hd.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "qspi_hal.h"
#include "qspi_hal_config.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "QSPI_SLAVE";

typedef struct qspi_hal_s {
  TaskHandle_t rx_task_handle;
  TaskHandle_t tx_task_handle;
  SemaphoreHandle_t tx_mutex;
  QueueHandle_t tx_queue;

  qspi_hal_config_t config;
  qspi_hal_stats_t stats;

  uint8_t *rx_buf[2]; // Ping-pong RX
  uint8_t *tx_buf[2]; // Double-buffered TX
  uint8_t tx_buf_idx;
  size_t tx_len;

  volatile bool running;
  volatile bool initialized;
} qspi_hal_t;

typedef struct {
  uint8_t *data;
  size_t len;
  uint8_t type;
} tx_queue_item_t;

static void qspi_slave_rx_task(void *arg);
static void qspi_slave_tx_task(void *arg);
static void qspi_parse_rx_frame(qspi_hal_t *hal, uint8_t *rx_data,
                                size_t trans_len);

esp_err_t qspi_hal_init(const qspi_hal_config_t *config,
                        qspi_hal_handle_t *handle) {
  if (config == NULL || handle == NULL || config->mode != QSPI_MODE_SLAVE)
    return ESP_ERR_INVALID_ARG;

  qspi_hal_t *hal = calloc(1, sizeof(qspi_hal_t));
  if (hal == NULL)
    return ESP_ERR_NO_MEM;

  memcpy(&hal->config, config, sizeof(qspi_hal_config_t));

  // Set default pins
  if (hal->config.gpio_clk == 0)
    hal->config.gpio_clk = QSPI_PIN_CLK;
  if (hal->config.gpio_cs == 0)
    hal->config.gpio_cs = QSPI_PIN_CS;
  if (hal->config.gpio_d0 == 0)
    hal->config.gpio_d0 = QSPI_PIN_D0;
  if (hal->config.gpio_d1 == 0)
    hal->config.gpio_d1 = QSPI_PIN_D1;
  if (hal->config.gpio_d2 == 0)
    hal->config.gpio_d2 = QSPI_PIN_D2;
  if (hal->config.gpio_d3 == 0)
    hal->config.gpio_d3 = QSPI_PIN_D3;
  if (hal->config.gpio_dr == 0)
    hal->config.gpio_dr = QSPI_PIN_DR_SLAVE;

  // SPI Slave HD config
  spi_bus_config_t bus_cfg = {.mosi_io_num = hal->config.gpio_d0,
                              .miso_io_num = hal->config.gpio_d1,
                              .sclk_io_num = hal->config.gpio_clk,
                              .quadwp_io_num = hal->config.gpio_d2,
                              .quadhd_io_num = hal->config.gpio_d3,
                              .max_transfer_sz = QSPI_DMA_BUFFER_SIZE,
                              .flags = SPICOMMON_BUSFLAG_SLAVE |
                                       SPICOMMON_BUSFLAG_QUAD};

  spi_slave_hd_slot_config_t slave_cfg = {.spics_io_num = hal->config.gpio_cs,
                                          .flags = 0,
                                          .mode = 0,
                                          .command_bits = 8,
                                          .address_bits = 0,
                                          .dummy_bits = 0,
                                          .queue_size = QSPI_QUEUE_SIZE,
                                          .dma_chan = QSPI_DMA_CHAN};

  esp_err_t ret = spi_slave_hd_init(QSPI_HOST_ID, &bus_cfg, &slave_cfg);
  if (ret != ESP_OK) {
    free(hal);
    return ret;
  }

  // Allocate RX buffers (ping-pong)
  for (int i = 0; i < 2; i++) {
    hal->rx_buf[i] = heap_caps_malloc(QSPI_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (hal->rx_buf[i] == NULL) {
      for (int j = 0; j < i; j++)
        free(hal->rx_buf[j]);
      spi_slave_hd_deinit(QSPI_HOST_ID);
      free(hal);
      return ESP_ERR_NO_MEM;
    }
    memset(hal->rx_buf[i], 0, QSPI_DMA_BUFFER_SIZE);
  }

  // Allocate TX buffers (double-buffered)
  for (int i = 0; i < 2; i++) {
    hal->tx_buf[i] = heap_caps_malloc(QSPI_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (hal->tx_buf[i] == NULL) {
      for (int j = 0; j < i; j++)
        free(hal->tx_buf[j]);
      for (int j = 0; j < 2; j++)
        free(hal->rx_buf[j]);
      spi_slave_hd_deinit(QSPI_HOST_ID);
      free(hal);
      return ESP_ERR_NO_MEM;
    }
    memset(hal->tx_buf[i], 0xFF, QSPI_DMA_BUFFER_SIZE);
  }

  hal->tx_buf_idx = 0;

  // GPIO Data Ready (input)
  gpio_config_t io_cfg = {.pin_bit_mask = (1ULL << hal->config.gpio_dr),
                          .mode = GPIO_MODE_INPUT,
                          .pull_up_en = GPIO_PULLUP_ENABLE,
                          .pull_down_en = GPIO_PULLDOWN_DISABLE,
                          .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_cfg);

  hal->tx_mutex = xSemaphoreCreateMutex();
  hal->tx_queue = xQueueCreate(8, sizeof(tx_queue_item_t));

  if (hal->tx_mutex == NULL || hal->tx_queue == NULL) {
    if (hal->tx_mutex)
      vSemaphoreDelete(hal->tx_mutex);
    if (hal->tx_queue)
      vQueueDelete(hal->tx_queue);
    for (int i = 0; i < 2; i++) {
      free(hal->rx_buf[i]);
      free(hal->tx_buf[i]);
    }
    spi_slave_hd_deinit(QSPI_HOST_ID);
    free(hal);
    return ESP_ERR_NO_MEM;
  }

  hal->initialized = true;
  *handle = hal;

  ESP_LOGI(TAG, "Init: Slave HD QIO");
  return ESP_OK;
}

esp_err_t qspi_hal_deinit(qspi_hal_handle_t handle) {
  if (handle == NULL || !handle->initialized)
    return ESP_ERR_INVALID_ARG;

  if (handle->running)
    qspi_hal_stop(handle);

  vSemaphoreDelete(handle->tx_mutex);
  vQueueDelete(handle->tx_queue);

  for (int i = 0; i < 2; i++) {
    free(handle->rx_buf[i]);
    free(handle->tx_buf[i]);
  }

  spi_slave_hd_deinit(QSPI_HOST_ID);

  handle->initialized = false;
  free(handle);

  return ESP_OK;
}

esp_err_t qspi_hal_start(qspi_hal_handle_t handle) {
  if (handle == NULL || !handle->initialized)
    return ESP_ERR_INVALID_ARG;

  if (handle->running)
    return ESP_ERR_INVALID_STATE;

  handle->running = true;

  BaseType_t ret = xTaskCreate(qspi_slave_rx_task, "qspi_rx", 4096, handle, 11,
                               &handle->rx_task_handle);
  if (ret != pdPASS) {
    handle->running = false;
    return ESP_FAIL;
  }

  ret = xTaskCreate(qspi_slave_tx_task, "qspi_tx", 4096, handle, 10,
                    &handle->tx_task_handle);
  if (ret != pdPASS) {
    vTaskDelete(handle->rx_task_handle);
    handle->running = false;
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Started");
  return ESP_OK;
}

esp_err_t qspi_hal_stop(qspi_hal_handle_t handle) {
  if (handle == NULL || !handle->running)
    return ESP_ERR_INVALID_ARG;

  handle->running = false;

  if (handle->rx_task_handle) {
    vTaskDelete(handle->rx_task_handle);
    handle->rx_task_handle = NULL;
  }

  if (handle->tx_task_handle) {
    vTaskDelete(handle->tx_task_handle);
    handle->tx_task_handle = NULL;
  }

  return ESP_OK;
}

esp_err_t qspi_hal_stream_write(qspi_hal_handle_t handle, const uint8_t *data,
                                size_t len, uint8_t frame_type) {
  if (handle == NULL || !handle->initialized || data == NULL || len == 0)
    return ESP_ERR_INVALID_ARG;

  if (len > QSPI_DMA_BUFFER_SIZE - QSPI_HEADER_SIZE - 2)
    return ESP_ERR_INVALID_SIZE;

  tx_queue_item_t item;
  item.data = malloc(len);
  if (item.data == NULL)
    return ESP_ERR_NO_MEM;

  memcpy(item.data, data, len);
  item.len = len;
  item.type = frame_type;

  if (xQueueSend(handle->tx_queue, &item, pdMS_TO_TICKS(QSPI_TIMEOUT_MS)) !=
      pdPASS) {
    free(item.data);
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

static void qspi_slave_rx_task(void *arg) {
  qspi_hal_t *hal = (qspi_hal_t *)arg;
  uint8_t buf_idx = 0;

  while (hal->running) {
    // Wait for DR signal if enabled
    if (hal->config.enable_dr_handshake) {
      uint32_t wait_count = 0;
      while (gpio_get_level(hal->config.gpio_dr) == 0 && hal->running) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (++wait_count > QSPI_DR_POLL_TIMEOUT_MS) {
          break; // Timeout, check queue anyway
        }
      }
    }

    spi_slave_hd_data_t *ret_trans;
    spi_slave_hd_data_t trans = {.data = hal->rx_buf[buf_idx],
                                 .len = QSPI_DMA_BUFFER_SIZE};

    // Queue RX buffer
    esp_err_t ret = spi_slave_hd_queue_trans(QSPI_HOST_ID, SPI_SLAVE_CHAN_RX,
                                             &trans, portMAX_DELAY);
    if (ret != ESP_OK) {
      hal->stats.dma_errors++;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Wait for master write
    ret = spi_slave_hd_get_trans_res(QSPI_HOST_ID, SPI_SLAVE_CHAN_RX,
                                     &ret_trans, portMAX_DELAY);
    if (ret != ESP_OK || ret_trans->trans_len == 0) {
      buf_idx = (buf_idx + 1) % 2;
      continue;
    }

    // Parse frame
    size_t received_bytes = ret_trans->trans_len / 8;
    qspi_parse_rx_frame(hal, (uint8_t *)ret_trans->data, received_bytes);

    buf_idx = (buf_idx + 1) % 2;
  }

  vTaskDelete(NULL);
}

static void qspi_parse_rx_frame(qspi_hal_t *hal, uint8_t *rx_data,
                                size_t trans_len) {
  // Validate sync
  if (trans_len < QSPI_HEADER_SIZE ||
      rx_data[0] != ((QSPI_FRAME_SYNC >> 8) & 0xFF) ||
      rx_data[1] != (QSPI_FRAME_SYNC & 0xFF)) {
    return; // Not a valid frame
  }

  qspi_frame_t frame;
  frame.sync = (rx_data[0] << 8) | rx_data[1];
  frame.type = rx_data[2];
  frame.length = (rx_data[3] << 8) | rx_data[4];
  frame.header_crc = rx_data[5];

  // Validate length BEFORE accessing payload
  if (frame.length == 0 ||
      frame.length > QSPI_DMA_BUFFER_SIZE - QSPI_HEADER_SIZE - 2 ||
      frame.length > trans_len - QSPI_HEADER_SIZE - 2) {
    hal->stats.invalid_length_errors++;
    ESP_LOGW(TAG, "Invalid length: %d (trans_len=%d)", frame.length, trans_len);
    return;
  }

  frame.payload = &rx_data[QSPI_HEADER_SIZE];
  frame.payload_crc = (rx_data[QSPI_HEADER_SIZE + frame.length] << 8) |
                      rx_data[QSPI_HEADER_SIZE + frame.length + 1];

  bool valid = true;
  if (hal->config.enable_crc) {
    valid = qspi_validate_frame(&frame);
    if (!valid) {
      hal->stats.crc_errors++;
      ESP_LOGW(TAG, "CRC error");
    }
  }

  if (valid) {
    hal->stats.rx_frames++;
    hal->stats.rx_bytes += frame.length;

    if (hal->config.rx_callback)
      hal->config.rx_callback(&frame, hal->config.user_data);
  }
}

static void qspi_slave_tx_task(void *arg) {
  qspi_hal_t *hal = (qspi_hal_t *)arg;
  tx_queue_item_t tx_item;

  while (hal->running) {
    if (xQueueReceive(hal->tx_queue, &tx_item, pdMS_TO_TICKS(10)) == pdPASS) {
      if (xSemaphoreTake(hal->tx_mutex, portMAX_DELAY) == pdTRUE) {
        // Use double-buffered TX
        uint8_t curr_buf = hal->tx_buf_idx;
        uint8_t *tx_buffer = hal->tx_buf[curr_buf];

        size_t frame_len = QSPI_HEADER_SIZE + tx_item.len + 2;

        // Build frame
        tx_buffer[0] = (QSPI_FRAME_SYNC >> 8) & 0xFF;
        tx_buffer[1] = QSPI_FRAME_SYNC & 0xFF;
        tx_buffer[2] = tx_item.type;
        tx_buffer[3] = (tx_item.len >> 8) & 0xFF;
        tx_buffer[4] = tx_item.len & 0xFF;

        if (hal->config.enable_crc) {
          tx_buffer[5] = qspi_crc8(tx_buffer, 5);
          memcpy(&tx_buffer[QSPI_HEADER_SIZE], tx_item.data, tx_item.len);
          uint16_t pcrc = qspi_crc16_ccitt(tx_item.data, tx_item.len);
          tx_buffer[QSPI_HEADER_SIZE + tx_item.len] = (pcrc >> 8) & 0xFF;
          tx_buffer[QSPI_HEADER_SIZE + tx_item.len + 1] = pcrc & 0xFF;
        } else {
          tx_buffer[5] = 0;
          memcpy(&tx_buffer[QSPI_HEADER_SIZE], tx_item.data, tx_item.len);
          tx_buffer[QSPI_HEADER_SIZE + tx_item.len] = 0;
          tx_buffer[QSPI_HEADER_SIZE + tx_item.len + 1] = 0;
        }

        hal->tx_len = frame_len;

        // Queue TX transaction
        spi_slave_hd_data_t trans = {.data = tx_buffer, .len = frame_len};

        esp_err_t ret = spi_slave_hd_queue_trans(
            QSPI_HOST_ID, SPI_SLAVE_CHAN_TX, &trans, portMAX_DELAY);

        if (ret == ESP_OK) {
          // Toggle buffer AFTER queue (before wait)
          hal->tx_buf_idx = (hal->tx_buf_idx + 1) % 2;

          spi_slave_hd_data_t *ret_trans;
          ret = spi_slave_hd_get_trans_res(
              QSPI_HOST_ID, SPI_SLAVE_CHAN_TX, &ret_trans,
              pdMS_TO_TICKS(QSPI_SLAVE_TX_TIMEOUT_MS));

          if (ret == ESP_OK) {
            hal->stats.tx_frames++;
            hal->stats.tx_bytes += tx_item.len;
          } else if (ret == ESP_ERR_TIMEOUT) {
            hal->stats.timeout_errors++;
            ESP_LOGW(TAG, "TX timeout (master not reading)");
          } else {
            hal->stats.dma_errors++;
          }
        } else {
          hal->stats.dma_errors++;
          ESP_LOGE(TAG, "TX queue failed: %s", esp_err_to_name(ret));
        }

        xSemaphoreGive(hal->tx_mutex);
        free(tx_item.data);
      }
    }
  }

  vTaskDelete(NULL);
}

esp_err_t qspi_hal_get_stats(qspi_hal_handle_t handle,
                             qspi_hal_stats_t *stats) {
  if (handle == NULL || stats == NULL)
    return ESP_ERR_INVALID_ARG;

  memcpy(stats, &handle->stats, sizeof(qspi_hal_stats_t));
  return ESP_OK;
}

esp_err_t qspi_hal_clear_stats(qspi_hal_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  memset(&handle->stats, 0, sizeof(qspi_hal_stats_t));
  return ESP_OK;
}

bool qspi_hal_is_ready(qspi_hal_handle_t handle) {
  return (handle != NULL && handle->initialized && handle->running);
}
