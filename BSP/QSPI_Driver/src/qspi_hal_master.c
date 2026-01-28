// BSP/QSPI_Driver/src/qspi_hal_master.c

#include "driver/gpio.h"
#include "driver/spi_master.h"
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

static const char *TAG = "QSPI_MASTER";

typedef struct qspi_hal_s {
  spi_device_handle_t spi_dev;
  TaskHandle_t task_handle;
  SemaphoreHandle_t tx_mutex;
  QueueHandle_t tx_queue;

  qspi_hal_config_t config;
  qspi_hal_stats_t stats;

  uint8_t *dma_buf[2]; // [0] = TX, [1] = RX

  volatile bool running;
  volatile bool initialized;
} qspi_hal_t;

typedef struct {
  uint8_t *data;
  size_t len;
  uint8_t type;
} tx_queue_item_t;

static void qspi_master_task(void *arg);
static esp_err_t qspi_master_transmit(qspi_hal_t *hal, const uint8_t *data,
                                      size_t len);
static esp_err_t qspi_master_receive(qspi_hal_t *hal, uint8_t *buffer,
                                     size_t len);
static void qspi_parse_rx_frame(qspi_hal_t *hal, uint8_t *rx_buf);

esp_err_t qspi_hal_init(const qspi_hal_config_t *config,
                        qspi_hal_handle_t *handle) {
  if (config == NULL || handle == NULL || config->mode != QSPI_MODE_MASTER)
    return ESP_ERR_INVALID_ARG;

  qspi_hal_t *hal = calloc(1, sizeof(qspi_hal_t));
  if (hal == NULL)
    return ESP_ERR_NO_MEM;

  memcpy(&hal->config, config, sizeof(qspi_hal_config_t));

  // Set defaults
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
    hal->config.gpio_dr = QSPI_PIN_DR_MASTER;
  if (hal->config.clock_speed_hz == 0)
    hal->config.clock_speed_hz = QSPI_CLOCK_SPEED_HZ;

  // Initialize SPI bus
  spi_bus_config_t bus_cfg = {.mosi_io_num = hal->config.gpio_d0,
                              .miso_io_num = hal->config.gpio_d1,
                              .sclk_io_num = hal->config.gpio_clk,
                              .quadwp_io_num = hal->config.gpio_d2,
                              .quadhd_io_num = hal->config.gpio_d3,
                              .max_transfer_sz = QSPI_DMA_BUFFER_SIZE,
                              .flags = SPICOMMON_BUSFLAG_MASTER |
                                       SPICOMMON_BUSFLAG_QUAD};

  esp_err_t ret = spi_bus_initialize(QSPI_HOST_ID, &bus_cfg, QSPI_DMA_CHAN);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    free(hal);
    return ret;
  }

  // Add device - REMOVED SPI_DEVICE_HALFDUPLEX for full QIO
  spi_device_interface_config_t dev_cfg = {.command_bits = 8,
                                           .address_bits = 0,
                                           .dummy_bits = 0,
                                           .mode = 0,
                                           .clock_speed_hz =
                                               hal->config.clock_speed_hz,
                                           .spics_io_num = hal->config.gpio_cs,
                                           .queue_size = QSPI_QUEUE_SIZE,
                                           .flags = 0, // Full-duplex QIO mode
                                           .pre_cb = NULL,
                                           .post_cb = NULL};

  ret = spi_bus_add_device(QSPI_HOST_ID, &dev_cfg, &hal->spi_dev);
  if (ret != ESP_OK) {
    spi_bus_free(QSPI_HOST_ID);
    free(hal);
    return ret;
  }

  // Allocate DMA buffers
  for (int i = 0; i < 2; i++) {
    hal->dma_buf[i] = heap_caps_malloc(QSPI_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (hal->dma_buf[i] == NULL) {
      for (int j = 0; j < i; j++)
        free(hal->dma_buf[j]);
      spi_bus_remove_device(hal->spi_dev);
      spi_bus_free(QSPI_HOST_ID);
      free(hal);
      return ESP_ERR_NO_MEM;
    }
    memset(hal->dma_buf[i], 0, QSPI_DMA_BUFFER_SIZE);
  }

  // GPIO Data Ready (output for TX, input for monitoring slave)
  gpio_config_t io_cfg = {.pin_bit_mask = (1ULL << hal->config.gpio_dr),
                          .mode = GPIO_MODE_OUTPUT,
                          .pull_up_en = GPIO_PULLUP_DISABLE,
                          .pull_down_en = GPIO_PULLDOWN_DISABLE,
                          .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_cfg);
  gpio_set_level(hal->config.gpio_dr, 0);

  hal->tx_mutex = xSemaphoreCreateMutex();
  hal->tx_queue = xQueueCreate(8, sizeof(tx_queue_item_t));

  if (hal->tx_mutex == NULL || hal->tx_queue == NULL) {
    if (hal->tx_mutex)
      vSemaphoreDelete(hal->tx_mutex);
    if (hal->tx_queue)
      vQueueDelete(hal->tx_queue);
    for (int i = 0; i < 2; i++)
      free(hal->dma_buf[i]);
    spi_bus_remove_device(hal->spi_dev);
    spi_bus_free(QSPI_HOST_ID);
    free(hal);
    return ESP_ERR_NO_MEM;
  }

  hal->initialized = true;
  *handle = hal;

  ESP_LOGI(TAG, "Init: %d MHz QIO", hal->config.clock_speed_hz / 1000000);
  return ESP_OK;
}

esp_err_t qspi_hal_deinit(qspi_hal_handle_t handle) {
  if (handle == NULL || !handle->initialized)
    return ESP_ERR_INVALID_ARG;

  if (handle->running)
    qspi_hal_stop(handle);

  vSemaphoreDelete(handle->tx_mutex);
  vQueueDelete(handle->tx_queue);

  for (int i = 0; i < 2; i++)
    free(handle->dma_buf[i]);

  spi_bus_remove_device(handle->spi_dev);
  spi_bus_free(QSPI_HOST_ID);

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

  BaseType_t ret = xTaskCreate(qspi_master_task, "qspi_mst", 4096, handle, 10,
                               &handle->task_handle);

  if (ret != pdPASS) {
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

  if (handle->task_handle) {
    vTaskDelete(handle->task_handle);
    handle->task_handle = NULL;
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

esp_err_t qspi_hal_stream_read(qspi_hal_handle_t handle, uint8_t *buffer,
                               size_t *len, uint32_t timeout_ms) {
  return ESP_ERR_NOT_SUPPORTED;
}

static void qspi_master_task(void *arg) {
  qspi_hal_t *hal = (qspi_hal_t *)arg;
  tx_queue_item_t tx_item;
  uint8_t *tx_buf = hal->dma_buf[0];
  uint8_t *rx_buf = hal->dma_buf[1];

  TickType_t last_rx_poll = xTaskGetTickCount();
  const TickType_t poll_interval = pdMS_TO_TICKS(QSPI_MASTER_POLL_INTERVAL_MS);

  while (hal->running) {
    // TX: Check queue
    if (xQueueReceive(hal->tx_queue, &tx_item, 0) == pdPASS) {
      size_t frame_len = QSPI_HEADER_SIZE + tx_item.len + 2;

      // Build frame
      tx_buf[0] = (QSPI_FRAME_SYNC >> 8) & 0xFF;
      tx_buf[1] = QSPI_FRAME_SYNC & 0xFF;
      tx_buf[2] = tx_item.type;
      tx_buf[3] = (tx_item.len >> 8) & 0xFF;
      tx_buf[4] = (tx_item.len & 0xFF);

      if (hal->config.enable_crc) {
        tx_buf[5] = qspi_crc8(tx_buf, 5);
        memcpy(&tx_buf[QSPI_HEADER_SIZE], tx_item.data, tx_item.len);
        uint16_t pcrc = qspi_crc16_ccitt(tx_item.data, tx_item.len);
        tx_buf[QSPI_HEADER_SIZE + tx_item.len] = (pcrc >> 8) & 0xFF;
        tx_buf[QSPI_HEADER_SIZE + tx_item.len + 1] = pcrc & 0xFF;
      } else {
        tx_buf[5] = 0;
        memcpy(&tx_buf[QSPI_HEADER_SIZE], tx_item.data, tx_item.len);
        tx_buf[QSPI_HEADER_SIZE + tx_item.len] = 0;
        tx_buf[QSPI_HEADER_SIZE + tx_item.len + 1] = 0;
      }

      free(tx_item.data);

      // Assert DR and transmit
      if (hal->config.enable_dr_handshake)
        gpio_set_level(hal->config.gpio_dr, 1);

      esp_err_t ret = qspi_master_transmit(hal, tx_buf, frame_len);

      if (hal->config.enable_dr_handshake)
        gpio_set_level(hal->config.gpio_dr, 0);

      if (ret == ESP_OK) {
        hal->stats.tx_frames++;
        hal->stats.tx_bytes += tx_item.len;
      } else {
        hal->stats.dma_errors++;
        ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(ret));
      }
    }

    // RX: Poll at interval (fix tick comparison)
    TickType_t now = xTaskGetTickCount();
    if ((now - last_rx_poll) >= poll_interval) {
      last_rx_poll = now;

      memset(rx_buf, 0, QSPI_DMA_BUFFER_SIZE);
      esp_err_t ret = qspi_master_receive(hal, rx_buf, QSPI_DMA_BUFFER_SIZE);

      if (ret == ESP_OK) {
        qspi_parse_rx_frame(hal, rx_buf);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);
}

static void qspi_parse_rx_frame(qspi_hal_t *hal, uint8_t *rx_buf) {
  // Validate sync
  if (rx_buf[0] != ((QSPI_FRAME_SYNC >> 8) & 0xFF) ||
      rx_buf[1] != (QSPI_FRAME_SYNC & 0xFF)) {
    return; // Not a valid frame
  }

  qspi_frame_t frame;
  frame.sync = (rx_buf[0] << 8) | rx_buf[1];
  frame.type = rx_buf[2];
  frame.length = (rx_buf[3] << 8) | rx_buf[4];
  frame.header_crc = rx_buf[5];

  // Validate length BEFORE accessing payload
  if (frame.length == 0 ||
      frame.length > QSPI_DMA_BUFFER_SIZE - QSPI_HEADER_SIZE - 2) {
    hal->stats.invalid_length_errors++;
    ESP_LOGW(TAG, "Invalid frame length: %d", frame.length);
    return;
  }

  frame.payload = &rx_buf[QSPI_HEADER_SIZE];
  frame.payload_crc = (rx_buf[QSPI_HEADER_SIZE + frame.length] << 8) |
                      rx_buf[QSPI_HEADER_SIZE + frame.length + 1];

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

static esp_err_t qspi_master_transmit(qspi_hal_t *hal, const uint8_t *data,
                                      size_t len) {
  spi_transaction_ext_t trans = {.base = {.cmd = QSPI_CMD_WRITE,
                                          .flags = SPI_TRANS_MODE_QIO,
                                          .length = len * 8,
                                          .tx_buffer = data,
                                          .rx_buffer = NULL}};

  return spi_device_transmit(hal->spi_dev, (spi_transaction_t *)&trans);
}

static esp_err_t qspi_master_receive(qspi_hal_t *hal, uint8_t *buffer,
                                     size_t len) {
  spi_transaction_ext_t trans = {.base = {.cmd = QSPI_CMD_READ,
                                          .flags = SPI_TRANS_MODE_QIO,
                                          .rxlength = len * 8,
                                          .tx_buffer = NULL,
                                          .rx_buffer = buffer}};

  return spi_device_transmit(hal->spi_dev, (spi_transaction_t *)&trans);
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
