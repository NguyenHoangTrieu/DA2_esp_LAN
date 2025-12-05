/**
 * @file lora_tdma_connect.c
 * @brief LoRa TDMA connect task (gateway side)
 *
 * This module glues LoRa TDMA handler <-> WAN MCU handler:
 *
 *  - Uplink flow (sensor -> LoRa -> gateway -> WAN MCU):
 *      * lora_handler_handle_rx() decodes TDMA frame
 *      * our RX callback (lora_tdma_connect_rx_cb) is called
 *      * we build an uplink packet:
 *            [sensor_addr(2)][length(2)][data(length)]
 *        and push it to WAN via mcu_wan_enqueue_uplink(HANDLER_LORA, ...)
 *
 *  - Downlink flow (server -> WAN MCU -> gateway -> LoRa -> sensor):
 *      * dispatch_downlink_to_handler() calls
 *            lora_tdma_connect_enqueue_downlink(data, len)
 *      * data format:
 *            [sensor_addr(2)][length(2)][data(length)]
 *      * this module parses the buffer and sends the payload using
 *            lora_handler_send()
 */

#include "lora_tdma_connect.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mcu_wan_handler.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "LORA_TDMA_CONNECT";

/* ===== Configuration ===== */

#define LORA_TDMA_CONNECT_TASK_STACK_SIZE 4096
#define LORA_TDMA_CONNECT_TASK_PRIORITY 4
#define LORA_TDMA_POLL_DELAY_MS 10
#define LORA_TDMA_STATS_LOG_INTERVAL_MS 30000
#define LORA_TDMA_DOWNLINK_QUEUE_SIZE 20

/* ===== External radio handle (provided by E32 driver) ===== */
/* This handle must be initialized in the radio driver module. */
extern lora_e32_comm_handle_t g_lora_e32_handle;

/* ===== Global context & state ===== */

lora_handler_ctx_t g_lora_tdma_ctx;

typedef struct {
  uint16_t sensor_addr;
  uint16_t data_length;
  uint8_t *data_payload;
} lora_downlink_packet_t;

static TaskHandle_t g_lora_tdma_task = NULL;
static QueueHandle_t g_downlink_queue = NULL;
static bool g_lora_tdma_running = false;
static lora_tdma_connect_stats_t g_stats = {0};

/* ===== Forward declarations ===== */

static void lora_tdma_connect_task(void *pvParameters);
static void lora_tdma_connect_rx_cb(const lora_handler_frame_t *frame);
static void lora_tdma_connect_log_stats(void);

/* ===== Public API ===== */

esp_err_t lora_tdma_connect_start(void) {
  if (g_lora_tdma_running) {
    ESP_LOGW(TAG, "LoRa TDMA connect already running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Starting LoRa TDMA connect");

  /* Initialize TDMA handler context using global config & radio handle */
  lora_handler_init(&g_lora_tdma_ctx, g_lora_e32_handle);

  /* Register RX callback so we can forward to WAN uplink */
  lora_handler_register_rx_callback(&g_lora_tdma_ctx, lora_tdma_connect_rx_cb);

  /* Create downlink queue */
  g_downlink_queue = xQueueCreate(LORA_TDMA_DOWNLINK_QUEUE_SIZE,
                                  sizeof(lora_downlink_packet_t));
  if (g_downlink_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create LoRa downlink queue");
    return ESP_FAIL;
  }

  memset(&g_stats, 0, sizeof(g_stats));

  /* Create processing task */
  BaseType_t ret =
      xTaskCreate(lora_tdma_connect_task, "lora_tdma_connect",
                  LORA_TDMA_CONNECT_TASK_STACK_SIZE, NULL,
                  LORA_TDMA_CONNECT_TASK_PRIORITY, &g_lora_tdma_task);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create LoRa TDMA connect task");
    vQueueDelete(g_downlink_queue);
    g_downlink_queue = NULL;
    return ESP_FAIL;
  }

  g_lora_tdma_running = true;
  ESP_LOGI(TAG, "LoRa TDMA connect started");
  return ESP_OK;
}

esp_err_t lora_tdma_connect_stop(void) {
  if (!g_lora_tdma_running) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping LoRa TDMA connect");
  g_lora_tdma_running = false;

  /* Give the task some time to exit its loop */
  vTaskDelay(pdMS_TO_TICKS(100));

  if (g_lora_tdma_task != NULL) {
    vTaskDelete(g_lora_tdma_task);
    g_lora_tdma_task = NULL;
  }

  /* Clean up downlink queue and free any pending payloads */
  if (g_downlink_queue != NULL) {
    lora_downlink_packet_t pkt;
    while (xQueueReceive(g_downlink_queue, &pkt, 0) == pdTRUE) {
      if (pkt.data_payload) {
        free(pkt.data_payload);
      }
    }
    vQueueDelete(g_downlink_queue);
    g_downlink_queue = NULL;
  }

  ESP_LOGI(TAG, "LoRa TDMA connect stopped");
  return ESP_OK;
}

bool lora_tdma_connect_enqueue_downlink(uint8_t *data, uint16_t len) {
  if (g_downlink_queue == NULL || data == NULL || len < 4) {
    ESP_LOGE(TAG, "Invalid downlink parameters");
    return false;
  }

  /* Parse: [sensor_addr(2)][length(2)][data...] */
  uint16_t sensor_addr = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
  uint16_t payload_len = ((uint16_t)data[2] << 8) | (uint16_t)data[3];

  if (payload_len == 0) {
    ESP_LOGW(TAG, "Downlink payload length is zero");
    return false;
  }

  if (4 + payload_len > len) {
    ESP_LOGW(TAG, "Downlink buffer too short: expected %u, got %u",
             (unsigned)(4 + payload_len), len);
    /* Clamp to available data to avoid overflow */
    payload_len = (len > 4) ? (len - 4) : 0;
  }

  if (payload_len == 0) {
    ESP_LOGW(TAG, "No data after clamping length");
    return false;
  }

  if (payload_len > LORA_HANDLER_MAX_PAYLOAD) {
    ESP_LOGW(TAG, "Downlink payload too large: %u > %u, truncating",
             payload_len, (unsigned)LORA_HANDLER_MAX_PAYLOAD);
    payload_len = LORA_HANDLER_MAX_PAYLOAD;
  }

  lora_downlink_packet_t packet;
  packet.sensor_addr = sensor_addr;
  packet.data_length = payload_len;
  packet.data_payload = (uint8_t *)malloc(payload_len);

  if (packet.data_payload == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for downlink payload");
    return false;
  }

  memcpy(packet.data_payload, data + 4, payload_len);

  if (xQueueSend(g_downlink_queue, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
    free(packet.data_payload);
    g_stats.downlink_dropped++;
    ESP_LOGW(TAG, "LoRa downlink queue full");
    return false;
  }

  g_stats.downlink_enqueued++;
  ESP_LOGD(TAG, "Downlink enqueued: sensor=0x%04X, len=%u", sensor_addr,
           payload_len);
  return true;
}

void lora_tdma_connect_get_stat(lora_tdma_connect_stats_t *out) {
  if (!out) {
    return;
  }

  lora_handler_stats_t core_stats;
  lora_handler_get_stats(&g_lora_tdma_ctx, &core_stats);

  out->tx_ok = core_stats.tx_ok;
  out->rx_ok = core_stats.rx_ok;
  out->rx_error = core_stats.rx_error;
  out->missed_beacon = core_stats.missed_beacon;

  out->uplink_forwarded = g_stats.uplink_forwarded;
  out->uplink_queue_full = g_stats.uplink_queue_full;
  out->downlink_enqueued = g_stats.downlink_enqueued;
  out->downlink_dropped = g_stats.downlink_dropped;
}

/* ===== Internal helpers ===== */

static void lora_tdma_connect_task(void *pvParameters) {
  ESP_LOGI(TAG, "LoRa TDMA connect task started");

  TickType_t last_stats_log = xTaskGetTickCount();

  while (g_lora_tdma_running) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    /* Run TDMA scheduler (no RTOS inside lora_handler) */
    lora_handler_process(&g_lora_tdma_ctx, now_ms);

    /* Process pending downlink packets (one per loop to keep latency low) */
    lora_downlink_packet_t pkt;
    if (xQueueReceive(g_downlink_queue, &pkt, 0) == pdTRUE) {
      uint16_t send_len = pkt.data_length;

      if (send_len > LORA_HANDLER_MAX_PAYLOAD) {
        send_len = LORA_HANDLER_MAX_PAYLOAD;
      }

      bool sent = false;
      if (send_len > 0) {
        sent = lora_handler_send(&g_lora_tdma_ctx, pkt.sensor_addr,
                                 pkt.data_payload, (uint8_t)send_len);
      }

      if (!sent) {
        g_stats.downlink_dropped++;
        ESP_LOGW(TAG,
                 "Failed to queue downlink to LoRa TDMA (addr=0x%04X, len=%u)",
                 pkt.sensor_addr, send_len);
      } else {
        ESP_LOGD(TAG, "Downlink passed to LoRa TDMA (addr=0x%04X, len=%u)",
                 pkt.sensor_addr, send_len);
      }

      free(pkt.data_payload);
    }

    /* Periodic statistics logging */
    TickType_t now_tick = xTaskGetTickCount();
    if ((now_tick - last_stats_log) >=
        pdMS_TO_TICKS(LORA_TDMA_STATS_LOG_INTERVAL_MS)) {
      lora_tdma_connect_log_stats();
      last_stats_log = now_tick;
    }

    vTaskDelay(pdMS_TO_TICKS(LORA_TDMA_POLL_DELAY_MS));
  }

  ESP_LOGI(TAG, "LoRa TDMA connect task exiting");
  vTaskDelete(NULL);
}

static void lora_tdma_connect_rx_cb(const lora_handler_frame_t *frame) {
  if (!frame) {
    return;
  }

  /* Only forward DATA frames; ignore beacons here */
  if (frame->type != LORA_HANDLER_FRAME_DATA || frame->len == 0) {
    return;
  }

  uint16_t sensor_addr = frame->src_id;
  uint16_t payload_len = frame->len;

  if (payload_len > LORA_HANDLER_MAX_PAYLOAD) {
    payload_len = LORA_HANDLER_MAX_PAYLOAD;
  }

  /* Uplink format: [sensor_addr(2)][length(2)][data(length)] */
  uint8_t uplink_buf[4 + LORA_HANDLER_MAX_PAYLOAD];

  uplink_buf[0] = (uint8_t)(sensor_addr >> 8);
  uplink_buf[1] = (uint8_t)(sensor_addr & 0xFF);
  uplink_buf[2] = (uint8_t)(payload_len >> 8);
  uplink_buf[3] = (uint8_t)(payload_len & 0xFF);

  if (payload_len > 0) {
    memcpy(&uplink_buf[4], frame->payload, payload_len);
  }

  if (mcu_wan_enqueue_uplink(HANDLER_LORA, uplink_buf,
                             (uint16_t)(4 + payload_len))) {
    g_stats.uplink_forwarded++;
    ESP_LOGD(TAG, "Forwarded LoRa uplink from sensor 0x%04X (%u bytes)",
             sensor_addr, payload_len);
  } else {
    g_stats.uplink_queue_full++;
    ESP_LOGW(TAG, "WAN uplink queue full (LoRa, sensor=0x%04X)", sensor_addr);
  }
}

static void lora_tdma_connect_log_stats(void) {
  lora_tdma_connect_stats_t s;
  lora_tdma_connect_get_stat(&s);

  ESP_LOGI(TAG, "=== LoRa TDMA connect statistics ===");
  ESP_LOGI(TAG, "LoRa TX_OK: %lu, RX_OK: %lu, RX_ERR: %lu, Missed beacon: %lu",
           s.tx_ok, s.rx_ok, s.rx_error, s.missed_beacon);
  ESP_LOGI(TAG, "Uplink forwarded: %lu, Uplink queue full: %lu",
           s.uplink_forwarded, s.uplink_queue_full);
  ESP_LOGI(TAG, "Downlink enqueued: %lu, Downlink dropped: %lu",
           s.downlink_enqueued, s.downlink_dropped);
  ESP_LOGI(TAG, "====================================");
}
