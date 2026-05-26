/**
 * @file rs485_handler.c
 * @brief RS485 Handler Implementation
 */

#include "rs485_handler.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mcu_wan_handler.h"
#include "rs485_comm.h"
#include "bench_latency_lan.h"
#include <string.h>

static const char *TAG = "RS485_HANDLER";

/* ===== Configuration ===== */
#define RS485_HANDLER_TASK_STACK_SIZE (8 * 1024)   // PSRAM stack
#define RS485_HANDLER_TASK_PRIORITY 5
#define RS485_RX_BUFFER_SIZE 512
#define RS485_TX_TIMEOUT_MS 1000
#define RS485_RX_POLL_INTERVAL_MS 20
#define RS485_DOWNLINK_QUEUE_SIZE 10
#define RS485_STATS_LOG_INTERVAL_MS 10000 // 10 seconds
uint32_t g_rs485_baud_rate = RS485_DEFAULT_BAUD_RATE;

/* ===== Statistics ===== */
typedef struct {
  uint32_t tx_ok;
  uint32_t tx_error;
  uint32_t rx_ok;
  uint32_t uplink_forwarded;
  uint32_t uplink_queue_full;
  uint32_t downlink_enqueued;
  uint32_t downlink_dropped;
} rs485_handler_stats_t;

/* ===== Internal Context ===== */
typedef struct {
  uint8_t *data;
  uint16_t len;
} rs485_downlink_msg_t;

static struct {
  TaskHandle_t task_handle;
  StackType_t  *task_stack;   // PSRAM stack buffer
  StaticTask_t *task_tcb;     // internal SRAM TCB
  QueueHandle_t downlink_queue;
  rs485_comm_handle_t comm_handle;
  rs485_handler_stats_t stats;
  bool is_running;
} g_rs485_ctx = {0};

/* ===== Forward Declarations ===== */
static void rs485_handler_task(void *arg);

/* ===== Public API Implementation ===== */

esp_err_t rs485_handler_start(void) {
  if (g_rs485_ctx.is_running) {
    ESP_LOGW(TAG, "RS485 handler already running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Starting RS485 handler");

  // Initialize RS485 communication driver
  rs485_comm_config_t config = {
      .baud_rate = g_rs485_baud_rate,
      .rx_buffer_size = RS485_DEFAULT_RX_BUF_SIZE,
      .tx_buffer_size = RS485_DEFAULT_TX_BUF_SIZE,
  };

  esp_err_t ret = rs485_comm_init(&config, &g_rs485_ctx.comm_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize RS485 comm: %s", esp_err_to_name(ret));
    return ret;
  }

  // Set initial mode to RX
  ret = rs485_comm_set_mode(g_rs485_ctx.comm_handle, RS485_MODE_ONLY_RECEIVE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set RX mode: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "RS485 set to RX mode");

  // Create downlink queue
  g_rs485_ctx.downlink_queue =
      xQueueCreate(RS485_DOWNLINK_QUEUE_SIZE, sizeof(rs485_downlink_msg_t));
  if (!g_rs485_ctx.downlink_queue) {
    ESP_LOGE(TAG, "Failed to create downlink queue");
    return ESP_ERR_NO_MEM;
  }

  // Reset statistics
  memset(&g_rs485_ctx.stats, 0, sizeof(rs485_handler_stats_t));

  // Create handler task (PSRAM stack)
  g_rs485_ctx.task_stack = heap_caps_malloc(RS485_HANDLER_TASK_STACK_SIZE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_rs485_ctx.task_tcb   = heap_caps_malloc(sizeof(StaticTask_t),
                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!g_rs485_ctx.task_stack || !g_rs485_ctx.task_tcb) {
    ESP_LOGE(TAG, "Failed to alloc RS485 task stack/TCB");
    heap_caps_free(g_rs485_ctx.task_stack); g_rs485_ctx.task_stack = NULL;
    heap_caps_free(g_rs485_ctx.task_tcb);   g_rs485_ctx.task_tcb   = NULL;
    vQueueDelete(g_rs485_ctx.downlink_queue);
    return ESP_ERR_NO_MEM;
  }
  /* Set is_running = true BEFORE creating the task.
   * The new task has priority 5 vs Module Monitor priority 3, so FreeRTOS
   * will preempt the caller immediately on xTaskCreateStaticPinnedToCore().
   * If the flag were set after, the task would see is_running=false and exit
   * before the caller ever resumed. */
  g_rs485_ctx.is_running = true;

  g_rs485_ctx.task_handle = xTaskCreateStaticPinnedToCore(
      rs485_handler_task, "rs485_handler",
      RS485_HANDLER_TASK_STACK_SIZE / sizeof(StackType_t),
      NULL, RS485_HANDLER_TASK_PRIORITY,
      g_rs485_ctx.task_stack, g_rs485_ctx.task_tcb, tskNO_AFFINITY);

  if (!g_rs485_ctx.task_handle) {
    ESP_LOGE(TAG, "Failed to create RS485 handler task");
    g_rs485_ctx.is_running = false;
    heap_caps_free(g_rs485_ctx.task_stack); g_rs485_ctx.task_stack = NULL;
    heap_caps_free(g_rs485_ctx.task_tcb);   g_rs485_ctx.task_tcb   = NULL;
    vQueueDelete(g_rs485_ctx.downlink_queue);
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "RS485 handler started successfully");
  return ESP_OK;
}

esp_err_t rs485_handler_stop(void) {
  if (!g_rs485_ctx.is_running) {
    ESP_LOGW(TAG, "RS485 handler not running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping RS485 handler");
  g_rs485_ctx.is_running = false;

  // Delete task
  if (g_rs485_ctx.task_handle) {
    vTaskDelete(g_rs485_ctx.task_handle);
    g_rs485_ctx.task_handle = NULL;
    heap_caps_free(g_rs485_ctx.task_stack); g_rs485_ctx.task_stack = NULL;
    heap_caps_free(g_rs485_ctx.task_tcb);   g_rs485_ctx.task_tcb   = NULL;
  }

  // Delete queue
  if (g_rs485_ctx.downlink_queue) {
    vQueueDelete(g_rs485_ctx.downlink_queue);
    g_rs485_ctx.downlink_queue = NULL;
  }

  ESP_LOGI(TAG, "RS485 handler stopped");
  return ESP_OK;
}

bool rs485_handler_enqueue_downlink(uint8_t *data, uint16_t len) {
  if (!data || len == 0) {
    ESP_LOGW(TAG, "Invalid downlink request: data=%p len=%u", (void *)data,
             len);
    g_rs485_ctx.stats.downlink_dropped++;
    return false;
  }

  if (!g_rs485_ctx.is_running) {
    rs485_gpio_mode_config_t gpio_cfg;
    esp_err_t cfg_ret = rs485_comm_get_gpio_config(&gpio_cfg);
    if (cfg_ret == ESP_OK) {
      ESP_LOGW(TAG,
               "Downlink rejected: handler not running yet (configured stack=%u). "
               "Ensure module_monitor_task started RS485 after JSON apply.",
               gpio_cfg.stack_id);
    } else {
      ESP_LOGW(TAG,
               "Downlink rejected: no RS485 JSON config loaded and handler not running. "
               "Send CFRS:JSON:<stack_id>:<json> first.");
    }
    g_rs485_ctx.stats.downlink_dropped++;
    return false;
  }

  if (!g_rs485_ctx.downlink_queue) {
    ESP_LOGW(TAG, "Downlink rejected: handler queue not created");
    g_rs485_ctx.stats.downlink_dropped++;
    return false;
  }

  // Allocate memory for message
  uint8_t *msg_data = (uint8_t *)malloc(len);
  if (!msg_data) {
    ESP_LOGE(TAG, "Failed to allocate downlink buffer");
    g_rs485_ctx.stats.downlink_dropped++;
    return false;
  }

  memcpy(msg_data, data, len);
  rs485_downlink_msg_t msg = {.data = msg_data, .len = len};

  if (xQueueSend(g_rs485_ctx.downlink_queue, &msg, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Downlink queue full (len=%u)", len);
    free(msg_data);
    g_rs485_ctx.stats.downlink_dropped++;
    return false;
  }

  g_rs485_ctx.stats.downlink_enqueued++;
  ESP_LOGI(TAG, "Enqueued downlink: %u bytes", len);
  return true;
}

/* ===== Internal Task Implementation ===== */

static void rs485_handler_task(void *arg) {
  uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];
  rs485_downlink_msg_t downlink_msg;
  TickType_t last_stats_log = xTaskGetTickCount();

  ESP_LOGI(TAG, "RS485 handler task started");

  while (g_rs485_ctx.is_running) {
    // Handle downlink (WAN -> RS485)
    if (xQueueReceive(g_rs485_ctx.downlink_queue, &downlink_msg, 0) == pdTRUE) {
      // rs485_comm_write automatically handles TX mode switching
      esp_err_t ret =
          rs485_comm_write(g_rs485_ctx.comm_handle, downlink_msg.data,
                           downlink_msg.len, RS485_TX_TIMEOUT_MS);

      if (ret == ESP_OK) {
        g_rs485_ctx.stats.tx_ok++;
        ESP_LOGI(TAG, "Sent downlink: %u bytes", downlink_msg.len);
      } else {
        g_rs485_ctx.stats.tx_error++;
        ESP_LOGE(TAG, "Failed to send downlink");
      }

      free(downlink_msg.data);

      // Explicitly ensure we're back in RX mode after transmission
      ret =
          rs485_comm_set_mode(g_rs485_ctx.comm_handle, RS485_MODE_ONLY_RECEIVE);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to return to RX mode after TX");
      }
    }

    // Handle uplink (RS485 -> WAN)
    size_t available = rs485_comm_available(g_rs485_ctx.comm_handle);
    if (available > 0) {
      size_t to_read =
          (available > RS485_RX_BUFFER_SIZE) ? RS485_RX_BUFFER_SIZE : available;
      size_t actual_read = 0;

      esp_err_t ret = rs485_comm_read(g_rs485_ctx.comm_handle, rx_buffer,
                                      to_read, &actual_read, 100);

      if (ret == ESP_OK && actual_read > 0) {
        g_rs485_ctx.stats.rx_ok++;
        ESP_LOGI(TAG, "Received RS485 data: %u bytes", actual_read);
        ESP_LOG_BUFFER_HEX(TAG, rx_buffer, actual_read);

        // Forward to WAN uplink. When §5 latency bench is enabled, stamp T1
        // here (LAN ingress) and ship as HANDLER_LAT so WAN can compute
        // T2 − T1 right after socket send. Otherwise the RS485 frame goes
        // through its normal production route.
#if BENCH_LATENCY_LAN_ENABLE
        bool ok = bench_latency_lan_send(rx_buffer, (uint16_t)actual_read);
#else
        bool ok = mcu_wan_enqueue_uplink(HANDLER_RS485, rx_buffer, actual_read);
#endif
        if (ok) {
          g_rs485_ctx.stats.uplink_forwarded++;
          ESP_LOGI(TAG, "Forwarded to WAN uplink: %u bytes", actual_read);
        } else {
          g_rs485_ctx.stats.uplink_queue_full++;
          ESP_LOGW(TAG, "WAN uplink queue full");
        }
      }
    }

    // Log statistics every 10 seconds
    TickType_t now = xTaskGetTickCount();
    if ((now - last_stats_log) >= pdMS_TO_TICKS(RS485_STATS_LOG_INTERVAL_MS)) {
      ESP_LOGI(TAG,
               "Stats: TX_OK=%lu TX_ERR=%lu RX_OK=%lu UP_FWD=%lu UP_FULL=%lu "
               "DN_ENQ=%lu DN_DROP=%lu",
               g_rs485_ctx.stats.tx_ok, g_rs485_ctx.stats.tx_error,
               g_rs485_ctx.stats.rx_ok, g_rs485_ctx.stats.uplink_forwarded,
               g_rs485_ctx.stats.uplink_queue_full,
               g_rs485_ctx.stats.downlink_enqueued,
               g_rs485_ctx.stats.downlink_dropped);
      last_stats_log = now;
    }

    vTaskDelay(pdMS_TO_TICKS(RS485_RX_POLL_INTERVAL_MS));
  }

  ESP_LOGI(TAG, "RS485 handler task exiting");
  vTaskDelete(NULL);
}
