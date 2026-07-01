#include "esp_log.h"
#include "esp_timer.h"
#include "frame_types.h"
#include "bench_counter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mcu_wan_handler.h"
#include "rs485_handler.h"
#include "stack_handler.h"
#include "storage_handler.h"
#include "wan_comm.h"
#include "bench_throughput.h"
#include "bench_latency_lan.h"  /* §5: BENCH_LATENCY_LAN_ENABLE gate */
#include "esp_heap_caps.h"
#include "clock_sync_lan.h"     /* §5: feed cross-MCU offset from RTC packet */
#include <string.h>

static const char *TAG = "WAN_UL";

/* Set to 1 for once-per-second [UL-DBG ...] timing breakdown of the
 * dispatcher iter (mutex / qrx / build / send / pflush / sdcheck). */
#define WAN_UL_DBG_INSTRUMENTATION 0

// CONFIGURATION

#define UPLINK_TASK_STACK_SIZE 1024 * 16
#define UPLINK_TASK_PRIORITY 5 // Lower than downlink
/* Queue item is ~2 KB inline. We keep control block in internal RAM but move
 * queue storage to PSRAM via xQueueCreateStatic, allowing a deeper queue
 * without exhausting internal heap during boot. */
#define UPLINK_QUEUE_SIZE 32
#define UPLINK_QUEUE_SEND_WAIT_MS 20
#define MAX_PAYLOAD_SIZE INTER_MCU_PAYLOAD_MAX_LEN
#define ACK_TIMEOUT_MS 2000  /* STM32 forwards DT to ThingsBoard via MQTT before ACKing; 200ms was too short */
#define UPLINK_BURST_COUNT 8      /* Max queue items drained per loop iteration */
#define UPLINK_ACK_TIGHT_POLLS 32 /* DQ polls per ACK without yield (covers ~400µs WAN prep window) */
#define RTC_REQUEST_INTERVAL_MS 1000
#define MAX_RETRY_COUNT 3
#define HANDSHAKE_INTERVAL_MS 1000
#define HANDSHAKE_TIMEOUT_MS 100
#define MUTEX_TIMEOUT_MS 50
#define SD_RETRY_DELAY_MS 2000  // Delay between SD retryattempts
#define MAX_FILE_RETRY_ATTEMPTS 3  // Max retry per file before delete

// FW Version
// FW Version macros removed (defined in frame_types.h and mcu_wan_handler.h)

// TYPES

typedef struct {
  handler_id_t source_id;
  uint8_t data[MAX_PAYLOAD_SIZE];
  uint16_t length;
  char rtc_timestamp[20];
  uplink_route_t route;
} uplink_item_t;

typedef struct {
  char rtc_string[20];
  bool valid;
} rtc_cache_t;

// EXTERNAL REFERENCES

extern wan_comm_handle_t g_wan_handle;
extern SemaphoreHandle_t g_qspi_mutex;
extern SemaphoreHandle_t g_rtc_mutex;
extern volatile bool g_handshake_done;
extern bool g_handler_running;

// MODULE STATE

static QueueHandle_t g_uplink_queue = NULL;
static StaticQueue_t *g_uplink_queue_tcb = NULL;
static uint8_t *g_uplink_queue_storage = NULL;
static TaskHandle_t g_uplink_task_handle = NULL;
static StackType_t *g_uplink_stack = NULL;
static StaticTask_t *g_uplink_tcb = NULL;
static internet_status_t g_internet_status = INTERNET_STATUS_OFFLINE;
static rtc_cache_t g_rtc_cache = {.rtc_string = {0}, .valid = false};
static uint32_t g_cached_wan_fw_version = 0;

// Statistics
static uint32_t g_uplink_sent_count = 0;
static uint32_t g_uplink_fail_count = 0;
static uint32_t g_sd_backup_count = 0;
static uint32_t g_sd_retry_success_count = 0;
static uint32_t g_uplink_queue_drop_count = 0;
static uint32_t g_uplink_queue_max_depth = 0;

/* Shared with downlink task: tick when last CF command was dispatched.
 * Uplink task suppresses SD retry for CF_SD_SUPPRESS_MS after each CF dispatch
 * to prevent stale SD packets from being returned as the CF RPC response. */
volatile TickType_t g_last_cf_dispatch_tick = 0;
#define CF_SD_SUPPRESS_MS 3000

// FORWARD DECLARATIONS

static void uplink_handler_task(void *pvParameters);
static esp_err_t perform_handshake(void);
static esp_err_t request_rtc_and_status(void);
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out, uint32_t ack_timeout_ms);
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len);
// Downlink
esp_err_t mcu_wan_handler_start_downlink_task(void);
void mcu_wan_handler_stop_downlink_task(void);

// PUBLIC API

esp_err_t mcu_wan_handler_start_uplink_task(void) {
  if (g_uplink_task_handle != NULL) {
    ESP_LOGW(TAG, "Uplink task already running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "============================================");
  ESP_LOGI(TAG, "Starting Uplink Handler Task (Priority %d)",
           UPLINK_TASK_PRIORITY);
  ESP_LOGI(TAG, "============================================");

  // Create uplink queue (storage in PSRAM, control block in internal RAM)
  size_t q_bytes = UPLINK_QUEUE_SIZE * sizeof(uplink_item_t);
  g_uplink_queue_storage = (uint8_t *)heap_caps_malloc(q_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_uplink_queue_tcb = (StaticQueue_t *)heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (g_uplink_queue_storage && g_uplink_queue_tcb) {
    g_uplink_queue = xQueueCreateStatic(
        UPLINK_QUEUE_SIZE,
        sizeof(uplink_item_t),
        g_uplink_queue_storage,
        g_uplink_queue_tcb);
  }

  if (!g_uplink_queue) {
    if (g_uplink_queue_storage) {
      heap_caps_free(g_uplink_queue_storage);
      g_uplink_queue_storage = NULL;
    }
    if (g_uplink_queue_tcb) {
      heap_caps_free(g_uplink_queue_tcb);
      g_uplink_queue_tcb = NULL;
    }
    ESP_LOGE(TAG, "Failed to create uplink queue");
    return ESP_FAIL;
  }

  g_uplink_queue_drop_count = 0;
  g_uplink_queue_max_depth = 0;

  // Stack shifted to PSRAM
  g_uplink_stack = (StackType_t *)heap_caps_malloc(UPLINK_TASK_STACK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_uplink_tcb = (StaticTask_t *)heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (!g_uplink_stack || !g_uplink_tcb) {
      ESP_LOGE(TAG, "Failed to allocate memory for uplink task");
      if (g_uplink_stack) heap_caps_free(g_uplink_stack);
      if (g_uplink_tcb) heap_caps_free(g_uplink_tcb);
      vQueueDelete(g_uplink_queue);
      g_uplink_queue = NULL;
      if (g_uplink_queue_storage) {
        heap_caps_free(g_uplink_queue_storage);
        g_uplink_queue_storage = NULL;
      }
      if (g_uplink_queue_tcb) {
        heap_caps_free(g_uplink_queue_tcb);
        g_uplink_queue_tcb = NULL;
      }
      return ESP_FAIL;
  }

  g_uplink_task_handle = xTaskCreateStatic(uplink_handler_task, "wan_uplink", UPLINK_TASK_STACK_SIZE / sizeof(StackType_t),
                                           NULL, UPLINK_TASK_PRIORITY, g_uplink_stack, g_uplink_tcb);

  if (g_uplink_task_handle == NULL) {
    ESP_LOGE(TAG, "Failed to create uplink task");
    heap_caps_free(g_uplink_stack);
    heap_caps_free(g_uplink_tcb);
    vQueueDelete(g_uplink_queue);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Uplink task started");
  ESP_LOGI(TAG, "LAN FW: v%u.%u.%u.%u", LAN_FW_VERSION_MAJOR,
           LAN_FW_VERSION_MINOR, LAN_FW_VERSION_PATCH, LAN_FW_VERSION_BUILD);

  return ESP_OK;
}

void mcu_wan_handler_stop_uplink_task(void) {
  if (g_uplink_task_handle) {
    vTaskDelete(g_uplink_task_handle);
    g_uplink_task_handle = NULL;
    if (g_uplink_stack) heap_caps_free(g_uplink_stack);
    if (g_uplink_tcb) heap_caps_free(g_uplink_tcb);
    g_uplink_stack = NULL;
    g_uplink_tcb = NULL;

    ESP_LOGI(TAG, "Uplink task stopped");
    ESP_LOGI(TAG,
             "Statistics: TX=%lu, FAIL=%lu, SD_BACKUP=%lu, SD_RETRY_OK=%lu, Q_DROP=%lu, Q_MAX=%lu/%d",
             g_uplink_sent_count, g_uplink_fail_count, g_sd_backup_count,
             g_sd_retry_success_count,
             g_uplink_queue_drop_count, g_uplink_queue_max_depth, UPLINK_QUEUE_SIZE);
  }

  if (g_uplink_queue) {
    vQueueDelete(g_uplink_queue);
    g_uplink_queue = NULL;
  }
  if (g_uplink_queue_storage) {
    heap_caps_free(g_uplink_queue_storage);
    g_uplink_queue_storage = NULL;
  }
  if (g_uplink_queue_tcb) {
    heap_caps_free(g_uplink_queue_tcb);
    g_uplink_queue_tcb = NULL;
  }
}

static bool enqueue_uplink_internal_to(handler_id_t source_id, const uint8_t *data,
                                       uint16_t len, uplink_route_t route,
                                       TickType_t wait_ticks) {
  if (!g_uplink_queue || !data || len == 0) {
    ESP_LOGE(TAG, "Invalid uplink parameters");
    return false;
  }

  if (len > MAX_PAYLOAD_SIZE) {
    ESP_LOGE(TAG, "Uplink data too large: %u > %d", len, MAX_PAYLOAD_SIZE);
    return false;
  }

  /* Non-blocking caller: skip the 2KB memcpy if queue is full. */
  if (wait_ticks == 0 && uxQueueSpacesAvailable(g_uplink_queue) == 0) {
    g_uplink_queue_drop_count++;
    return false;
  }

  uplink_item_t item;
  item.source_id = source_id;
  item.length = len;
  item.route = route;
  memcpy(item.data, data, len);

  // Attach current RTC timestamp
  if (xSemaphoreTake(g_rtc_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (g_rtc_cache.valid) {
      strncpy(item.rtc_timestamp, g_rtc_cache.rtc_string, 20);
    } else {
      strcpy(item.rtc_timestamp, "00/00/0000-00:00:00");
    }
    xSemaphoreGive(g_rtc_mutex);
  }

  if (xQueueSend(g_uplink_queue, &item, wait_ticks) != pdTRUE) {
    g_uplink_queue_drop_count++;
#if !BENCH_QUIET_LOG
    ESP_LOGW(TAG, "Uplink queue full");
#endif
    return false;
  }

  {
    UBaseType_t q_now = uxQueueMessagesWaiting(g_uplink_queue);
    if ((uint32_t)q_now > g_uplink_queue_max_depth) {
      g_uplink_queue_max_depth = (uint32_t)q_now;
    }
  }

#if !BENCH_QUIET_LOG
  ESP_LOGI(TAG, "Uplink queued from handler %s: %u bytes (route=%s)",
           handler_id_to_string(source_id), len,
           (route == UPLINK_ROUTE_LOCAL) ? "LOCAL" : "CLOUD");
#endif
  return true;
}

bool mcu_wan_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                            uint16_t len) {
  return enqueue_uplink_internal_to(source_id, data, len, UPLINK_ROUTE_CLOUD,
                                    pdMS_TO_TICKS(UPLINK_QUEUE_SEND_WAIT_MS));
}

bool mcu_wan_enqueue_uplink_local(handler_id_t source_id, uint8_t *data,
                                  uint16_t len) {
  return enqueue_uplink_internal_to(source_id, data, len, UPLINK_ROUTE_LOCAL,
                                    pdMS_TO_TICKS(UPLINK_QUEUE_SEND_WAIT_MS));
}

/* Non-blocking variant: returns immediately on full queue. */
bool mcu_wan_try_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                                uint16_t len) {
  return enqueue_uplink_internal_to(source_id, data, len, UPLINK_ROUTE_CLOUD,
                                    0 /* no wait */);
}

internet_status_t mcu_wan_handler_get_internet_status(void) {
  return g_internet_status;
}

esp_err_t mcu_wan_handler_get_rtc(char *buffer) {
  if (!buffer) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(g_rtc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (g_rtc_cache.valid) {
      strncpy(buffer, g_rtc_cache.rtc_string, 20);
      xSemaphoreGive(g_rtc_mutex);
      return ESP_OK;
    }
    xSemaphoreGive(g_rtc_mutex);
  }

  return ESP_ERR_NOT_FOUND;
}

uint32_t mcu_wan_handler_get_wan_fw_version(void) {
  return g_cached_wan_fw_version;
}

// UPLINK HANDLER TASK

/**
 * @brief Uplink task main loop
 *
 * Phase 1: Handshake (blocking until success)
 * Phase 2: Loop {
 *   - Process uplink queue
 *   - Retry SD card data
 *   - Update RTC/Internet status (1s interval)
 * }
 */
static void uplink_handler_task(void *pvParameters) {
  ESP_LOGI(TAG, "============================================");
  ESP_LOGI(TAG, "Uplink Handler Task (Priority %d)", UPLINK_TASK_PRIORITY);
  ESP_LOGI(TAG, "============================================");

  // PHASE 1: Handshake Loop

  ESP_LOGI(TAG, "Phase 1: Handshake with WAN MCU");

  while (g_handler_running && !g_handshake_done) {
    // Take SPI mutex for handshake
    if (xSemaphoreTake(g_qspi_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (perform_handshake() == ESP_OK) {
        g_handshake_done = true;
        ESP_LOGI(TAG, "Handshake successful!");
        xSemaphoreGive(g_qspi_mutex);
        break;
      }
      xSemaphoreGive(g_qspi_mutex);
    }

    ESP_LOGW(TAG, "Handshake failed, retrying in %dms", HANDSHAKE_INTERVAL_MS);
    vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_INTERVAL_MS));
  }

  ESP_LOGI(TAG, "Module handlers managed by Module Monitor Task");

  // PHASE 2: Uplink/RTC/SD Loop

  ESP_LOGI(TAG, "Phase 2: Uplink processing loop");

  TickType_t last_rtc_request = xTaskGetTickCount();
  TickType_t last_flush = xTaskGetTickCount();
  TickType_t last_sd_retry_attempt = 0;  // Track last SD retry to avoid spam
  uint8_t consecutive_sd_failures = 0;   // Track consecutive failures for same file

#if WAN_UL_DBG_INSTRUMENTATION
  /* Per-second timing breakdown sums. Divide by iters/items at report. */
  uint64_t dbg_iters         = 0;  /* total dispatcher iterations              */
  uint64_t dbg_iter_us_total = 0;  /* wall time inside iter (excluding delay) */
  uint64_t dbg_bursts        = 0;  /* iters that drained >=1 item              */
  uint64_t dbg_items         = 0;  /* items dispatched (bench + production)    */
  uint64_t dbg_mutex_wait_us = 0;  /* total wait on initial g_qspi_mutex take */
  uint64_t dbg_send_us_total = 0;  /* total wall time inside wan_comm_send_data */
  uint64_t dbg_pflush_count  = 0;  /* how many periodic flushes fired          */
  uint64_t dbg_pflush_us     = 0;  /* total wall time inside periodic flush    */
  uint64_t dbg_retake_fail   = 0;  /* bench re-take of g_qspi_mutex failed     */
  uint64_t dbg_qrx_us_total  = 0;  /* total time inside xQueueReceive          */
  uint64_t dbg_qrx_count     = 0;  /* number of xQueueReceive calls            */
  uint64_t dbg_build_us_total= 0;  /* total time inside build_data_packet      */
  uint64_t dbg_gap_us_total  = 0;  /* time between bench retake and next item  */
  uint64_t dbg_postburst_us  = 0;  /* time from burst end to mutex give        */
  uint64_t dbg_sdcheck_us    = 0;  /* time inside storage_handler_has_data + SD block */
  int64_t  dbg_report_t0     = esp_timer_get_time();
  int64_t  dbg_last_unaccounted_t = 0;  /* for tracking gaps between events */
#endif
  uplink_item_t uplink_item;
  bool had_work = false;  /* set each iteration for adaptive vTaskDelay */

  while (g_handler_running) {

    TickType_t now = xTaskGetTickCount();
#if WAN_UL_DBG_INSTRUMENTATION
    int64_t  dbg_iter_start = esp_timer_get_time();
    uint32_t dbg_items_this_iter = 0;
    dbg_iters++;
#endif

    // Try to acquire SPI mutex (non-blocking / short timeout)
    // If downlink task has it, we'll skip and try next iteration
#if WAN_UL_DBG_INSTRUMENTATION
    int64_t dbg_mwait_t0 = esp_timer_get_time();
#endif
    if (xSemaphoreTake(g_qspi_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) ==
        pdTRUE) {
#if WAN_UL_DBG_INSTRUMENTATION
      dbg_mutex_wait_us += (uint64_t)(esp_timer_get_time() - dbg_mwait_t0);
#endif

      // A) Check Uplink Queue — burst up to UPLINK_BURST_COUNT items
      int burst_count = 0;
      while (burst_count < UPLINK_BURST_COUNT) {
#if WAN_UL_DBG_INSTRUMENTATION
        int64_t dbg_qrx_t0 = esp_timer_get_time();
#endif
        bool dbg_qrx_ok = (xQueueReceive(g_uplink_queue, &uplink_item, 0) == pdTRUE);
#if WAN_UL_DBG_INSTRUMENTATION
        dbg_qrx_us_total += (uint64_t)(esp_timer_get_time() - dbg_qrx_t0);
        dbg_qrx_count++;
#endif
        if (!dbg_qrx_ok) break;

#if !BENCH_QUIET_LOG
        ESP_LOGI(TAG, "Processing uplink from handler %s: %u bytes",
                 handler_id_to_string(uplink_item.source_id),
                 uplink_item.length);
#endif

        uint8_t packet[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
        uint16_t packet_len = 0;
#if WAN_UL_DBG_INSTRUMENTATION
        int64_t dbg_build_t0 = esp_timer_get_time();
#endif
        build_data_packet(&uplink_item, packet, &packet_len);
#if WAN_UL_DBG_INSTRUMENTATION
        dbg_build_us_total += (uint64_t)(esp_timer_get_time() - dbg_build_t0);
#endif

        // Routing policy (3 paths):
        //  1. HANDLER_BENCH      : throughput test, fire-and-forget, no SD.
        //  2. UPLINK_ROUTE_LOCAL : response to a CF command from the config
        //                          app (UART/USB/Web). Must reach WAN MCU
        //                          immediately so it can correlate with the
        //                          recent CF source and route back to the
        //                          originating channel. NEVER persisted —
        //                          a stale local response replayed from SD
        //                          would route to MQTT (app already timed out).
        //  3. UPLINK_ROUTE_CLOUD : node telemetry. Online → send + ACK gate.
        //                          Offline → SD backup, replay when online.
        if (uplink_item.source_id == HANDLER_BENCH) {
          /* Bench bypass: release g_qspi_mutex around the send so downlink
           * can interleave; wan_comm_send_data has its own transfer_mutex. */
          xSemaphoreGive(g_qspi_mutex);

#if WAN_UL_DBG_INSTRUMENTATION
          int64_t dbg_send_t0 = esp_timer_get_time();
#endif
#if BENCH_TP_MODE_PROD_REAL
          wan_comm_status_t st = wan_comm_send_data(g_wan_handle,
                                                     packet, packet_len);
          if (st == WAN_COMM_OK) {
            bench_throughput_count_tx((uint32_t)uplink_item.length);
          } else {
            bench_throughput_count_tx_drop();
          }
#else
          (void)wan_comm_send_data(g_wan_handle, packet, packet_len);
#endif

#if WAN_UL_DBG_INSTRUMENTATION
          dbg_send_us_total += (uint64_t)(esp_timer_get_time() - dbg_send_t0);
          dbg_items++;
          dbg_items_this_iter++;
#endif

          /* Re-take. If contended, bail out without holding the mutex. */
          burst_count++;
#if WAN_UL_DBG_INSTRUMENTATION
          int64_t dbg_retake_t0 = esp_timer_get_time();
#endif
          if (xSemaphoreTake(g_qspi_mutex,
                             pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) != pdTRUE) {
#if WAN_UL_DBG_INSTRUMENTATION
            dbg_mutex_wait_us += (uint64_t)(esp_timer_get_time() - dbg_retake_t0);
            dbg_retake_fail++;
#endif
            had_work = true;
            goto bench_bailout_no_mutex;
          }
#if WAN_UL_DBG_INSTRUMENTATION
          dbg_mutex_wait_us += (uint64_t)(esp_timer_get_time() - dbg_retake_t0);
#endif
          continue;
        } else if (uplink_item.route == UPLINK_ROUTE_LOCAL) {
          ack_type_t ack_result;
          esp_err_t send_result =
              send_data_to_wan(packet, packet_len, &ack_result, ACK_TIMEOUT_MS);
          if (send_result == ESP_OK) {
            g_uplink_sent_count++;
#if !BENCH_QUIET_LOG
            ESP_LOGI(TAG, "Local response forwarded (#%lu)",
                     g_uplink_sent_count);
#endif
          } else {
            /* Drop on failure — local responses are time-sensitive; SD
             * persistence would cause WAN to misroute the stale reply. */
            g_uplink_fail_count++;
            ESP_LOGW(TAG, "Local response dropped after retries (handler=%s)",
                     handler_id_to_string(uplink_item.source_id));
          }
#if BENCH_LATENCY_LAN_ENABLE
        } else if (uplink_item.source_id == HANDLER_LAT) {
          /* Bench latency (§5) bypass: always push to WAN regardless of
           * internet status. The bench measures internal latency only —
           * even if LTE/WiFi/Eth can't reach the server, we still want the
           * packet to cross the SPI bridge so WAN can stamp T2 and log it.
           * NEVER persist to SD (stale benchmark data is useless). */
          ack_type_t ack_result;
          esp_err_t send_result =
              send_data_to_wan(packet, packet_len, &ack_result, ACK_TIMEOUT_MS);
          if (send_result == ESP_OK) {
            g_uplink_sent_count++;
          } else {
            g_uplink_fail_count++;
            ESP_LOGW(TAG, "Bench LAT send failed (no SD fallback)");
          }
#endif /* BENCH_LATENCY_LAN_ENABLE */
        } else if (g_internet_status == INTERNET_STATUS_ONLINE) {
          ack_type_t ack_result;
          esp_err_t send_result =
              send_data_to_wan(packet, packet_len, &ack_result, ACK_TIMEOUT_MS);

          if (send_result == ESP_OK) {
            if (ack_result == ACK_TYPE_INTERNET_OK) {
              g_uplink_sent_count++;
#if !BENCH_QUIET_LOG
              ESP_LOGI(TAG, "Uplink sent successfully (#%lu)",
                       g_uplink_sent_count);
#endif
            } else if (ack_result == ACK_TYPE_NO_INTERNET) {
              ESP_LOGW(TAG, "ACK received but NO_INTERNET, saving to SD");
              g_internet_status = INTERNET_STATUS_OFFLINE;
              storage_handler_save(packet, packet_len);
              g_sd_backup_count++;
            }
          } else {
            ESP_LOGW(TAG, "Send failed after retries, saving to SD");
            storage_handler_save(packet, packet_len);
            g_uplink_fail_count++;
            g_sd_backup_count++;
            /* Delay SD retry so batch buffer has time to flush to a real file */
            last_sd_retry_attempt = xTaskGetTickCount();
          }
        } else {
          ESP_LOGW(TAG, "Internet offline, saving to SD card");
          storage_handler_save(packet, packet_len);
          g_sd_backup_count++;
        }
        burst_count++;
      }  /* end burst while */
      had_work = (burst_count > 0);

#if WAN_UL_DBG_INSTRUMENTATION
      int64_t dbg_postburst_t0 = esp_timer_get_time();
#endif

      {
        UBaseType_t q_now = uxQueueMessagesWaiting(g_uplink_queue);
        if ((uint32_t)q_now > g_uplink_queue_max_depth) {
          g_uplink_queue_max_depth = (uint32_t)q_now;
        }
      }

      // B) SD Card Backup Retry. has_data() is O(1) (cached counter).
#if WAN_UL_DBG_INSTRUMENTATION
      int64_t dbg_sdcheck_t0 = esp_timer_get_time();
#endif
      bool dbg_sd_has = storage_handler_has_data();
#if WAN_UL_DBG_INSTRUMENTATION
      dbg_sdcheck_us += (uint64_t)(esp_timer_get_time() - dbg_sdcheck_t0);
#endif

      if (dbg_sd_has && g_internet_status == INTERNET_STATUS_ONLINE) {

        // Rate limiting: Only retry every SD_RETRY_DELAY_MS
        if ((now - last_sd_retry_attempt) < pdMS_TO_TICKS(SD_RETRY_DELAY_MS)) {
          // Skip this iteration - wait for delay period
          goto skip_sd_retry;
        }

        // Suppress SD retry for CF_SD_SUPPRESS_MS after a CF command was
        // dispatched to give the BLE stack time to complete the operation and
        // queue the real uplink response before SD stale data is sent.
        if (g_last_cf_dispatch_tick != 0 &&
            (now - g_last_cf_dispatch_tick) < pdMS_TO_TICKS(CF_SD_SUPPRESS_MS)) {
          ESP_LOGD(TAG, "SD retry suppressed — CF dispatched %lums ago",
                   (unsigned long)((now - g_last_cf_dispatch_tick) * portTICK_PERIOD_MS));
          goto skip_sd_retry;
        }

        last_sd_retry_attempt = now;

        // Prepare retry session (open oldest file)
        if (storage_handler_prepare_retry() == ESP_OK) {
          ESP_LOGI(TAG, "Starting SD card retry session");

          bool session_success = true;
          uint8_t sd_buffer[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
          uint16_t sd_length = 0; // Changed to uint16_t to match new API
          uint16_t packets_sent = 0;

          // Process all packets in the file stream
          while (storage_handler_get_next_packet(sd_buffer, &sd_length,
                                                 sizeof(sd_buffer)) == ESP_OK) {

            // ===== CORRUPT FILE DETECTION =====
            // Minimum valid packet: [handler(3)][len(2)][rtc(19)] = 24 bytes
            if (sd_length < 24) {
              ESP_LOGE(TAG, "Corrupt packet detected: only %u bytes (min 24)", sd_length);
              ESP_LOGE(TAG, "Aborting and DELETING corrupt file");
              session_success = false;  // Mark as failed
              consecutive_sd_failures++;  // Increment failure counter
              break;  // Abort immediately
            }

            ESP_LOGI(TAG, "Retrying SD packet: %u bytes", sd_length);

            ack_type_t ack_result =
                ACK_TYPE_TIMEOUT; // Initialize to avoid garbage
            esp_err_t send_result =
                send_data_to_wan(sd_buffer, sd_length, &ack_result, ACK_TIMEOUT_MS);

            if (send_result == ESP_OK && ack_result == ACK_TYPE_INTERNET_OK) {
              g_sd_retry_success_count++;
              packets_sent++;
              ESP_LOGI(TAG, "SD packet sent OK (#%lu)",
                       g_sd_retry_success_count);
              // Continue to next packet
            } else {
              ESP_LOGW(TAG,
                       "SD packet send failed/timeout, aborting retry session");
              session_success = false;
              consecutive_sd_failures++;

              if (ack_result == ACK_TYPE_NO_INTERNET) {
                g_internet_status = INTERNET_STATUS_OFFLINE;
              }
              break; // Stop processing this file, retry later
            }
          }

          // ===== FORCED DELETE AFTER MAX ATTEMPTS =====
          if (!session_success && consecutive_sd_failures >= MAX_FILE_RETRY_ATTEMPTS) {
            ESP_LOGE(TAG, "File failed %u times consecutively - FORCE DELETING",
                     consecutive_sd_failures);
            storage_handler_finish_retry(true);  // Force delete (true = success)
            consecutive_sd_failures = 0;  // Reset counter for next file
          } else {
            storage_handler_finish_retry(session_success);
            if (session_success) {
              consecutive_sd_failures = 0;  // Reset on success
            }
          }
        }
      }
      
skip_sd_retry:

      // C) RTC Periodic Timer (1 second interval)

      if ((now - last_rtc_request) >= pdMS_TO_TICKS(RTC_REQUEST_INTERVAL_MS)) {
        if (request_rtc_and_status() == ESP_OK) {
          ESP_LOGD(TAG, "RTC and Internet status updated");
        }
        last_rtc_request = now;
      }

#if WAN_UL_DBG_INSTRUMENTATION
      dbg_postburst_us += (uint64_t)(esp_timer_get_time() - dbg_postburst_t0);
#endif

      // Release SPI mutex
      xSemaphoreGive(g_qspi_mutex);
    }

bench_bailout_no_mutex:
    // D) Periodic Flush (outside SPI mutex)

    if ((now - last_flush) >= pdMS_TO_TICKS(INTER_MCU_BATCH_INTERVAL_MS)) {
#if WAN_UL_DBG_INSTRUMENTATION
      int64_t dbg_pf_t0 = esp_timer_get_time();
#endif
      storage_handler_flush();
      wan_comm_flush_dma_buffer(g_wan_handle);
      last_flush = now;
#if WAN_UL_DBG_INSTRUMENTATION
      dbg_pflush_count++;
      dbg_pflush_us += (uint64_t)(esp_timer_get_time() - dbg_pf_t0);
#endif
    }

#if WAN_UL_DBG_INSTRUMENTATION
    /* Account iter wall time and emit a once-per-second breakdown. */
    if (dbg_items_this_iter > 0) {
      dbg_bursts++;
    }
    dbg_iter_us_total += (uint64_t)(esp_timer_get_time() - dbg_iter_start);

    int64_t dbg_now_us = esp_timer_get_time();
    if (dbg_now_us - dbg_report_t0 >= 1000000) {
      uint64_t window_us = (uint64_t)(dbg_now_us - dbg_report_t0);
      uint64_t accounted_us = dbg_send_us_total + dbg_qrx_us_total +
                              dbg_build_us_total + dbg_mutex_wait_us +
                              dbg_pflush_us + dbg_postburst_us;
      ESP_LOGI(TAG,
        "[UL-DBG %llums] iters=%llu items=%llu items/s=%.1f "
        "iter_avg=%lluus | qrx_avg=%lluus(n=%llu) build_avg=%lluus "
        "send_avg=%lluus mutex_avg=%lluus pflush_avg=%lluus(n=%llu) "
        "postburst_avg=%lluus sdcheck_avg=%lluus "
        "| accounted=%lluus/iter unaccounted=%lluus/iter retake_fail=%llu",
        (unsigned long long)(window_us / 1000ull),
        (unsigned long long)dbg_iters,
        (unsigned long long)dbg_items,
        (double)dbg_items * 1e6 / (double)window_us,
        (unsigned long long)(dbg_iters ? dbg_iter_us_total / dbg_iters : 0),
        (unsigned long long)(dbg_qrx_count ? dbg_qrx_us_total / dbg_qrx_count : 0),
        (unsigned long long)dbg_qrx_count,
        (unsigned long long)(dbg_items ? dbg_build_us_total / dbg_items : 0),
        (unsigned long long)(dbg_items ? dbg_send_us_total / dbg_items : 0),
        (unsigned long long)(dbg_iters ? dbg_mutex_wait_us / dbg_iters : 0),
        (unsigned long long)(dbg_pflush_count ? dbg_pflush_us / dbg_pflush_count : 0),
        (unsigned long long)dbg_pflush_count,
        (unsigned long long)(dbg_iters ? dbg_postburst_us / dbg_iters : 0),
        (unsigned long long)(dbg_iters ? dbg_sdcheck_us / dbg_iters : 0),
        (unsigned long long)(dbg_iters ? accounted_us / dbg_iters : 0),
        (unsigned long long)(dbg_iters ? (dbg_iter_us_total - accounted_us) / dbg_iters : 0),
        (unsigned long long)dbg_retake_fail);
      dbg_iters = dbg_iter_us_total = dbg_bursts = dbg_items = 0;
      dbg_mutex_wait_us = dbg_send_us_total = 0;
      dbg_pflush_count = dbg_pflush_us = 0;
      dbg_retake_fail = 0;
      dbg_qrx_us_total = dbg_qrx_count = dbg_build_us_total = dbg_gap_us_total = 0;
      dbg_postburst_us = dbg_sdcheck_us = 0;
      dbg_report_t0 = dbg_now_us;
    }
#endif /* WAN_UL_DBG_INSTRUMENTATION */

    // Yield when queue active, 10ms sleep when idle (tick-rate granularity).
    if (had_work) {
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  ESP_LOGI(TAG, "Uplink Handler Task exiting");
  vTaskDelete(NULL);
}

// INTERNAL FUNCTIONS

/**
 * @brief Perform handshake with WAN MCU
 * NOTE: Caller must hold g_qspi_mutex
 *
 * Request:  [CF][0x01][fw_version(4)]
 * Response: [ACK][0x10][internet_flag][wan_fw_version(4)]
 */
static esp_err_t perform_handshake(void) {
  uint8_t handshake_req[5];
  handshake_req[0] = 0x01; // Handshake subtype
  handshake_req[1] = (LAN_FW_VERSION >> 24) & 0xFF;
  handshake_req[2] = (LAN_FW_VERSION >> 16) & 0xFF;
  handshake_req[3] = (LAN_FW_VERSION >> 8) & 0xFF;
  handshake_req[4] = LAN_FW_VERSION & 0xFF;

  ESP_LOGI(TAG, "Sending handshake: LAN FW v%u.%u.%u.%u", LAN_FW_VERSION_MAJOR,
           LAN_FW_VERSION_MINOR, LAN_FW_VERSION_PATCH, LAN_FW_VERSION_BUILD);
  
  // Debug: Log handshake payload (CF header will be added by wan_comm_send_command)
  ESP_LOGI(TAG, "Handshake payload (5 bytes):");
  ESP_LOG_BUFFER_HEXDUMP(TAG, handshake_req, sizeof(handshake_req), ESP_LOG_INFO);

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, handshake_req, sizeof(handshake_req));
  if (status != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to send handshake request");
    return ESP_FAIL;
  }

  // Ensure command is transmitted immediately before waiting for response
  if (wan_comm_flush_dma_buffer(g_wan_handle) != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to flush handshake request");
    return ESP_FAIL;
  }

  // Wait for ACK response from WAN MCU
  vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS));

  uint8_t response[16] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  // Debug: Log raw response data
  ESP_LOGI(TAG, "Handshake response (%d bytes):", sizeof(response));
  ESP_LOG_BUFFER_HEXDUMP(TAG, response, sizeof(response), ESP_LOG_INFO);

  if (status == WAN_COMM_OK && response[0] == 0x02 &&
      response[1] == ACK_TYPE_HANDSHAKE) {
    // Extract internet status and WAN FW version
    g_internet_status = (internet_status_t)response[2];
    g_cached_wan_fw_version =
        ((uint32_t)response[3] << 24) | ((uint32_t)response[4] << 16) |
        ((uint32_t)response[5] << 8) | ((uint32_t)response[6]);

    return ESP_OK;
  }

  ESP_LOGE(TAG, "Invalid handshake response");
  return ESP_FAIL;
}

/**
 * @brief Request RTC and Internet status from WAN MCU
 * NOTE: Caller must hold g_qspi_mutex
 *
 * Request:  [CF][R][T]
 * Response: [R][T][dd/mm/yyyy-hh:mm:ss][status]
 */
static esp_err_t request_rtc_and_status(void) {
  /* §5 cross-MCU clock sync: each request carries a fresh 4-byte nonce.
   * WAN echoes it back in the response; LAN rejects any response whose
   * echo doesn't match — that's how we know the wan_us LAN just read
   * was loaded for THIS cycle (not stale from a previous one). */
  static uint32_t s_rtc_nonce = 0;

  /* Ring of recent (nonce → lan_send_us). A stale RTC response (one whose echo
   * is from a PREVIOUS cycle, common when the WAN is busy under §5 load) is
   * still perfectly usable for clock sync — provided we pair its wan_us with
   * the lan_send_us of the SAME cycle it echoes, not the latest one. Rejecting
   * stale samples (the old behaviour) starved clock_sync down to one sample,
   * froze the offset, and let the ~21 ppm crystal skew drift straight into the
   * measured latency. Pairing by echoed nonce keeps the skew regression fed
   * every second → offset re-anchors → no drift. */
#define RTC_NONCE_HIST 16
  static uint32_t s_nonce_hist[RTC_NONCE_HIST]   = {0};
  static uint64_t s_lansend_hist[RTC_NONCE_HIST] = {0};

  s_rtc_nonce++;
  if (s_rtc_nonce == 0) s_rtc_nonce = 1; /* skip 0 — reserved as "no nonce" */

  uint8_t rtc_request[6];
  rtc_request[0] = 'R';
  rtc_request[1] = 'T';
  memcpy(&rtc_request[2], &s_rtc_nonce, sizeof(uint32_t));

  /* Stamp LAN's clock right before issuing [R][T]. WAN stamps wan_us within
   * ~1-3 ms of receiving the request, so using lan_send_us as the LAN-side
   * reference makes the offset bias = WAN's processing time (a few ms)
   * rather than vTaskDelay (~150 ms). */
  uint64_t lan_send_us = (uint64_t)esp_timer_get_time();

  /* Remember this cycle so a later (possibly stale) response can be paired. */
  {
    uint32_t hi = s_rtc_nonce % RTC_NONCE_HIST;
    s_nonce_hist[hi]   = s_rtc_nonce;
    s_lansend_hist[hi] = lan_send_us;
  }

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, rtc_request, sizeof(rtc_request));
  if (status != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  // Ensure command is transmitted immediately before waiting for response
  if (wan_comm_flush_dma_buffer(g_wan_handle) != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  /* Give WAN time to dispatch [R][T] → downlink_send_rtc_response and load a
   * fresh wan_us into the SPI TX buffer. Under §5 bench load, WAN's uplink
   * task is busy shipping HANDLER_LAT frames through WiFi (~50 ms each) so
   * the dispatch can lag behind. 150 ms gives plenty of headroom; with the
   * nonce check, any sample that slips through stale gets rejected. */
  vTaskDelay(pdMS_TO_TICKS(150));

  uint8_t response[40] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK && response[0] == 'R' && response[1] == 'T') {
    /* === Always-fresh fields (RTC string + internet status) ===
     * These don't depend on wan_us cycle — even a response loaded one RTC
     * cycle ago still has the right calendar time and the right online/
     * offline flag. Update them unconditionally so internet detection and
     * RTC cache keep working independently of the clock-sync freshness
     * check below. */
    if (xSemaphoreTake(g_rtc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      memcpy(g_rtc_cache.rtc_string, &response[2], 19);
      g_rtc_cache.rtc_string[19] = '\0';
      g_rtc_cache.valid = true;
      xSemaphoreGive(g_rtc_mutex);
    }
    g_internet_status = (internet_status_t)response[22];

    /* === wan_us → clock sync, paired by echoed nonce ===
     * The WAN echoes the nonce of the request whose wan_us it loaded. Look that
     * nonce up in our ring and pair wan_us with the lan_send_us of THAT cycle —
     * so a response that's a few cycles stale is still a valid (lan,wan) point
     * (its wan_us and the paired lan_send_us belong to the same instant). Only
     * skip if the echoed nonce is unknown (older than the ring / never sent). */
    uint32_t echoed_nonce = 0;
    memcpy(&echoed_nonce, &response[31], sizeof(uint32_t));
    uint32_t ei = echoed_nonce % RTC_NONCE_HIST;
    if (echoed_nonce != 0 && s_nonce_hist[ei] == echoed_nonce) {
      uint64_t wan_us = 0;
      memcpy(&wan_us, &response[23], sizeof(uint64_t));
      clock_sync_lan_update(wan_us, s_lansend_hist[ei]);
      ESP_LOGD(TAG, "RTC: %s, Internet: %s, wan_us=%llu, nonce=%u (lag=%u)",
               g_rtc_cache.rtc_string,
               g_internet_status ? "ONLINE" : "OFFLINE",
               (unsigned long long)wan_us,
               (unsigned)echoed_nonce,
               (unsigned)(s_rtc_nonce - echoed_nonce));
    } else {
      ESP_LOGD(TAG, "RTC: %s, Internet: %s, nonce=%u unknown (clock skip)",
               g_rtc_cache.rtc_string,
               g_internet_status ? "ONLINE" : "OFFLINE",
               (unsigned)echoed_nonce);
    }

    return ESP_OK;
  }

  return ESP_FAIL;
}

/**
 * @brief Send data with retry and ACK
 * NOTE: Caller must hold g_qspi_mutex
 *
 * @param data Payload without DT header (wan_comm_send_data adds DT)
 * @param length Packet length
 * @param[out] ack_out ACK type received
 * @return ESP_OK on success
 */
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out, uint32_t ack_timeout_ms) {
  (void)ack_timeout_ms;  /* fire-and-forget: no ACK wait */

  if (!data || length == 0 || !ack_out) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Fire-and-forget. Framing CRC8+CRC16 guards delivery integrity;
   * g_internet_status (refreshed by request_rtc_and_status, 1s cadence)
   * decides SD persistence at the call site. */
  wan_comm_status_t status = wan_comm_send_data(g_wan_handle, data, length);
  if (status != WAN_COMM_OK) {
    *ack_out = ACK_TYPE_TIMEOUT;
    return ESP_FAIL;
  }
  *ack_out = ACK_TYPE_INTERNET_OK;
  return ESP_OK;
}

/**
 * @brief Build data payload: [handler_type(3)][length(2)][rtc(19)][data]
 */
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len) {
  uint8_t *p = packet;

  // Handler type (3 bytes)
  const char *type_str = handler_id_to_string(item->source_id);
  memcpy(p, type_str, 3);
  p += 3;

  // Total data length (RTC + payload)
  uint16_t total_data_len = 19 + item->length;
  *p++ = (total_data_len >> 8) & 0xFF;
  *p++ = total_data_len & 0xFF;

  // RTC timestamp (19 bytes)
  memcpy(p, item->rtc_timestamp, 19);
  p += 19;

  // Payload
  memcpy(p, item->data, item->length);

  *packet_len = 3 + 2 + 19 + item->length;
}

// ===== GLOBAL VARIABLES (shared with downlink) =====
wan_comm_handle_t g_wan_handle = NULL;
SemaphoreHandle_t g_qspi_mutex = NULL;
SemaphoreHandle_t g_rtc_mutex = NULL;
volatile bool g_handshake_done = false;
bool g_handler_running = false;
void (*g_config_callback)(const uint8_t *, uint16_t, bool) = NULL;

// ===== BACKWARD COMPATIBILITY WRAPPERS =====
esp_err_t mcu_wan_handler_start(void) {
  if (g_handler_running) {
    ESP_LOGW(TAG, "Handler already running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Starting MCU WAN Handler (SPI Split Architecture)");

  wan_comm_config_t wan_config = {.gpio_sck = 12,
                                  .gpio_cs = 10,
                                  .gpio_io0 = 11,
                                  .gpio_io1 = 13,
                                  .gpio_data_ready_input = 45,
                                  .clock_speed_hz = 60000000,
                                  .mode = 0,
                                  .host_id = SPI2_HOST,
                                  .dma_channel = SPI_DMA_CH_AUTO,
                                  .queue_size = 7};

  wan_comm_status_t status = wan_comm_init(&wan_config, &g_wan_handle);
  if (status != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to initialize WAN comm: %d", status);
    return ESP_FAIL;
  }

  g_rtc_mutex = xSemaphoreCreateMutex();
  if (!g_rtc_mutex) {
    ESP_LOGE(TAG, "Failed to create RTC mutex");
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  g_qspi_mutex = xSemaphoreCreateMutex();
  if (!g_qspi_mutex) {
    ESP_LOGE(TAG, "Failed to create SPI mutex");
    vSemaphoreDelete(g_rtc_mutex);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  if (storage_handler_init() != ESP_OK) {
    ESP_LOGW(TAG, "Failed to initialize storage handler");
  }

  g_handler_running = true;
  g_handshake_done = false;

  if (mcu_wan_handler_start_downlink_task() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start downlink task");
    vSemaphoreDelete(g_qspi_mutex);
    vSemaphoreDelete(g_rtc_mutex);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  if (mcu_wan_handler_start_uplink_task() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start uplink task");
    mcu_wan_handler_stop_downlink_task();
    vSemaphoreDelete(g_qspi_mutex);
    vSemaphoreDelete(g_rtc_mutex);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "MCU WAN Handler started successfully");
  return ESP_OK;
}

esp_err_t mcu_wan_handler_stop(void) {
  if (!g_handler_running) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping MCU WAN Handler");
  g_handler_running = false;

  mcu_wan_handler_stop_uplink_task();
  mcu_wan_handler_stop_downlink_task();

  if (g_qspi_mutex) {
    vSemaphoreDelete(g_qspi_mutex);
    g_qspi_mutex = NULL;
  }

  if (g_rtc_mutex) {
    vSemaphoreDelete(g_rtc_mutex);
    g_rtc_mutex = NULL;
  }

  if (g_wan_handle) {
    wan_comm_deinit(g_wan_handle);
    g_wan_handle = NULL;
  }

  storage_handler_deinit();

  ESP_LOGI(TAG, "MCU WAN Handler stopped");
  return ESP_OK;
}

void mcu_wan_handler_register_config_callback(void (*callback)(const uint8_t *,
                                                               uint16_t,
                                                               bool)) {
  g_config_callback = callback;
  ESP_LOGI(TAG, "Config callback %s", callback ? "registered" : "unregistered");
}