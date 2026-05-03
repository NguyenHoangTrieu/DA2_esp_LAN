#include "esp_log.h"
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
#include <string.h>

static const char *TAG = "WAN_UL";

// CONFIGURATION

#define UPLINK_TASK_STACK_SIZE 1024 * 16
#define UPLINK_TASK_PRIORITY 5 // Lower than downlink
/* Queue depth: each item is ~2 KB inline, so 50 items = ~103 KB of internal
 * RAM.  On NVS-restore boots module handlers allocate ~40+ KB before this
 * queue is created, causing xQueueCreate to fail.  5 items (~10 KB) is more
 * than sufficient — the SPI link can only transfer one packet per ~200 ms. */
#define UPLINK_QUEUE_SIZE 5
#define MAX_PAYLOAD_SIZE 2048
#define ACK_TIMEOUT_MS 2000  /* STM32 forwards DT to ThingsBoard via MQTT before ACKing; 200ms was too short */
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
                                  ack_type_t *ack_out);
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

  // Create uplink queue
  g_uplink_queue = xQueueCreate(UPLINK_QUEUE_SIZE, sizeof(uplink_item_t));
  if (!g_uplink_queue) {
    ESP_LOGE(TAG, "Failed to create uplink queue");
    return ESP_FAIL;
  }

  // Stack shifted to PSRAM
  g_uplink_stack = (StackType_t *)heap_caps_malloc(UPLINK_TASK_STACK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_uplink_tcb = (StaticTask_t *)heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (!g_uplink_stack || !g_uplink_tcb) {
      ESP_LOGE(TAG, "Failed to allocate memory for uplink task");
      if (g_uplink_stack) heap_caps_free(g_uplink_stack);
      if (g_uplink_tcb) heap_caps_free(g_uplink_tcb);
      vQueueDelete(g_uplink_queue);
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
             "Statistics: TX=%lu, FAIL=%lu, SD_BACKUP=%lu, SD_RETRY_OK=%lu",
             g_uplink_sent_count, g_uplink_fail_count, g_sd_backup_count,
             g_sd_retry_success_count);
  }

  if (g_uplink_queue) {
    vQueueDelete(g_uplink_queue);
    g_uplink_queue = NULL;
  }
}

bool mcu_wan_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                            uint16_t len) {
  if (!g_uplink_queue || !data || len == 0) {
    ESP_LOGE(TAG, "Invalid uplink parameters");
    return false;
  }

  if (len > MAX_PAYLOAD_SIZE) {
    ESP_LOGE(TAG, "Uplink data too large: %u > %d", len, MAX_PAYLOAD_SIZE);
    return false;
  }

  uplink_item_t item;
  item.source_id = source_id;
  item.length = len;
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

  if (xQueueSend(g_uplink_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
#if !BENCH_QUIET_LOG
    ESP_LOGW(TAG, "Uplink queue full");
#endif
    return false;
  }

#if !BENCH_QUIET_LOG
  ESP_LOGI(TAG, "Uplink queued from handler %s: %u bytes",
           handler_id_to_string(source_id), len);
#endif
  return true;
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
  uplink_item_t uplink_item;

  while (g_handler_running) {

    TickType_t now = xTaskGetTickCount();

    // Try to acquire SPI mutex (non-blocking / short timeout)
    // If downlink task has it, we'll skip and try next iteration
    if (xSemaphoreTake(g_qspi_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) ==
        pdTRUE) {

      // A) Check Uplink Queue

      if (xQueueReceive(g_uplink_queue, &uplink_item, 0) == pdTRUE) {

#if !BENCH_QUIET_LOG
        ESP_LOGI(TAG, "Processing uplink from handler %s: %u bytes",
                 handler_id_to_string(uplink_item.source_id),
                 uplink_item.length);
#endif

        uint8_t packet[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
        uint16_t packet_len = 0;
        build_data_packet(&uplink_item, packet, &packet_len);

        if (g_internet_status == INTERNET_STATUS_ONLINE) {
          ack_type_t ack_result;
          esp_err_t send_result =
              send_data_to_wan(packet, packet_len, &ack_result);

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
      }

      // B) Check SD Card Backup Retry (if internet online)

      if (storage_handler_has_data() &&
          g_internet_status == INTERNET_STATUS_ONLINE) {

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
                send_data_to_wan(sd_buffer, sd_length, &ack_result);

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

      // Release SPI mutex
      xSemaphoreGive(g_qspi_mutex);
    }

    // D) Periodic Flush (500ms to match timeout batching, outside SPI mutex)
    // This handles timeout flushes set by storage handler timer callback

    if ((now - last_flush) >= pdMS_TO_TICKS(500)) {
      storage_handler_flush();
      wan_comm_flush_dma_buffer(g_wan_handle);
      last_flush = now;
    }

    // Small delay before next iteration
    vTaskDelay(pdMS_TO_TICKS(10));
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
  uint8_t rtc_request[2] = {'R', 'T'};

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, rtc_request, sizeof(rtc_request));
  if (status != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  // Ensure command is transmitted immediately before waiting for response
  if (wan_comm_flush_dma_buffer(g_wan_handle) != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  uint8_t response[32] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK && response[0] == 'R' && response[1] == 'T') {
    // Update RTC cache
    if (xSemaphoreTake(g_rtc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      memcpy(g_rtc_cache.rtc_string, &response[2], 19);
      g_rtc_cache.rtc_string[19] = '\0';
      g_rtc_cache.valid = true;
      xSemaphoreGive(g_rtc_mutex);
    }

    // Update internet status
    g_internet_status = (internet_status_t)response[22];

    ESP_LOGD(TAG, "RTC: %s, Internet: %s", g_rtc_cache.rtc_string,
             g_internet_status ? "ONLINE" : "OFFLINE");

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
                                  ack_type_t *ack_out) {
  if (!data || length == 0 || !ack_out) {
    return ESP_ERR_INVALID_ARG;
  }

  for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {

#if !BENCH_QUIET_LOG
    ESP_LOGI(TAG, "Transmit attempt %d/%d", retry + 1, MAX_RETRY_COUNT);
#endif

    wan_comm_status_t status = wan_comm_send_data(g_wan_handle, data, length);
    if (status != WAN_COMM_OK) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Poll ACK within ACK_TIMEOUT_MS
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(ACK_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {

      /* Use 256-byte buffer so the ACK [0x02][0x11] is found even when WAN
       * bundles a local-response DT payload before the ACK in the same
       * 1024-byte SPI frame. */
      uint8_t ack_response[256] = {0};
      status = wan_comm_request_data(g_wan_handle, ack_response,
                                     sizeof(ack_response));

      if (status == WAN_COMM_OK) {
        /* Scan the full 256-byte response for the ACK pattern */
        for (int i = 0; i <= (int)sizeof(ack_response) - 3; i++) {
          if (ack_response[i] == 0x02 &&
              ack_response[i + 1] == ACK_TYPE_RECEIVED_OK) {

            *ack_out = (ack_type_t)ack_response[i + 2];
            /* STM32 may return boolean 0x01/0x00 instead of enum 0x12/0x13 —
             * normalise: any non-zero internet byte = INTERNET_OK */
            if (*ack_out != ACK_TYPE_INTERNET_OK && *ack_out != ACK_TYPE_NO_INTERNET) {
              *ack_out = (ack_response[i + 2] != 0) ? ACK_TYPE_INTERNET_OK
                                                     : ACK_TYPE_NO_INTERNET;
            }
#if !BENCH_QUIET_LOG
            ESP_LOGI(TAG, "ACK received: %s (ack=0x%02X internet=0x%02X)",
                     (*ack_out == ACK_TYPE_INTERNET_OK) ? "INTERNET_OK" : "NO_INTERNET",
                     ack_response[i + 1], ack_response[i + 2]);
#endif
            return ESP_OK;
          }
        }
      }

      // Small yield to allow other tasks to run
      taskYIELD();
    }

    ESP_LOGW(TAG, "ACK timeout on attempt %d", retry + 1);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  ESP_LOGW(TAG, "Max retries reached");
  *ack_out = ACK_TYPE_TIMEOUT;
  return ESP_FAIL;
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
                                  .clock_speed_hz = 10000000,
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