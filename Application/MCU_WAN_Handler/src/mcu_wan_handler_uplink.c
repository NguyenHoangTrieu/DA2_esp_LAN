#include "can_handler.h"
#include "esp_log.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lora_tdma_connect.h"
#include "mcu_wan_handler.h"
#include "rs485_handler.h"
#include "stack_handler.h"
#include "storage_handler.h"
#include "wan_comm.h"
#include "zigbee_nostack_connect.h"
#include <string.h>

static const char *TAG = "WAN_UL";

// CONFIGURATION

#define UPLINK_TASK_STACK_SIZE 4096
#define UPLINK_TASK_PRIORITY 5 // Lower than downlink
#define UPLINK_QUEUE_SIZE 50
#define MAX_PAYLOAD_SIZE 512
#define ACK_TIMEOUT_MS 200
#define RTC_REQUEST_INTERVAL_MS 1000
#define MAX_RETRY_COUNT 3
#define HANDSHAKE_INTERVAL_MS 1000
#define HANDSHAKE_TIMEOUT_MS 100
#define MUTEX_TIMEOUT_MS 50

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
extern stack_comm_type_t g_stack_1_type;
extern stack_comm_type_t g_stack_2_type;

// MODULE STATE

static QueueHandle_t g_uplink_queue = NULL;
static TaskHandle_t g_uplink_task_handle = NULL;
static internet_status_t g_internet_status = INTERNET_STATUS_OFFLINE;
static rtc_cache_t g_rtc_cache = {.rtc_string = {0}, .valid = false};
static uint32_t g_cached_wan_fw_version = 0;

// Statistics
static uint32_t g_uplink_sent_count = 0;
static uint32_t g_uplink_fail_count = 0;
static uint32_t g_sd_backup_count = 0;
static uint32_t g_sd_retry_success_count = 0;

// FORWARD DECLARATIONS

static void uplink_handler_task(void *pvParameters);
static esp_err_t perform_handshake(void);
static esp_err_t request_rtc_and_status(void);
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out);
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len);
static void stack_handler_start(stack_comm_type_t stack_type);
static const char *handler_id_to_string(handler_id_t id);

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

  // Create task
  BaseType_t ret =
      xTaskCreate(uplink_handler_task, "wan_uplink", UPLINK_TASK_STACK_SIZE,
                  NULL, UPLINK_TASK_PRIORITY, &g_uplink_task_handle);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create uplink task");
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
    ESP_LOGW(TAG, "Uplink queue full");
    return false;
  }

  ESP_LOGI(TAG, "Uplink queued from handler %s: %u bytes",
           handler_id_to_string(source_id), len);
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
    // Take QSPI mutex for handshake
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

  // Start stack handlers after handshake
  stack_handler_start(g_stack_1_type);
  stack_handler_start(g_stack_2_type);

  // PHASE 2: Uplink/RTC/SD Loop

  ESP_LOGI(TAG, "Phase 2: Uplink processing loop");

  TickType_t last_rtc_request = xTaskGetTickCount();
  TickType_t last_flush = xTaskGetTickCount();
  uplink_item_t uplink_item;

  while (g_handler_running) {

    TickType_t now = xTaskGetTickCount();

    // Try to acquire QSPI mutex (non-blocking / short timeout)
    // If downlink task has it, we'll skip and try next iteration
    if (xSemaphoreTake(g_qspi_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) ==
        pdTRUE) {

      // A) Check Uplink Queue

      if (xQueueReceive(g_uplink_queue, &uplink_item, 0) == pdTRUE) {

        ESP_LOGI(TAG, "Processing uplink from handler %s: %u bytes",
                 handler_id_to_string(uplink_item.source_id),
                 uplink_item.length);

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
              ESP_LOGI(TAG, "Uplink sent successfully (#%lu)",
                       g_uplink_sent_count);
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

        // Prepare retry session (open oldest file)
        if (storage_handler_prepare_retry() == ESP_OK) {
          ESP_LOGI(TAG, "Starting SD card retry session");

          bool session_success = true;
          uint8_t sd_buffer[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
          uint16_t sd_length = 0; // Changed to uint16_t to match new API

          // Process all packets in the file stream
          while (storage_handler_get_next_packet(sd_buffer, &sd_length,
                                                 sizeof(sd_buffer)) == ESP_OK) {

            ESP_LOGI(TAG, "Retrying SD packet: %u bytes", sd_length);

            ack_type_t ack_result =
                ACK_TYPE_TIMEOUT; // Initialize to avoid garbage
            esp_err_t send_result =
                send_data_to_wan(sd_buffer, sd_length, &ack_result);

            if (send_result == ESP_OK && ack_result == ACK_TYPE_INTERNET_OK) {
              g_sd_retry_success_count++;
              ESP_LOGI(TAG, "SD packet sent OK (#%lu)",
                       g_sd_retry_success_count);
              // Continue to next packet
            } else {
              ESP_LOGW(TAG,
                       "SD packet send failed/timeout, aborting retry session");
              session_success = false;

              if (ack_result == ACK_TYPE_NO_INTERNET) {
                g_internet_status = INTERNET_STATUS_OFFLINE;
              }
              break; // Stop processing this file, retry later
            }
          }

          storage_handler_finish_retry(session_success);
        }
      }

      // C) RTC Periodic Timer (1 second interval)

      if ((now - last_rtc_request) >= pdMS_TO_TICKS(RTC_REQUEST_INTERVAL_MS)) {
        if (request_rtc_and_status() == ESP_OK) {
          ESP_LOGD(TAG, "RTC and Internet status updated");
        }
        last_rtc_request = now;
      }

      // Release QSPI mutex
      xSemaphoreGive(g_qspi_mutex);
    }

    // D) Periodic Flush (5 seconds, outside QSPI mutex)

    if ((now - last_flush) >= pdMS_TO_TICKS(5000)) {
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
  uint8_t handshake_req[7];
  handshake_req[0] = (WAN_COMM_HEADER_CF >> 8) & 0xFF;
  handshake_req[1] = WAN_COMM_HEADER_CF & 0xFF;
  handshake_req[2] = 0x01; // Handshake subtype
  handshake_req[3] = (LAN_FW_VERSION >> 24) & 0xFF;
  handshake_req[4] = (LAN_FW_VERSION >> 16) & 0xFF;
  handshake_req[5] = (LAN_FW_VERSION >> 8) & 0xFF;
  handshake_req[6] = LAN_FW_VERSION & 0xFF;

  ESP_LOGI(TAG, "Sending handshake: LAN FW v%u.%u.%u.%u", LAN_FW_VERSION_MAJOR,
           LAN_FW_VERSION_MINOR, LAN_FW_VERSION_PATCH, LAN_FW_VERSION_BUILD);

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, handshake_req, sizeof(handshake_req));
  if (status != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to send handshake request");
    return ESP_FAIL;
  }

  // Wait for ACK response from WAN MCU
  vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS));

  uint8_t response[16] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK && response[0] == 0x02 &&
      response[1] == ACK_TYPE_HANDSHAKE) {
    // Extract internet status and WAN FW version
    g_internet_status = (internet_status_t)response[2];
    g_cached_wan_fw_version =
        ((uint32_t)response[3] << 24) | ((uint32_t)response[4] << 16) |
        ((uint32_t)response[5] << 8) | ((uint32_t)response[6]);

    ESP_LOGI(TAG, "Handshake ACK received:");
    ESP_LOGI(TAG, "  Internet: %s", g_internet_status ? "ONLINE" : "OFFLINE");
    ESP_LOGI(TAG, "  WAN FW: v%u.%u.%u.%u",
             FW_VERSION_MAJOR(g_cached_wan_fw_version),
             FW_VERSION_MINOR(g_cached_wan_fw_version),
             FW_VERSION_PATCH(g_cached_wan_fw_version),
             FW_VERSION_BUILD(g_cached_wan_fw_version));

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
  uint8_t rtc_request[4] = {(WAN_COMM_HEADER_CF >> 8) & 0xFF,
                            WAN_COMM_HEADER_CF & 0xFF, 'R', 'T'};

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, rtc_request, sizeof(rtc_request));
  if (status != WAN_COMM_OK) {
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
 * @param data Complete packet (DT header + payload)
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

    ESP_LOGI(TAG, "Transmit attempt %d/%d", retry + 1, MAX_RETRY_COUNT);

    wan_comm_status_t status = wan_comm_send_data(g_wan_handle, data, length);
    if (status != WAN_COMM_OK) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Poll ACK within ACK_TIMEOUT_MS
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(ACK_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {

      uint8_t ack_response[8] = {0};
      status = wan_comm_request_data(g_wan_handle, ack_response,
                                     sizeof(ack_response));

      if (status == WAN_COMM_OK && ack_response[0] == 0x02 &&
          ack_response[1] == ACK_TYPE_RECEIVED_OK) {

        *ack_out = (ack_type_t)ack_response[2];
        ESP_LOGI(TAG, "ACK received: %s",
                 (*ack_out == ACK_TYPE_INTERNET_OK) ? "INTERNET_OK"
                                                    : "NO_INTERNET");
        return ESP_OK;
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
 * @brief Build data packet: [DT][handler_type(3)][length(2)][rtc(19)][data]
 */
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len) {
  uint8_t *p = packet;

  // DT header
  *p++ = (WAN_COMM_HEADER_DT >> 8) & 0xFF;
  *p++ = WAN_COMM_HEADER_DT & 0xFF;

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

  *packet_len = DATA_PACKET_HEADER_SIZE + 19 + item->length;
}

/**
 * @brief Start stack handler based on type
 */
static void stack_handler_start(stack_comm_type_t stack_type) {
  switch (stack_type) {
  case STACK_COMM_TYPE_CAN:
    can_handler_start();
    break;
  case STACK_COMM_TYPE_ZIGBEE:
    zigbee_nostack_connect_start();
    break;
  case STACK_COMM_TYPE_LORA:
    lora_tdma_connect_start();
    break;
  case STACK_COMM_TYPE_RS485:
    rs485_handler_start();
    break;
  default:
    ESP_LOGW(TAG, "Unknown stack type: %d", stack_type);
    break;
  }
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

  ESP_LOGI(TAG, "Starting MCU WAN Handler (QSPI Split Architecture)");

  wan_comm_config_t wan_config = {.gpio_sck = 12,
                                  .gpio_cs = 10,
                                  .gpio_io0 = 11,
                                  .gpio_io1 = 13,
                                  .gpio_io2 = 14,
                                  .gpio_io3 = 15,
                                  .gpio_data_ready_input = 46,
                                  .clock_speed_hz = 40000000,
                                  .mode = 0,
                                  .host_id = SPI2_HOST,
                                  .dma_channel = SPI_DMA_CH_AUTO,
                                  .queue_size = 7,
                                  .enable_quad_mode = true};

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
    ESP_LOGE(TAG, "Failed to create QSPI mutex");
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