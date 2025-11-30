/**
 * @file mcu_wan_handler.c
 * @brief MCU WAN Communication Handler Implementation (SPI Master - LAN MCU)
 */

#include "mcu_wan_handler.h"
#include "config_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "wan_comm.h"
#include <string.h>
#include <time.h>

static const char *TAG = "MCU_WAN";

// ===== Configuration =====
#define MCU_WAN_TASK_STACK_SIZE 4096
#define MCU_WAN_TASK_PRIORITY 5
#define DATA_QUEUE_SIZE 50
#define HANDSHAKE_INTERVAL_MS 1000
#define PERIODIC_REQUEST_INTERVAL_MS 1000
#define ACK_TIMEOUT_MS 500
#define MAX_RETRY_COUNT 3
#define SD_CARD_BUFFER_SIZE 2048

// ===== Protocol Commands =====
#define CMD_HANDSHAKE_ACK 0x01
#define CMD_REQUEST_RTC_CONFIG 0x02
#define CMD_DATA_PACKET 0x03

// ===== ACK Types =====
#define ACK_RECEIVED_INTERNET_OK 0x01
#define ACK_RECEIVED_NO_INTERNET 0x02

// ===== RTC Structure =====
typedef struct {
  uint8_t day;
  uint8_t month;
  uint16_t year;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} rtc_time_t;

// ===== Data Packet Structure =====
typedef struct {
  uint8_t *data;
  uint16_t length;
  uint64_t timestamp_ms;
} wan_data_packet_t;

// ===== Config Cache =====
typedef struct {
  uint8_t config_data[256];
  uint16_t config_length;
  bool has_config;
} config_cache_t;

// ===== Global Variables =====
static wan_comm_handle_t g_wan_handle = NULL;
static TaskHandle_t g_task_handle = NULL;
static QueueHandle_t g_data_queue = NULL;
static bool g_handler_running = false;
static rtc_time_t g_rtc_cache = {0};
static internet_status_t g_internet_status = INTERNET_STATUS_OFFLINE;
static config_cache_t g_config_cache = {0};

// ===== Forward Declarations =====
static void mcu_wan_handler_task(void *pvParameters);
static esp_err_t perform_handshake(void);
static esp_err_t request_rtc_config_and_status(void);
static esp_err_t send_data_with_retry(const uint8_t *data, uint16_t length);
static esp_err_t save_to_sd_card(const uint8_t *data, uint16_t length);
static esp_err_t read_from_sd_card(uint8_t *buffer, uint16_t *length);
static bool sd_card_has_data(void);
static void delete_from_sd_card(void);

// ===== Public API =====

esp_err_t mcu_wan_handler_start(void) {
  if (g_handler_running) {
    ESP_LOGW(TAG, "Handler already running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Starting MCU WAN Handler (SPI Master)");

  // Initialize WAN communication
  wan_comm_config_t wan_config = {.gpio_sck = 12,
                                  .gpio_cs = 10,
                                  .gpio_io0 = 11, // MOSI
                                  .gpio_io1 = 13, // MISO
                                  .gpio_io2 = -1,
                                  .gpio_io3 = -1,
                                  .clock_speed_hz = 10000000, // 10 MHz
                                  .mode = 0,
                                  .host_id = SPI2_HOST,
                                  .dma_channel = SPI_DMA_CH_AUTO,
                                  .queue_size = 7,
                                  .enable_quad_mode = false};

  wan_comm_status_t status = wan_comm_init(&wan_config, &g_wan_handle);
  if (status != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to initialize WAN comm: %d", status);
    return ESP_FAIL;
  }

  // Create data queue
  g_data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(wan_data_packet_t));
  if (g_data_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create data queue");
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  // TODO: Initialize SD card
  ESP_LOGI(TAG, "TODO: Initialize SD card for buffering");

  // Create handler task
  BaseType_t ret =
      xTaskCreate(mcu_wan_handler_task, "mcu_wan_task", MCU_WAN_TASK_STACK_SIZE,
                  NULL, MCU_WAN_TASK_PRIORITY, &g_task_handle);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task");
    vQueueDelete(g_data_queue);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  g_handler_running = true;
  ESP_LOGI(TAG, "MCU WAN Handler started successfully");
  return ESP_OK;
}

esp_err_t mcu_wan_handler_stop(void) {
  if (!g_handler_running) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping MCU WAN Handler");
  g_handler_running = false;

  if (g_task_handle != NULL) {
    vTaskDelete(g_task_handle);
    g_task_handle = NULL;
  }

  if (g_data_queue != NULL) {
    vQueueDelete(g_data_queue);
    g_data_queue = NULL;
  }

  if (g_wan_handle != NULL) {
    wan_comm_deinit(g_wan_handle);
    g_wan_handle = NULL;
  }

  ESP_LOGI(TAG, "MCU WAN Handler stopped");
  return ESP_OK;
}

esp_err_t mcu_wan_handler_queue_data(const uint8_t *data, uint16_t length) {
  if (g_data_queue == NULL || data == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  wan_data_packet_t packet;
  packet.data = (uint8_t *)malloc(length);
  if (packet.data == NULL) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(packet.data, data, length);
  packet.length = length;
  packet.timestamp_ms = esp_timer_get_time() / 1000;

  if (xQueueSend(g_data_queue, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
    free(packet.data);
    ESP_LOGE(TAG, "Data queue full");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Data queued (%d bytes)", length);
  return ESP_OK;
}

internet_status_t mcu_wan_handler_get_internet_status(void) {
  return g_internet_status;
}

esp_err_t mcu_wan_handler_get_config(uint8_t *buffer, uint16_t buffer_size,
                                     uint16_t *actual_length) {
  if (!g_config_cache.has_config) {
    return ESP_ERR_NOT_FOUND;
  }

  if (buffer == NULL || actual_length == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (g_config_cache.config_length > buffer_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  memcpy(buffer, g_config_cache.config_data, g_config_cache.config_length);
  *actual_length = g_config_cache.config_length;

  return ESP_OK;
}

// ===== Main Task =====

static void mcu_wan_handler_task(void *pvParameters) {
  ESP_LOGI(TAG, "MCU WAN Handler task started");

  // Phase 1: Handshake
  ESP_LOGI(TAG, "Phase 1: Handshake");
  while (g_handler_running) {
    if (perform_handshake() == ESP_OK) {
      ESP_LOGI(TAG, "Handshake successful, entering Data Mode");
      break;
    }
    ESP_LOGW(TAG, "Handshake failed, retrying in 1s...");
    vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_INTERVAL_MS));
  }

  // Phase 2: Main Loop
  ESP_LOGI(TAG, "Phase 2: Data Mode");
  TickType_t last_periodic_request = xTaskGetTickCount();
  wan_data_packet_t data_packet;

  while (g_handler_running) {
    // Check 1s periodic timer
    TickType_t now = xTaskGetTickCount();
    if ((now - last_periodic_request) >=
        pdMS_TO_TICKS(PERIODIC_REQUEST_INTERVAL_MS)) {
      // Send periodic request for RTC, config, and internet status
      request_rtc_config_and_status();
      last_periodic_request = now;
    }

    // Check data queue (Priority: real-time data)
    if (xQueueReceive(g_data_queue, &data_packet, pdMS_TO_TICKS(10)) ==
        pdTRUE) {
      ESP_LOGI(TAG, "Data received from queue (%d bytes)", data_packet.length);

      // Package data with current RTC
      uint8_t *packaged_data = (uint8_t *)malloc(19 + data_packet.length);
      if (packaged_data != NULL) {
        // Format RTC: dd/mm/yyyy-hh:mm:ss
        char rtc_buffer[32];  // Larger buffer for snprintf safety
        snprintf(rtc_buffer, sizeof(rtc_buffer), "%02d/%02d/%04d-%02d:%02d:%02d",
                (int)(g_rtc_cache.day % 100),      // Clamp to 0-99
                (int)(g_rtc_cache.month % 100),    // Clamp to 0-99
                (int)g_rtc_cache.year,
                (int)(g_rtc_cache.hour % 100),     // Clamp to 0-99
                (int)(g_rtc_cache.minute % 100),   // Clamp to 0-99
                (int)(g_rtc_cache.second % 100));  // Clamp to 0-99
        memcpy(packaged_data, rtc_buffer, 19);
        memcpy(&packaged_data[19], data_packet.data, data_packet.length);

        // Check internet status
        if (g_internet_status == INTERNET_STATUS_ONLINE) {
          // Send to WAN with retry
          if (send_data_with_retry(packaged_data, 19 + data_packet.length) !=
              ESP_OK) {
            ESP_LOGE(TAG, "Failed to send after retries, saving to SD");
            save_to_sd_card(packaged_data, 19 + data_packet.length);
          }
        } else {
          ESP_LOGW(TAG, "Internet offline, saving to SD card");
          save_to_sd_card(packaged_data, 19 + data_packet.length);
        }

        free(packaged_data);
      }

      free(data_packet.data);
      continue;
    }

    // Check SD card recovery (lower priority)
    if (sd_card_has_data() && g_internet_status == INTERNET_STATUS_ONLINE) {
      uint8_t sd_buffer[SD_CARD_BUFFER_SIZE];
      uint16_t sd_length = 0;

      if (read_from_sd_card(sd_buffer, &sd_length) == ESP_OK) {
        ESP_LOGI(TAG, "Sending SD card data (%d bytes)", sd_length);
        if (send_data_with_retry(sd_buffer, sd_length) == ESP_OK) {
          delete_from_sd_card();
          ESP_LOGI(TAG, "SD data sent successfully");
        } else {
          ESP_LOGW(TAG, "Failed to send SD data, will retry later");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGI(TAG, "MCU WAN Handler task exiting");
  vTaskDelete(NULL);
}

// ===== Handshake Implementation =====

static esp_err_t perform_handshake(void) {
  ESP_LOGI(TAG, "Sending handshake ACK to WAN MCU");

  uint8_t handshake_cmd = CMD_HANDSHAKE_ACK;
  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, &handshake_cmd, 1);

  if (status != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to send handshake");
    return ESP_FAIL;
  }

  // Wait for ACK response
  uint8_t response[16] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK) {
    uint16_t header = (response[0] << 8) | response[1];
    uint8_t cmd = response[2];

    if (header == WAN_COMM_HEADER_CF && cmd == CMD_HANDSHAKE_ACK) {
      ESP_LOGI(TAG, "Received ACK from WAN MCU - Handshake complete");
      return ESP_OK;
    }
  }

  return ESP_FAIL;
}

// ===== Periodic Request Implementation =====

static esp_err_t request_rtc_config_and_status(void) {
  ESP_LOGD(TAG, "Requesting RTC, Config, and Internet Status");

  uint8_t request_cmd = CMD_REQUEST_RTC_CONFIG;
  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, &request_cmd, 1);

  if (status != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to send request");
    return ESP_FAIL;
  }

  // Wait for response
  uint8_t response[512] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status != WAN_COMM_OK) {
    ESP_LOGW(TAG, "No response to RTC/Config request");
    return ESP_FAIL;
  }

  // Parse response: [Header 2B][CMD 1B][RTC 19B][-][Config data][-][Internet
  // status 1B]
  uint16_t header = (response[0] << 8) | response[1];
  uint8_t cmd = response[2];

  if (header != WAN_COMM_HEADER_CF || cmd != CMD_REQUEST_RTC_CONFIG) {
    ESP_LOGE(TAG, "Invalid response header");
    return ESP_FAIL;
  }

  // Parse RTC (bytes 3-21)
  char rtc_str[20] = {0};
  memcpy(rtc_str, &response[3], 19);
  if (sscanf(rtc_str, "%hhu/%hhu/%hu-%hhu:%hhu:%hhu", &g_rtc_cache.day,
             &g_rtc_cache.month, &g_rtc_cache.year, &g_rtc_cache.hour,
             &g_rtc_cache.minute, &g_rtc_cache.second) == 6) {
    ESP_LOGI(TAG, "RTC updated: %s", rtc_str);
  }

  // Find separators and parse config
  uint16_t offset = 22; // After RTC
  if (response[offset] == '-') {
    offset++;
    // Find next separator
    const uint8_t *separator =
        memchr(&response[offset], '-', sizeof(response) - offset);
    if (separator) {
      uint16_t config_len = separator - &response[offset];
      if (config_len > 0 && config_len < sizeof(g_config_cache.config_data)) {
        if (strncmp((char *)&response[offset], "NO_CF", 5) == 0) {
          g_config_cache.has_config = false;
          ESP_LOGI(TAG, "No config available");
        } else {
          memcpy(g_config_cache.config_data, &response[offset], config_len);
          g_config_cache.config_length = config_len;
          g_config_cache.has_config = true;
          ESP_LOGI(TAG, "Config received: %.*s", config_len, &response[offset]);

          config_type_t cfg_type =
              config_parse_type((char *)&response[offset], config_len);
          config_command_t cmd;
          cmd.type = cfg_type;
          memcpy(cmd.raw_data, &response[offset], config_len);
          cmd.data_len = config_len;
          xQueueSend(g_config_handler_queue, &cmd, pdMS_TO_TICKS(100));
        }
      }

      // Parse internet status
      offset = (separator - response) + 1;
      if (offset < sizeof(response)) {
        g_internet_status =
            response[offset] ? INTERNET_STATUS_ONLINE : INTERNET_STATUS_OFFLINE;
        ESP_LOGI(TAG, "Internet status: %s",
                 g_internet_status == INTERNET_STATUS_ONLINE ? "ONLINE"
                                                             : "OFFLINE");
      }
    }
  }

  return ESP_OK;
}

// ===== Reliable Send with Retry =====

static esp_err_t send_data_with_retry(const uint8_t *data, uint16_t length) {
  esp_err_t result = ESP_FAIL;

  for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {
    ESP_LOGI(TAG, "Sending data (attempt %d/%d)", retry + 1, MAX_RETRY_COUNT);

    // Send data packet
    wan_comm_status_t status = wan_comm_send_data(g_wan_handle, data, length);
    if (status != WAN_COMM_OK) {
      ESP_LOGE(TAG, "Failed to send data");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Wait for ACK with timeout
    uint8_t ack_buffer[16] = {0};
    status =
        wan_comm_request_data(g_wan_handle, ack_buffer, sizeof(ack_buffer));

    if (status == WAN_COMM_OK) {
      uint16_t header = (ack_buffer[0] << 8) | ack_buffer[1];
      if (header == WAN_COMM_HEADER_CF) {
        uint8_t ack_type = ack_buffer[2];

        if (ack_type == ACK_RECEIVED_INTERNET_OK) {
          ESP_LOGI(TAG, "ACK received: Internet OK");
          g_internet_status = INTERNET_STATUS_ONLINE;
          result = ESP_OK;
          break;
        } else if (ack_type == ACK_RECEIVED_NO_INTERNET) {
          ESP_LOGW(TAG, "ACK received: No Internet");
          g_internet_status = INTERNET_STATUS_OFFLINE;
          // Save to SD card
          save_to_sd_card(data, length);
          result = ESP_OK; // ACK received, but need to buffer
          break;
        }
      }
    }

    ESP_LOGW(TAG, "No ACK received, retrying...");
    vTaskDelay(pdMS_TO_TICKS(ACK_TIMEOUT_MS));
  }

  return result;
}

// ===== SD Card Operations (Placeholder) =====

static esp_err_t save_to_sd_card(const uint8_t *data, uint16_t length) {
  ESP_LOGI(TAG, "TODO: Saving %d bytes to SD card", length);
  // TODO: Implement SD card write
  return ESP_OK;
}

static esp_err_t read_from_sd_card(uint8_t *buffer, uint16_t *length) {
  ESP_LOGD(TAG, "TODO: Reading oldest data from SD card");
  // TODO: Implement SD card read
  *length = 0;
  return ESP_ERR_NOT_FOUND;
}

static bool sd_card_has_data(void) {
  // TODO: Check if SD card has buffered data
  return false;
}

static void delete_from_sd_card(void) {
  // TODO: Delete oldest entry from SD card
  ESP_LOGI(TAG, "TODO: Delete oldest SD card entry");
}
