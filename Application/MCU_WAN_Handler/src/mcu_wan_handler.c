/**
 * @file mcu_wan_handler.c
 * @brief MCU WAN Handler - LAN Side (SPI Master)
 *
 * Implements Diagram 1: System Control & Data Handling Logic (LAN Side)
 * - Handshake with WAN MCU every 1 second until ACK received
 * - Periodic RTC/Internet status requests
 * - Data queue management with SD card backup
 * - Transmission retry with ACK verification
 */
#include "mcu_wan_handler.h"
#include "can_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lora_e32_comm.h"
#include "lora_tdma_connect.h"
#include "lora_tdma_handler.h"
#include "wan_comm.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MCU_WAN";

// ===== Configuration =====
#define MCU_WAN_TASK_STACK_SIZE 4096
#define MCU_WAN_TASK_PRIORITY 5
#define UPLINK_QUEUE_SIZE 50
#define HANDSHAKE_INTERVAL_MS 1000
#define RTC_REQUEST_INTERVAL_MS 1000
#define DATA_POLLING_INTERVAL_MS 100
#define ACK_TIMEOUT_MS 1000
#define MAX_RETRY_COUNT 3
#define MAX_PAYLOAD_SIZE 512
#define SD_CARD_MAX_FILES 100

// Global configuration variables (defined in other modules)
extern can_config_t g_can_config;
extern uint16_t g_can_whitelist[MAX_WHITELISTED_IDS];
extern uint16_t g_can_whitelist_count;
extern lora_handler_config_t g_lora_handler_cfg;
extern uint8_t g_lora_handler_crypto_key_len;
extern e32_params_t g_lora_e32_params;
extern int g_lora_e32_baud_rate;

// ===== Uplink Queue Item =====
typedef struct {
  handler_id_t source_id;
  uint8_t data[MAX_PAYLOAD_SIZE];
  uint16_t length;
  char rtc_timestamp[20];
} uplink_item_t;

// ===== RTC Cache =====
typedef struct {
  char rtc_string[20];
  bool valid;
} rtc_cache_t;

// ===== Global Variables =====
static wan_comm_handle_t g_wan_handle = NULL;
static TaskHandle_t g_task_handle = NULL;
static QueueHandle_t g_uplink_queue = NULL;
static SemaphoreHandle_t g_rtc_mutex = NULL;
static bool g_handler_running = false;

// RTC and Internet status
static internet_status_t g_internet_status = INTERNET_STATUS_OFFLINE;
static rtc_cache_t g_rtc_cache = {{0}, false};

// SD Card backup management
static uint32_t g_sd_card_file_count = 0;

// Config callback
static void (*g_config_callback)(const uint8_t *, uint16_t, bool) = NULL;

// ===== Forward Declarations =====
static void mcu_wan_handler_task(void *pvParameters);
static esp_err_t perform_handshake(void);
static esp_err_t request_rtc_and_status(void);
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out);
static esp_err_t save_to_sd_card(const uint8_t *data, uint16_t length);
static esp_err_t read_oldest_from_sd_card(uint8_t *buffer, uint16_t *length);
static bool sd_card_has_data(void);
static void delete_oldest_from_sd_card(void);
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len);
static void dispatch_downlink_to_handler(handler_id_t target_id,
                                         const uint8_t *data, uint16_t length);
static const char *handler_id_to_string(handler_id_t id);
static handler_id_t string_to_handler_id(const uint8_t *type_str);

// External downlink callbacks
extern bool can_handler_enqueue_downlink(uint8_t *data, uint16_t len);
extern bool lora_tdma_connect_enqueue_downlink(uint8_t *data, uint16_t len);
extern bool zigbee_nostack_connect_enqueue_downlink(uint8_t *data,
                                                    uint16_t len);

// ===== GPIO Handshake Configuration =====
#define GPIO_DATA_READY_PIN 14
#define NOTIFY_DATA_READY (1 << 0)

static volatile bool g_data_ready_flag = false;

// GPIO ISR Handler
static void IRAM_ATTR gpio_data_ready_isr(void *arg) {
  g_data_ready_flag = true;
  BaseType_t xTaskWoken = pdFALSE;
  if (g_task_handle) {
    xTaskNotifyFromISR(g_task_handle, NOTIFY_DATA_READY, eSetBits, &xTaskWoken);
  }
  if (xTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// Setup GPIO 14 as input with interrupt
static esp_err_t setup_data_ready_gpio(void) {
  gpio_config_t io_conf = {
      .pin_bit_mask = BIT64(GPIO_DATA_READY_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE // Trigger on rising edge
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure GPIO %d", GPIO_DATA_READY_PIN);
    return ret;
  }

  ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to install ISR service");
    return ret;
  }

  ret = gpio_isr_handler_add(GPIO_DATA_READY_PIN, gpio_data_ready_isr, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add ISR handler");
    return ret;
  }

  ESP_LOGI(TAG, "GPIO %d configured for data-ready notification",
           GPIO_DATA_READY_PIN);
  return ESP_OK;
}

/**
 * @brief Build and send LAN configuration response to WAN MCU
 */
static void send_lan_config_response(void) {
  // Build config response packet: [CQ][length(2)][config_data]
  uint8_t config_packet[512];
  uint16_t offset = 0;

  // Prefix "CQ" (Config Query Response)
  config_packet[offset++] = 'C';
  config_packet[offset++] = 'Q';

  // Reserve 2 bytes for length (will fill later)
  uint16_t length_offset = offset;
  offset += 2;

  // Format: key=value separated by | for easy parsing

  // ==================== CAN CONFIG ====================
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "can_baud_rate=%lu|", g_can_config.baud_rate);

  const char *can_mode_str = (g_can_config.operating_mode == CAN_MODE_NORMAL)
                                 ? "NORMAL"
                                 : "LISTEN_ONLY";
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "can_mode=%s|", can_mode_str);

  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "can_whitelist_count=%d|", g_can_whitelist_count);

  // CAN whitelist (comma-separated)
  if (g_can_whitelist_count > 0) {
    offset += snprintf((char *)&config_packet[offset],
                       sizeof(config_packet) - offset, "can_whitelist=");
    for (uint16_t i = 0; i < g_can_whitelist_count && i < MAX_WHITELISTED_IDS;
         i++) {
      if (i > 0) {
        offset += snprintf((char *)&config_packet[offset],
                           sizeof(config_packet) - offset, ",");
      }
      offset += snprintf((char *)&config_packet[offset],
                         sizeof(config_packet) - offset, "0x%03X",
                         g_can_whitelist[i]);
    }
    config_packet[offset++] = '|';
  } else {
    offset += snprintf((char *)&config_packet[offset],
                       sizeof(config_packet) - offset, "can_whitelist=|");
  }

  // ==================== LORA TDMA CONFIG ====================
  const char *lora_role_str =
      (g_lora_handler_cfg.role == LORA_HANDLER_ROLE_GATEWAY) ? "GATEWAY"
                                                             : "NODE";
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_role=%s|", lora_role_str);

  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_node_id=0x%04X|", g_lora_handler_cfg.node_id);

  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_gateway_id=0x%04X|", g_lora_handler_cfg.gateway_id);

  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_num_slots=%u|", g_lora_handler_cfg.num_slots);

  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_my_slot=%u|", g_lora_handler_cfg.my_slot);

  offset += snprintf(
      (char *)&config_packet[offset], sizeof(config_packet) - offset,
      "lora_slot_duration_ms=%lu|", g_lora_handler_cfg.slot_duration_ms);

  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_crypto_key_len=%u|", g_lora_handler_crypto_key_len);

  // ==================== LORA E32 CONFIG ====================
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_baud=%d|", g_lora_e32_baud_rate);

  // Address High + Low
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_addh=0x%02X|", g_lora_e32_params.addh);
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_addl=0x%02X|", g_lora_e32_params.addl);

  // Speed config byte
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_sped=0x%02X|", g_lora_e32_params.sped);

  // Channel
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_chan=%u|", g_lora_e32_params.chan);

  // Option byte
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_option=0x%02X|", g_lora_e32_params.option);

  // Fill in the length (excluding prefix and length field itself)
  uint16_t data_length = offset - 4;
  config_packet[length_offset] = (data_length >> 8) & 0xFF;
  config_packet[length_offset + 1] = data_length & 0xFF;

  // Send back to WAN MCU
  wan_comm_status_t status =
      wan_comm_send_data(g_wan_handle, config_packet, offset);

  if (status == WAN_COMM_OK) {
    ESP_LOGI(TAG, "LAN config response sent to WAN MCU (%u bytes)", offset);
  } else {
    ESP_LOGE(TAG, "Failed to send LAN config response");
  }
}

// ===== Public API =====

esp_err_t mcu_wan_handler_start(void) {
  if (g_handler_running) {
    ESP_LOGW(TAG, "Handler already running");
    return ESP_OK;
  }

  if (setup_data_ready_gpio() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to setup data-ready GPIO");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Starting MCU WAN Handler (SPI Master - LAN Side)");

  // Initialize WAN communication (SPI Master)
  wan_comm_config_t wan_config = {.gpio_sck = 12,
                                  .gpio_cs = 10,
                                  .gpio_io0 = 11,
                                  .gpio_io1 = 13,
                                  .gpio_io2 = -1,
                                  .gpio_io3 = -1,
                                  .clock_speed_hz = 10000000,
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

  // Create uplink queue
  g_uplink_queue = xQueueCreate(UPLINK_QUEUE_SIZE, sizeof(uplink_item_t));
  if (g_uplink_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create uplink queue");
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  // Create RTC mutex
  g_rtc_mutex = xSemaphoreCreateMutex();

  // Create handler task
  g_handler_running = true;
  BaseType_t ret =
      xTaskCreate(mcu_wan_handler_task, "mcu_wan_task", MCU_WAN_TASK_STACK_SIZE,
                  NULL, MCU_WAN_TASK_PRIORITY, &g_task_handle);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task");
    vQueueDelete(g_uplink_queue);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "MCU WAN Handler started successfully");
  return ESP_OK;
}

esp_err_t mcu_wan_handler_stop(void) {
  if (!g_handler_running)
    return ESP_OK;

  ESP_LOGI(TAG, "Stopping MCU WAN Handler");
  g_handler_running = false;

  if (g_task_handle != NULL) {
    vTaskDelete(g_task_handle);
    g_task_handle = NULL;
  }

  if (g_uplink_queue != NULL) {
    vQueueDelete(g_uplink_queue);
    g_uplink_queue = NULL;
  }

  if (g_rtc_mutex != NULL) {
    vSemaphoreDelete(g_rtc_mutex);
    g_rtc_mutex = NULL;
  }

  if (g_wan_handle != NULL) {
    wan_comm_deinit(g_wan_handle);
    g_wan_handle = NULL;
  }

  return ESP_OK;
}

bool mcu_wan_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                            uint16_t len) {
  if (g_uplink_queue == NULL || data == NULL || len == 0) {
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

  ESP_LOGI(TAG, "Uplink queued from handler %d (%u bytes)", source_id, len);
  return true;
}

internet_status_t mcu_wan_handler_get_internet_status(void) {
  return g_internet_status;
}

esp_err_t mcu_wan_handler_get_rtc(char *buffer) {
  if (buffer == NULL)
    return ESP_ERR_INVALID_ARG;

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

void mcu_wan_handler_register_config_callback(void (*callback)(const uint8_t *,
                                                               uint16_t,
                                                               bool)) {
  g_config_callback = callback;
}

// ===== Main Task (Diagram 1 Implementation) =====

static void mcu_wan_handler_task(void *pvParameters) {
  ESP_LOGI(TAG, "MCU WAN Handler task started");

  // ========================================
  // PHASE 1: Handshake Loop (Every 1 second)
  // ========================================
  ESP_LOGI(TAG, "Phase 1: Handshake with WAN MCU");
  while (g_handler_running) {
    if (perform_handshake() == ESP_OK) {
      ESP_LOGI(TAG, "Handshake successful, entering Data Mode");
      break;
    }
    ESP_LOGW(TAG, "Handshake failed, retrying in 1s");
    vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_INTERVAL_MS));
  }

  // ========================================
  // PHASE 2: Data Mode (Main Loop)
  // ========================================
  ESP_LOGI(TAG, "Phase 2: Data Mode with GPIO handshake");
  TickType_t last_rtc_request = xTaskGetTickCount();
  uplink_item_t uplink_item;
  uint8_t rx_buffer[256];

  while (g_handler_running) {
    TickType_t now = xTaskGetTickCount();
    uint32_t notification_value = 0;
    // ===== Check A: GPIO Notification - Data Ready from WAN =====
    if (xTaskNotifyWait(0, NOTIFY_DATA_READY, &notification_value,
                        pdMS_TO_TICKS(100)) == pdTRUE) {

      if (notification_value & NOTIFY_DATA_READY) {
        ESP_LOGI(TAG, "Data-ready signal received from WAN MCU");
        g_data_ready_flag = false;
        uint8_t dq_cmd[2] = {'D', 'Q'};
        wan_comm_send_command(g_wan_handle, dq_cmd, sizeof(dq_cmd));
        vTaskDelay(pdMS_TO_TICKS(50));
        // Poll data immediately
        wan_comm_status_t comm_status =
            wan_comm_request_data(g_wan_handle, rx_buffer, sizeof(rx_buffer));
        ESP_LOGI(TAG,
                 "Polled data: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X, "
                 "checking type...",
                 rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);

        if (comm_status == WAN_COMM_OK && rx_buffer[0] == 'D' &&
            rx_buffer[1] == 'T') {
          // Data Packet received: [DT][handler_type(3)][length(2)][payload]
          uint8_t handler_type[4] = {rx_buffer[2], rx_buffer[3], rx_buffer[4],
                                     '\0'};
          uint16_t payload_len = (rx_buffer[5] << 8) | rx_buffer[6];
          handler_id_t target_id = string_to_handler_id(handler_type);

          ESP_LOGI(TAG, "Downlink received: handler=%s, len=%u", handler_type,
                   payload_len);

          // Dispatch to appropriate handler
          dispatch_downlink_to_handler(
              target_id, &rx_buffer[DATA_PACKET_HEADER_SIZE], payload_len);

        } else if (comm_status == WAN_COMM_OK && rx_buffer[0] == 'C' &&
                   rx_buffer[1] == 'F') {
          if (rx_buffer[2] == 'C' && rx_buffer[3] == 'Q') {
            ESP_LOGI(TAG, "Config query request received from WAN MCU");
            send_lan_config_response();
            continue;
          }
          // Config Packet received: [CF][length(2)][config_data]
          uint16_t config_len = (rx_buffer[2] << 8) | rx_buffer[3];
          bool is_fota =
              (config_len >= 4 && memcmp(&rx_buffer[4], "CFFW", 4) == 0);

          ESP_LOGI(TAG, "Config received: len=%u, FOTA=%d", config_len,
                   is_fota);

          if (g_config_callback != NULL) {
            g_config_callback(&rx_buffer[4], config_len, is_fota);
          }
        }
      }
    }

    // ===== Check B: Output Queue (LAN Handler Queue) =====
    if (xQueueReceive(g_uplink_queue, &uplink_item, 0) == pdTRUE) {
      ESP_LOGI(TAG, "Processing uplink from handler %d (%u bytes)",
               uplink_item.source_id, uplink_item.length);

      // Build data_packet_t with RTC timestamp
      uint8_t packet[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
      uint16_t packet_len = 0;
      build_data_packet(&uplink_item, packet, &packet_len);

      // Check Internet Status before sending
      if (g_internet_status == INTERNET_STATUS_ONLINE) {
        ack_type_t ack_result;
        esp_err_t send_result =
            send_data_to_wan(packet, packet_len, &ack_result);

        if (send_result == ESP_OK) {
          if (ack_result == ACK_TYPE_INTERNET_OK) {
            ESP_LOGI(TAG, "Uplink sent successfully (ACK+INTERNET_OK)");
          } else if (ack_result == ACK_TYPE_NO_INTERNET) {
            ESP_LOGW(TAG, "ACK received but NO_INTERNET, saving to SD");
            g_internet_status = INTERNET_STATUS_OFFLINE;
            save_to_sd_card(packet, packet_len);
          }
        } else {
          ESP_LOGW(TAG, "Send failed after retries, saving to SD");
          save_to_sd_card(packet, packet_len);
        }
      } else {
        // Internet offline, save immediately to SD card
        ESP_LOGW(TAG, "Internet offline, saving to SD card");
        save_to_sd_card(packet, packet_len);
      }
      continue;
    }

    // ===== Check C: RTC Periodic Timer (Every 1 second) =====
    if ((now - last_rtc_request) >= pdMS_TO_TICKS(RTC_REQUEST_INTERVAL_MS)) {
      if (request_rtc_and_status() == ESP_OK) {
        ESP_LOGI(TAG, "RTC and Internet status updated");
      }
      last_rtc_request = now;
    }

    // ===== Check C2: SD Card Backup + Internet OK =====
    if (sd_card_has_data() && g_internet_status == INTERNET_STATUS_ONLINE) {
      uint8_t sd_buffer[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
      uint16_t sd_length = 0;

      if (read_oldest_from_sd_card(sd_buffer, &sd_length) == ESP_OK &&
          sd_length > 0) {
        ESP_LOGI(TAG, "Retrying SD card data (%u bytes)", sd_length);
        ack_type_t ack_result;

        if (send_data_to_wan(sd_buffer, sd_length, &ack_result) == ESP_OK &&
            ack_result == ACK_TYPE_INTERNET_OK) {
          delete_oldest_from_sd_card();
          ESP_LOGI(TAG, "SD data sent successfully, deleted from card");
        } else {
          ESP_LOGW(TAG, "SD data send failed, will retry later");
        }
      }
    }

    // Small delay if no events
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ESP_LOGI(TAG, "MCU WAN Handler task exiting");
  vTaskDelete(NULL);
}

// ===== Handshake Implementation =====
static esp_err_t perform_handshake(void) {
  uint8_t handshake_ack[2] = {FRAME_TYPE_ACK, ACK_TYPE_HANDSHAKE};

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, handshake_ack, 2);
  if (status != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  // Wait for ACK response from WAN MCU
  vTaskDelay(pdMS_TO_TICKS(50));
  uint8_t response[16] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK && response[0] == FRAME_TYPE_ACK &&
      response[1] == ACK_TYPE_HANDSHAKE) {
    return ESP_OK;
  }

  return ESP_FAIL;
}

// ===== RTC Request Implementation =====
static esp_err_t request_rtc_and_status(void) {
  // Send RTC request: prefix "RT"
  uint8_t rtc_request[2] = {'R', 'T'};

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, rtc_request, 2);
  if (status != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  // Receive RTC response: [RT][rtc_string(20)][network_status(1)]
  vTaskDelay(pdMS_TO_TICKS(50));
  uint8_t response[32] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK && response[0] == 'R' && response[1] == 'T') {
    if (xSemaphoreTake(g_rtc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      memcpy(g_rtc_cache.rtc_string, &response[2], 19);
      g_rtc_cache.rtc_string[19] = '\0';
      g_rtc_cache.valid = true;
      xSemaphoreGive(g_rtc_mutex);
    }

    // Update internet status
    g_internet_status = (internet_status_t)response[22];

    ESP_LOGI(TAG, "RTC: %s, Internet: %s", g_rtc_cache.rtc_string,
             g_internet_status == INTERNET_STATUS_ONLINE ? "ONLINE"
                                                         : "OFFLINE");
    return ESP_OK;
  }

  return ESP_FAIL;
}

// ===== Send Data with Retry & ACK =====
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out) {
  for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {
    ESP_LOGI(TAG, "Transmit attempt %d/%d", retry + 1, MAX_RETRY_COUNT);

    wan_comm_status_t status = wan_comm_send_data(g_wan_handle, data, length);
    if (status != WAN_COMM_OK) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Wait for ACK: [ACK_TYPE][ACK_RECEIVED][INTERNET_STATUS]
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t ack_response[8] = {0};
    status =
        wan_comm_request_data(g_wan_handle, ack_response, sizeof(ack_response));

    if (status == WAN_COMM_OK && ack_response[0] == FRAME_TYPE_ACK) {
      uint8_t ack_type = ack_response[1];
      uint8_t internet_flag = ack_response[2];

      if (ack_type == ACK_TYPE_RECEIVED_OK) {
        *ack_out = (ack_type_t)internet_flag;
        return ESP_OK;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(ACK_TIMEOUT_MS));
  }

  ESP_LOGW(TAG, "Max retries reached");
  *ack_out = ACK_TYPE_TIMEOUT;
  return ESP_FAIL;
}

// ===== Build Data Packet =====
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len) {
  // Format: [DT][handler_type(3)][length(2)][rtc(19)][payload]
  uint8_t *p = packet;

  // Prefix "DT"
  *p++ = 'D';
  *p++ = 'T';

  // Handler type (3 bytes)
  const char *type_str = handler_id_to_string(item->source_id);
  memcpy(p, type_str, 3);
  p += 3;

  // Data length (2 bytes, big endian) - includes RTC + payload
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

// ===== Dispatch Downlink to Handler =====
static void dispatch_downlink_to_handler(handler_id_t target_id,
                                         const uint8_t *data, uint16_t length) {
  bool success = false;

  switch (target_id) {
  case HANDLER_CAN:
    success = can_handler_enqueue_downlink((uint8_t *)data, length);
    break;
  case HANDLER_LORA:
    success = lora_tdma_connect_enqueue_downlink((uint8_t *)data, length);
    break;
  case HANDLER_ZIGBEE:
    success = zigbee_nostack_connect_enqueue_downlink((uint8_t *)data, length);
    break;
  default:
    ESP_LOGW(TAG, "Unknown target handler: %d", target_id);
    return;
  }

  if (success) {
    ESP_LOGI(TAG, "Downlink dispatched to handler %d", target_id);
  } else {
    ESP_LOGW(TAG, "Failed to dispatch downlink to handler %d", target_id);
  }
}

// ===== Helper Functions =====
static const char *handler_id_to_string(handler_id_t id) {
  switch (id) {
  case HANDLER_CAN:
    return "CAN";
  case HANDLER_LORA:
    return "LOR";
  case HANDLER_ZIGBEE:
    return "ZIG";
  default:
    return "UNK";
  }
}

static handler_id_t string_to_handler_id(const uint8_t *type_str) {
  if (memcmp(type_str, "CAN", 3) == 0)
    return HANDLER_CAN;
  if (memcmp(type_str, "LOR", 3) == 0)
    return HANDLER_LORA;
  if (memcmp(type_str, "ZIG", 3) == 0)
    return HANDLER_ZIGBEE;
  return HANDLER_CAN; // Default
}

// ===== SD Card Stub Functions (TODO: Implement with actual driver) =====
static esp_err_t save_to_sd_card(const uint8_t *data, uint16_t length) {
  ESP_LOGI(TAG, "SD Card: Saving %u bytes (file_%lu)", length,
           g_sd_card_file_count);
  g_sd_card_file_count++;
  // TODO: Implement actual SD card write
  return ESP_OK;
}

static esp_err_t read_oldest_from_sd_card(uint8_t *buffer, uint16_t *length) {
  // TODO: Implement actual SD card read
  *length = 0;
  return ESP_ERR_NOT_FOUND;
}

static bool sd_card_has_data(void) { return (g_sd_card_file_count > 0); }

static void delete_oldest_from_sd_card(void) {
  if (g_sd_card_file_count > 0) {
    g_sd_card_file_count--;
  }
  ESP_LOGI(TAG, "SD Card: Deleted oldest file");
}
