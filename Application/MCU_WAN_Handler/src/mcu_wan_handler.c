/**
 * @file mcu_wan_handler.c
 * @brief MCU WAN Handler - LAN Side (SPI Master) - DUAL TASK ARCHITECTURE
 *
 * Architecture: 2 separate tasks for clean separation of concerns
 * 
 * TASK 1: DOWNLINK POLL TASK (High Priority - 7)
 *   - Handles GPIO ISR notifications IMMEDIATELY
 *   - When ISR triggers, this task wakes up and processes data from WAN
 *   - NO other operations - dedicated solely to polling downlink data
 *   - Uses binary semaphore for exclusive SPI access during poll
 *
 * TASK 2: UPLINK TASK (Lower Priority - 5)  
 *   - Handles uplink queue (data from LORA, RS485, CAN, Zigbee)
 *   - Handles RTC/Internet status requests
 *   - Handles SD card backup operations
 *   - Must acquire SPI semaphore before any operation
 *   - Yields to downlink task when semaphore not available
 */
#include "mcu_wan_handler.h"
#include "SDCard_comm.h"
#include "can_driver.h"
#include "can_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lora_e32_comm.h"
#include "lora_tdma_connect.h"
#include "lora_tdma_handler.h"
#include "rs485_handler.h"
#include "stack_handler.h"
#include "wan_comm.h"
#include "zigbee_nostack_connect.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MCU_WAN";
static const char *TAG_DL = "MCU_WAN_DL";  // Downlink task tag
static const char *TAG_UL = "MCU_WAN_UL";  // Uplink task tag

// ===== Configuration =====
#define DOWNLINK_TASK_STACK_SIZE 4096
#define DOWNLINK_TASK_PRIORITY   7        // HIGH priority - responds to GPIO immediately
#define UPLINK_TASK_STACK_SIZE   4096
#define UPLINK_TASK_PRIORITY     5        // Lower priority than downlink
#define UPLINK_QUEUE_SIZE 50
#define HANDSHAKE_INTERVAL_MS 1000
#define RTC_REQUEST_INTERVAL_MS 1000
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
static TaskHandle_t g_downlink_task_handle = NULL;   // High priority - GPIO handler
static TaskHandle_t g_uplink_task_handle = NULL;     // Lower priority - uplink/RTC/SD
static QueueHandle_t g_uplink_queue = NULL;
static SemaphoreHandle_t g_rtc_mutex = NULL;
static SemaphoreHandle_t g_spi_mutex = NULL;         // SPI bus exclusive access
static bool g_handler_running = false;
static volatile bool g_handshake_done = false;       // Shared flag for handshake completion

// RTC and Internet status
static internet_status_t g_internet_status = INTERNET_STATUS_OFFLINE;
static rtc_cache_t g_rtc_cache = {{0}, false};

// Config callback
static void (*g_config_callback)(const uint8_t *, uint16_t, bool) = NULL;

// ===== Forward Declarations =====
static void downlink_poll_task(void *pvParameters);
static void uplink_handler_task(void *pvParameters);
static esp_err_t perform_handshake(void);
static esp_err_t request_rtc_and_status(void);
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out);
static esp_err_t save_to_sd_card(const uint8_t *data, uint16_t length);
static esp_err_t read_oldest_from_sd_card(uint8_t *buffer, uint16_t *length);
static void delete_oldest_from_sd_card(void);
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len);
static void dispatch_downlink_to_handler(handler_id_t target_id,
                                         const uint8_t *data, uint16_t length);
static const char *handler_id_to_string(handler_id_t id);
static handler_id_t string_to_handler_id(const uint8_t *type_str);
static void send_ack_to_wan(ack_type_t ack_type);

// External downlink callbacks
extern bool can_handler_enqueue_downlink(uint8_t *data, uint16_t len);
extern bool lora_tdma_connect_enqueue_downlink(uint8_t *data, uint16_t len);
extern bool zigbee_nostack_connect_enqueue_downlink(uint8_t *data,
                                                    uint16_t len);

// ===== GPIO Handshake Configuration =====
// NOTE: GPIO8 conflicts with SD Card D0, use GPIO9 instead!
#define GPIO_DATA_READY_PIN 46
#define NOTIFY_DATA_READY (1 << 0)

// GPIO ISR Handler - Notifies DOWNLINK task directly
// NOTE: Cannot release mutex from ISR (mutex has priority inheritance)
// Downlink task will naturally preempt uplink due to higher priority
static void IRAM_ATTR gpio_data_ready_isr(void *arg) {
  BaseType_t xTaskWoken = pdFALSE;
  
  if (g_downlink_task_handle) {
    // Notify downlink task - it will wake up and preempt lower priority tasks
    xTaskNotifyFromISR(g_downlink_task_handle, NOTIFY_DATA_READY, eSetBits, &xTaskWoken);
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

  // Log initial GPIO level for debugging
  int initial_level = gpio_get_level(GPIO_DATA_READY_PIN);
  ESP_LOGI(TAG, "GPIO %d configured for data-ready notification (ISR enabled)", GPIO_DATA_READY_PIN);
  ESP_LOGI(TAG, "GPIO %d initial level: %d", GPIO_DATA_READY_PIN, initial_level);
  
  return ESP_OK;
}

/**
 * @brief Build and send LAN configuration response to WAN MCU
 * NOTE: Caller must hold g_spi_mutex
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

  // stack handlers configuration
  offset += snprintf((char *)&config_packet[offset],
                     sizeof(config_packet) - offset, "stack_1_type=%s|",
                     stack_handler_type_to_string(g_stack_1_type));

  offset += snprintf((char *)&config_packet[offset],
                     sizeof(config_packet) - offset, "stack_2_type=%s|",
                     stack_handler_type_to_string(g_stack_2_type));

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

  // Header byte
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "lora_e32_header=0x%02X|", g_lora_e32_params.head);
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

  // ==================== RS485 CONFIG ====================
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "rs485_baud_rate=%lu|", (unsigned long)g_rs485_baud_rate);

  // Fill in the length (excluding prefix and length field itself)
  uint16_t data_length = offset - 4;
  config_packet[length_offset] = (data_length >> 8) & 0xFF;
  config_packet[length_offset + 1] = data_length & 0xFF;

  // Send back to WAN MCU
  wan_comm_status_t status =
      wan_comm_send_data(g_wan_handle, config_packet, offset);

  if (status == WAN_COMM_OK) {
    ESP_LOGI(TAG_DL, "LAN config response sent to WAN MCU (%u bytes)", offset);
  } else {
    ESP_LOGE(TAG_DL, "Failed to send LAN config response");
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

  ESP_LOGI(TAG, "Starting MCU WAN Handler (SPI Master - LAN Side) - DUAL TASK MODE");
  
  // Initialize SD card for data backup
  sd_card_config_t sd_config = SD_CARD_CONFIG_DEFAULT();
  if (sd_card_init(&sd_config) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to initialize SD card, continuing without backup");
  }

  // Initialize WAN communication (SPI Master)
  wan_comm_config_t wan_config = {.gpio_sck = 12,
                                  .gpio_cs = 10,
                                  .gpio_io0 = 11,
                                  .gpio_io1 = 13,
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

  // Create uplink queue
  g_uplink_queue = xQueueCreate(UPLINK_QUEUE_SIZE, sizeof(uplink_item_t));
  if (g_uplink_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create uplink queue");
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  // Create RTC mutex
  g_rtc_mutex = xSemaphoreCreateMutex();
  if (g_rtc_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create RTC mutex");
    vQueueDelete(g_uplink_queue);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  // Create SPI mutex - CRITICAL for bus arbitration
  g_spi_mutex = xSemaphoreCreateMutex();
  if (g_spi_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create SPI mutex");
    vSemaphoreDelete(g_rtc_mutex);
    vQueueDelete(g_uplink_queue);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  g_handler_running = true;
  g_handshake_done = false;

  // Create DOWNLINK task first (HIGH PRIORITY) - handles GPIO ISR
  BaseType_t ret = xTaskCreate(downlink_poll_task, "mcu_wan_dl", 
                               DOWNLINK_TASK_STACK_SIZE, NULL, 
                               DOWNLINK_TASK_PRIORITY, &g_downlink_task_handle);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create downlink task");
    vSemaphoreDelete(g_spi_mutex);
    vSemaphoreDelete(g_rtc_mutex);
    vQueueDelete(g_uplink_queue);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  // Create UPLINK task (LOWER PRIORITY) - handles uplink/RTC/SD
  ret = xTaskCreate(uplink_handler_task, "mcu_wan_ul", 
                    UPLINK_TASK_STACK_SIZE, NULL, 
                    UPLINK_TASK_PRIORITY, &g_uplink_task_handle);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create uplink task");
    vTaskDelete(g_downlink_task_handle);
    vSemaphoreDelete(g_spi_mutex);
    vSemaphoreDelete(g_rtc_mutex);
    vQueueDelete(g_uplink_queue);
    wan_comm_deinit(g_wan_handle);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "MCU WAN Handler started successfully (2 tasks)");
  ESP_LOGI(TAG, "  - Downlink task: priority %d (GPIO handler)", DOWNLINK_TASK_PRIORITY);
  ESP_LOGI(TAG, "  - Uplink task: priority %d (uplink/RTC/SD)", UPLINK_TASK_PRIORITY);
  return ESP_OK;
}

esp_err_t mcu_wan_handler_stop(void) {
  if (!g_handler_running)
    return ESP_OK;

  ESP_LOGI(TAG, "Stopping MCU WAN Handler");
  g_handler_running = false;

  if (g_downlink_task_handle != NULL) {
    vTaskDelete(g_downlink_task_handle);
    g_downlink_task_handle = NULL;
  }

  if (g_uplink_task_handle != NULL) {
    vTaskDelete(g_uplink_task_handle);
    g_uplink_task_handle = NULL;
  }

  if (g_uplink_queue != NULL) {
    vQueueDelete(g_uplink_queue);
    g_uplink_queue = NULL;
  }

  if (g_rtc_mutex != NULL) {
    vSemaphoreDelete(g_rtc_mutex);
    g_rtc_mutex = NULL;
  }

  if (g_spi_mutex != NULL) {
    vSemaphoreDelete(g_spi_mutex);
    g_spi_mutex = NULL;
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

// ===== Stack Handler Starter =====
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

// ============================================================================
// TASK 1: DOWNLINK POLL TASK (HIGH PRIORITY)
// - Handles GPIO ISR notifications
// - Takes SPI mutex immediately when notified
// - Processes all downlink data before releasing mutex
// ============================================================================
static void downlink_poll_task(void *pvParameters) {
  ESP_LOGI(TAG_DL, "Downlink Poll Task started (Priority %d)", DOWNLINK_TASK_PRIORITY);
  
  uint8_t rx_buffer[256];
  uint32_t notification_value = 0;

  // Wait for handshake to complete (done by uplink task)
  while (!g_handshake_done && g_handler_running) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG_DL, "Handshake complete, entering poll loop");

  while (g_handler_running) {
    // BLOCK here waiting for GPIO ISR notification
    // When ISR fires, this task wakes up IMMEDIATELY due to high priority
    if (xTaskNotifyWait(0, NOTIFY_DATA_READY, &notification_value, portMAX_DELAY) == pdTRUE) {
      
      if (notification_value & NOTIFY_DATA_READY) {
        ESP_LOGI(TAG_DL, ">>> GPIO ISR triggered - acquiring SPI bus <<<");
        
        // Take SPI mutex - blocks uplink task from using SPI
        if (xSemaphoreTake(g_spi_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
          ESP_LOGI(TAG_DL, "SPI bus acquired");
          
          // Poll for response - retry up to 5 times, resend DQ each time
          bool got_valid_response = false;
          uint8_t dq_cmd[2] = {'D', 'Q'};
          
          for (int retry = 0; retry < 10 && !got_valid_response; retry++) {
            // Send DQ (Data Query) command EACH retry to ensure Slave receives it
            ESP_LOGI(TAG_DL, "Sending DQ command (attempt %d/10)", retry + 1);
            wan_comm_send_command(g_wan_handle, dq_cmd, sizeof(dq_cmd));
            
            // Wait for Slave to receive DQ and load TX buffer with response
            vTaskDelay(pdMS_TO_TICKS(150));
            
            // Poll for response from Slave
            memset(rx_buffer, 0, sizeof(rx_buffer));
            wan_comm_status_t comm_status =
                wan_comm_request_data(g_wan_handle, rx_buffer, sizeof(rx_buffer));
            
            ESP_LOGI(TAG_DL, "Poll attempt %d: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X",
                     retry + 1, rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);

            if (comm_status == WAN_COMM_OK && rx_buffer[0] == 'D' && rx_buffer[1] == 'T') {
              // ===== DATA PACKET received =====
              uint8_t handler_type[4] = {rx_buffer[2], rx_buffer[3], rx_buffer[4], '\0'};
              uint16_t payload_len = (rx_buffer[5] << 8) | rx_buffer[6];
              handler_id_t target_id = string_to_handler_id(handler_type);

              ESP_LOGI(TAG_DL, "✓ Downlink DATA: handler=%s, len=%u", handler_type, payload_len);
              send_ack_to_wan(ACK_TYPE_RECEIVED_OK);
              dispatch_downlink_to_handler(target_id, &rx_buffer[DATA_PACKET_HEADER_SIZE], payload_len);
              got_valid_response = true;

            } else if (comm_status == WAN_COMM_OK && rx_buffer[0] == 'C' && rx_buffer[1] == 'F') {
              if (rx_buffer[2] == 'C' && rx_buffer[3] == 'Q') {
                // ===== CONFIG QUERY request =====
                ESP_LOGI(TAG_DL, "✓ Config query request from WAN MCU");
                send_lan_config_response();
                got_valid_response = true;
              } else {
                // ===== CONFIG PACKET received =====
                uint16_t config_len = (rx_buffer[2] << 8) | rx_buffer[3];
                bool is_fota = (config_len >= 4 && memcmp(&rx_buffer[4], "CFFW", 4) == 0);
                ESP_LOGI(TAG_DL, "✓ Config received: len=%u, FOTA=%d", config_len, is_fota);
                if (g_config_callback != NULL) {
                  g_config_callback(&rx_buffer[4], config_len, is_fota);
                }
                got_valid_response = true;
              }
            } else {
              // Invalid response - will resend DQ on next retry
              ESP_LOGD(TAG_DL, "No valid response, will retry");
            }
          }
          
          if (!got_valid_response) {
            ESP_LOGW(TAG_DL, "Failed to get valid response after 5 retries");
          }
          
          // Release SPI mutex - uplink task can now use SPI
          xSemaphoreGive(g_spi_mutex);
          ESP_LOGI(TAG_DL, ">>> SPI bus released <<<");
          
        } else {
          ESP_LOGE(TAG_DL, "Failed to acquire SPI mutex!");
        }
      }
    }
  }

  ESP_LOGI(TAG_DL, "Downlink Poll Task exiting");
  vTaskDelete(NULL);
}

// ============================================================================
// TASK 2: UPLINK HANDLER TASK (LOWER PRIORITY)
// - Performs initial handshake
// - Handles uplink queue
// - Handles RTC requests
// - Handles SD card backup
// - Must acquire SPI mutex for ALL operations
// ============================================================================
static void uplink_handler_task(void *pvParameters) {
  ESP_LOGI(TAG_UL, "Uplink Handler Task started (Priority %d)", UPLINK_TASK_PRIORITY);

  // ========================================
  // PHASE 1: Handshake Loop (Every 1 second)
  // ========================================
  ESP_LOGI(TAG_UL, "Phase 1: Handshake with WAN MCU");
  while (g_handler_running && !g_handshake_done) {
    // Take SPI mutex for handshake
    if (xSemaphoreTake(g_spi_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (perform_handshake() == ESP_OK) {
        g_handshake_done = true;
        ESP_LOGI(TAG_UL, "Handshake successful!");
        xSemaphoreGive(g_spi_mutex);
        break;
      }
      xSemaphoreGive(g_spi_mutex);
    }
    ESP_LOGW(TAG_UL, "Handshake failed, retrying in 1s");
    vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_INTERVAL_MS));
  }

  // Start stack handlers
  stack_handler_start(g_stack_1_type);
  stack_handler_start(g_stack_2_type);

  // ========================================
  // PHASE 2: Uplink/RTC/SD Loop
  // ========================================
  ESP_LOGI(TAG_UL, "Phase 2: Uplink processing loop");
  TickType_t last_rtc_request = xTaskGetTickCount();
  uplink_item_t uplink_item;

  while (g_handler_running) {
    TickType_t now = xTaskGetTickCount();
    
    // Try to acquire SPI mutex (non-blocking or short timeout)
    // If downlink task has it, we'll skip and try next iteration
    if (xSemaphoreTake(g_spi_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

      // ===== Check A: Uplink Queue =====
      if (xQueueReceive(g_uplink_queue, &uplink_item, 0) == pdTRUE) {
        ESP_LOGI(TAG_UL, "Processing uplink from handler %d (%u bytes)",
                 uplink_item.source_id, uplink_item.length);

        uint8_t packet[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
        uint16_t packet_len = 0;
        build_data_packet(&uplink_item, packet, &packet_len);

        if (g_internet_status == INTERNET_STATUS_ONLINE) {
          ack_type_t ack_result;
          esp_err_t send_result = send_data_to_wan(packet, packet_len, &ack_result);

          if (send_result == ESP_OK) {
            if (ack_result == ACK_TYPE_INTERNET_OK) {
              ESP_LOGI(TAG_UL, "Uplink sent successfully (ACK+INTERNET_OK)");
            } else if (ack_result == ACK_TYPE_NO_INTERNET) {
              ESP_LOGW(TAG_UL, "ACK received but NO_INTERNET, saving to SD");
              g_internet_status = INTERNET_STATUS_OFFLINE;
              save_to_sd_card(packet, packet_len);
            }
          } else {
            ESP_LOGW(TAG_UL, "Send failed after retries, saving to SD");
            save_to_sd_card(packet, packet_len);
          }
        } else {
          ESP_LOGW(TAG_UL, "Internet offline, saving to SD card");
          save_to_sd_card(packet, packet_len);
        }
      }

      // ===== Check B: SD Card Backup + Internet OK =====
      if (sd_card_has_data() && g_internet_status == INTERNET_STATUS_ONLINE) {
        uint8_t sd_buffer[MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20];
        uint16_t sd_length = 0;

        if (read_oldest_from_sd_card(sd_buffer, &sd_length) == ESP_OK && sd_length > 0) {
          ESP_LOGI(TAG_UL, "Retrying SD card data (%u bytes)", sd_length);
          ack_type_t ack_result;
          esp_err_t send_result = send_data_to_wan(sd_buffer, sd_length, &ack_result);

          if (send_result == ESP_OK) {
            if (ack_result == ACK_TYPE_INTERNET_OK) {
              delete_oldest_from_sd_card();
              ESP_LOGI(TAG_UL, "SD data sent successfully, deleted from card");
            } else if (ack_result == ACK_TYPE_NO_INTERNET) {
              // CRITICAL FIX: Update internet status to prevent retry loop
              g_internet_status = INTERNET_STATUS_OFFLINE;
              ESP_LOGW(TAG_UL, "SD data ACK: NO_INTERNET, waiting for reconnect");
            }
          } else {
            ESP_LOGW(TAG_UL, "SD data send failed, will retry later");
          }
        }
      }

      // ===== Check C: RTC Periodic Timer (Every 1 second) =====
      if ((now - last_rtc_request) >= pdMS_TO_TICKS(RTC_REQUEST_INTERVAL_MS)) {
        if (request_rtc_and_status() == ESP_OK) {
          ESP_LOGD(TAG_UL, "RTC and Internet status updated");
        }
        last_rtc_request = now;
      }

      // Release SPI mutex
      xSemaphoreGive(g_spi_mutex);
    }

    // Small delay before next iteration
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ESP_LOGI(TAG_UL, "Uplink Handler Task exiting");
  vTaskDelete(NULL);
}

// ===== Handshake Implementation =====
// NOTE: Caller must hold g_spi_mutex
static esp_err_t perform_handshake(void) {
  uint8_t handshake_ack[2] = {FRAME_TYPE_ACK, ACK_TYPE_HANDSHAKE};

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, handshake_ack, 2);
  if (status != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  // Ensure command is transmitted immediately before waiting for response
  if (wan_comm_flush_dma_buffer(g_wan_handle) != WAN_COMM_OK) {
    return ESP_FAIL;
  }

  // Wait for ACK response from WAN MCU
  vTaskDelay(pdMS_TO_TICKS(100));
  uint8_t response[16] = {0};
  status = wan_comm_request_data(g_wan_handle, response, sizeof(response));

  if (status == WAN_COMM_OK && response[0] == FRAME_TYPE_ACK &&
      response[1] == ACK_TYPE_HANDSHAKE) {
    return ESP_OK;
  }

  return ESP_FAIL;
}

// ===== RTC Request Implementation =====
// NOTE: Caller must hold g_spi_mutex
static esp_err_t request_rtc_and_status(void) {
  ESP_LOGD(TAG_UL, "Requesting RTC and Internet status from WAN MCU");
  uint8_t rtc_request[2] = {'R', 'T'};

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, rtc_request, 2);
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
    if (xSemaphoreTake(g_rtc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      memcpy(g_rtc_cache.rtc_string, &response[2], 19);
      g_rtc_cache.rtc_string[19] = '\0';
      g_rtc_cache.valid = true;
      xSemaphoreGive(g_rtc_mutex);
    }

    g_internet_status = (internet_status_t)response[22];

    ESP_LOGI(TAG_UL, "RTC: %s, Internet: %s", g_rtc_cache.rtc_string,
             g_internet_status == INTERNET_STATUS_ONLINE ? "ONLINE" : "OFFLINE");
    return ESP_OK;
  }

  return ESP_FAIL;
}

// ===== Send Data with Retry & ACK =====
// NOTE: Caller must hold g_spi_mutex
static esp_err_t send_data_to_wan(const uint8_t *data, uint16_t length,
                                  ack_type_t *ack_out) {
  if (!data || length == 0 || !ack_out)
    return ESP_ERR_INVALID_ARG;

  for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {
    ESP_LOGI(TAG_UL, "Transmit attempt %d/%d", retry + 1, MAX_RETRY_COUNT);

    wan_comm_status_t status = wan_comm_send_data(g_wan_handle, data, length);
    if (status != WAN_COMM_OK) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Poll ACK within ACK_TIMEOUT_MS
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(ACK_TIMEOUT_MS);

    uint32_t backoff_ms = 0;
    const uint32_t max_backoff_ms = 10;

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
      uint8_t ack_response[8] = {0};
      status = wan_comm_request_data(g_wan_handle, ack_response,
                                     sizeof(ack_response));

      if (status == WAN_COMM_OK && ack_response[0] == FRAME_TYPE_ACK &&
          ack_response[1] == ACK_TYPE_RECEIVED_OK) {
        *ack_out = (ack_type_t)ack_response[2];
        ESP_LOGI(TAG_UL, "ACK received");
        return ESP_OK;
      }

      if (backoff_ms == 0) {
        taskYIELD();
        backoff_ms = 1;
      } else {
        vTaskDelay(pdMS_TO_TICKS(backoff_ms));
        if (backoff_ms < max_backoff_ms) {
          backoff_ms <<= 1;
          if (backoff_ms > max_backoff_ms)
            backoff_ms = max_backoff_ms;
        }
      }
    }

    ESP_LOGW(TAG_UL, "ACK timeout on attempt %d", retry + 1);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  ESP_LOGW(TAG_UL, "Max retries reached");
  *ack_out = ACK_TYPE_TIMEOUT;
  return ESP_FAIL;
}

// ===== Build Data Packet =====
static void build_data_packet(const uplink_item_t *item, uint8_t *packet,
                              uint16_t *packet_len) {
  uint8_t *p = packet;

  *p++ = 'D';
  *p++ = 'T';

  const char *type_str = handler_id_to_string(item->source_id);
  memcpy(p, type_str, 3);
  p += 3;

  uint16_t total_data_len = 19 + item->length;
  *p++ = (total_data_len >> 8) & 0xFF;
  *p++ = total_data_len & 0xFF;

  memcpy(p, item->rtc_timestamp, 19);
  p += 19;

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
  case HANDLER_RS485:
    success = rs485_handler_enqueue_downlink((uint8_t *)data, length);
    break;
  default:
    ESP_LOGW(TAG_DL, "Unknown target handler: %d", target_id);
    return;
  }

  if (success) {
    ESP_LOGI(TAG_DL, "Downlink dispatched to handler %d", target_id);
  } else {
    ESP_LOGW(TAG_DL, "Failed to dispatch downlink to handler %d", target_id);
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
  case HANDLER_RS485:
    return "RS4";
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
  if (memcmp(type_str, "RS4", 3) == 0)
    return HANDLER_RS485;
  return HANDLER_UNKNOWN;
}

/**
 * @brief Send ACK back to WAN MCU after receiving downlink data
 * NOTE: Caller must hold g_spi_mutex
 */
static void send_ack_to_wan(ack_type_t ack_type) {
  uint8_t ack_packet[2];
  ack_packet[0] = FRAME_TYPE_ACK;
  ack_packet[1] = ack_type;

  wan_comm_status_t status = wan_comm_send_command(g_wan_handle, ack_packet, sizeof(ack_packet));

  if (status == WAN_COMM_OK) {
    ESP_LOGI(TAG_DL, "✓ ACK sent to WAN MCU: type=0x%02X", ack_type);
  } else {
    ESP_LOGE(TAG_DL, "✗ Failed to send ACK to WAN MCU");
  }
}

// ===== SD Card Functions =====
static esp_err_t save_to_sd_card(const uint8_t *data, uint16_t length) {
  return sd_card_save(data, length);
}

static esp_err_t read_oldest_from_sd_card(uint8_t *buffer, uint16_t *length) {
  return sd_card_read_oldest(buffer, length,
                             MAX_PAYLOAD_SIZE + DATA_PACKET_HEADER_SIZE + 20);
}

static void delete_oldest_from_sd_card(void) { sd_card_delete_oldest(); }
