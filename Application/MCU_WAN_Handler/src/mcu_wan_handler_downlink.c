#include "config_global.h"
#include "esp_log.h"
#include "fota_lan_handler.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// #include "ble_handler_task.h"
#include "rs485_handler.h"
#include "storage_handler.h"
#include "wan_comm.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WAN_DL";

//
// CONFIGURATION
//

#define DOWNLINK_TASK_STACK_SIZE 1024 * 16
#define DOWNLINK_TASK_PRIORITY 7 // HIGH - ISR response
#define DQ_RETRY_INTERVAL_MS 50  // (was 150ms)
#define DQ_RETRY_COUNT 10
#define DQ_RESPONSE_SIZE 8192  // Must be >= max config JSON size (~2KB+)
#define GPIO_ISR_TIMEOUT_MS 5000 // Max wait for ISR

//
// EXTERNAL REFERENCES
//
extern wan_comm_handle_t g_wan_handle;
extern SemaphoreHandle_t g_qspi_mutex;
extern volatile bool g_handshake_done;
extern bool g_handler_running;
extern void (*g_config_callback)(const uint8_t *, uint16_t, bool);

//
// MODULE STATE
//

static TaskHandle_t g_downlink_task_handle = NULL;
static uint32_t g_isr_trigger_count = 0;
static uint32_t g_dq_success_count = 0;
static uint32_t g_dq_fail_count = 0;

//
// FORWARD DECLARATIONS
//

static void downlink_poll_task(void *pvParameters);
static bool poll_wan_with_retry(uint8_t *rx_buffer, size_t buffer_size);
static void dispatch_downlink_to_handler(handler_id_t target_id,
                                         const uint8_t *data, uint16_t length);
static void send_ack_to_wan(ack_type_t ack_type);
static void send_lan_config_response(void);
static handler_id_t string_to_handler_id(const uint8_t *type_str);

//
// GPIO ISR CALLBACK
//

/**
 * @brief Data-ready ISR callback from wan_comm
 * Notifies downlink task immediately (<5ms response requirement)
 */
static void IRAM_ATTR gpio_data_ready_callback(void *user_arg) {
  BaseType_t xTaskWoken = pdFALSE;

  if (g_downlink_task_handle) {
    // Notify downlink task - it will wake up and preempt lower priority tasks
    xTaskNotifyFromISR(g_downlink_task_handle, 1, eSetBits, &xTaskWoken);
    g_isr_trigger_count++;
  }

  if (xTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

//
// PUBLIC API
//

esp_err_t mcu_wan_handler_start_downlink_task(void) {
  if (g_downlink_task_handle != NULL) {
    ESP_LOGW(TAG, "Downlink task already running");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "============================================");
  ESP_LOGI(TAG, "Starting Downlink Poll Task (Priority %d)",
           DOWNLINK_TASK_PRIORITY);
  ESP_LOGI(TAG, "============================================");

  // Register ISR callback
  if (wan_comm_register_data_ready_callback(
          g_wan_handle, gpio_data_ready_callback, NULL) != WAN_COMM_OK) {
    ESP_LOGE(TAG, "Failed to register data-ready callback");
    return ESP_FAIL;
  }

  // Create task
  BaseType_t ret =
      xTaskCreate(downlink_poll_task, "wan_downlink", DOWNLINK_TASK_STACK_SIZE,
                  NULL, DOWNLINK_TASK_PRIORITY, &g_downlink_task_handle);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create downlink task");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Downlink task started - waiting for GPIO ISR");
  return ESP_OK;
}

void mcu_wan_handler_stop_downlink_task(void) {
  if (g_downlink_task_handle) {
    vTaskDelete(g_downlink_task_handle);
    g_downlink_task_handle = NULL;

    ESP_LOGI(TAG, "Downlink task stopped");
    ESP_LOGI(TAG, "Statistics: ISR=%lu, DQ_OK=%lu, DQ_FAIL=%lu",
             g_isr_trigger_count, g_dq_success_count, g_dq_fail_count);
  }
}

//
// DOWNLINK POLL TASK
//

/**
 * @brief High-priority downlink task
 *
 * Flow:
 * 1. Block on GPIO ISR notification (portMAX_DELAY)
 * 2. Acquire SPI mutex (preempts uplink task)
 * 3. Poll WAN with DQ retry (10×50ms)
 * 4. Dispatch received data
 * 5. Release mutex and return to blocking
 */
static void downlink_poll_task(void *pvParameters) {
  ESP_LOGI(TAG, "Downlink Poll Task started (Priority %d)",
           DOWNLINK_TASK_PRIORITY);

  // Heap-allocate to avoid 4KB stack pressure on this high-priority task
  uint8_t *rx_buffer = (uint8_t *)malloc(DQ_RESPONSE_SIZE);
  if (rx_buffer == NULL) {
    ESP_LOGE(TAG, "Failed to allocate rx_buffer (%d bytes) - task exiting!", DQ_RESPONSE_SIZE);
    vTaskDelete(NULL);
    return;
  }
  uint32_t notification_value = 0;

  // Wait for handshake to complete (done by uplink task)
  while (!g_handshake_done && g_handler_running) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG, "Handshake complete - entering poll loop");
  ESP_LOGI(TAG, "ISR → Mutex → DQ Retry (10×%dms) → Dispatch → Release",
           DQ_RETRY_INTERVAL_MS);

  while (g_handler_running) {
    // BLOCK HERE waiting for GPIO ISR notification
    // When ISR fires, this task wakes up IMMEDIATELY due to high priority
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, portMAX_DELAY) ==
        pdTRUE) {

      ESP_LOGI(TAG, "GPIO ISR triggered (#%lu) - acquiring SPI bus",
               g_isr_trigger_count);

      // Take SPI mutex - blocks uplink task from using SPI
      if (xSemaphoreTake(g_qspi_mutex, pdMS_TO_TICKS(GPIO_ISR_TIMEOUT_MS)) ==
          pdTRUE) {

        TickType_t start = xTaskGetTickCount();

        // Poll for response - retry up to 10 times with 50ms interval
        bool got_valid_response =
            poll_wan_with_retry(rx_buffer, DQ_RESPONSE_SIZE);

        TickType_t elapsed = xTaskGetTickCount() - start;

        if (got_valid_response) {
          g_dq_success_count++;
          ESP_LOGI(TAG, "DQ success in %lums (total=%lu)",
                   elapsed * portTICK_PERIOD_MS, g_dq_success_count);
        } else {
          g_dq_fail_count++;
          ESP_LOGW(TAG, "DQ failed after %lums (total=%lu)",
                   elapsed * portTICK_PERIOD_MS, g_dq_fail_count);
        }

        // Release SPI mutex - uplink task can now use SPI
        xSemaphoreGive(g_qspi_mutex);

        ESP_LOGI(TAG, "SPI bus released");

      } else {
        ESP_LOGE(TAG, "Failed to acquire SPI mutex (timeout=%dms)",
                 GPIO_ISR_TIMEOUT_MS);
      }
    }
  }

  ESP_LOGI(TAG, "Downlink Poll Task exiting");
  free(rx_buffer);
  vTaskDelete(NULL);
}

//
// INTERNAL FUNCTIONS
//

/**
 * @brief Poll WAN MCU with DQ retry mechanism
 * 10 retries × 50ms interval
 *
 * @param rx_buffer Buffer to store response
 * @param buffer_size Buffer size
 * @return true if valid response received
 */
static bool poll_wan_with_retry(uint8_t *rx_buffer, size_t buffer_size) {
  uint8_t dq_cmd[2] = {'D', 'Q'};
  bool got_valid_response = false;

  for (int retry = 0; retry < DQ_RETRY_COUNT && !got_valid_response; retry++) {

    // Send DQ command EACH retry to ensure Slave receives it
    ESP_LOGI(TAG, "Sending DQ command (attempt %d/%d)", retry + 1,
             DQ_RETRY_COUNT);
    wan_comm_send_command(g_wan_handle, dq_cmd, sizeof(dq_cmd));

    // Small delay for slave to load TX buffer (optimized from 150ms → 50ms)
    vTaskDelay(pdMS_TO_TICKS(DQ_RETRY_INTERVAL_MS));

    // Poll for response from Slave
    memset(rx_buffer, 0, buffer_size);
    wan_comm_status_t comm_status =
        wan_comm_request_data(g_wan_handle, rx_buffer, buffer_size);

    if (comm_status == WAN_COMM_OK) {

      // Check for DATA packet: [D][T][handler_type(3)][length(2)][payload]
      if (rx_buffer[0] == 'D' && rx_buffer[1] == 'T') {

        uint8_t handler_type[4] = {rx_buffer[2], rx_buffer[3], rx_buffer[4], 0};
        uint16_t payload_len = (rx_buffer[5] << 8) | rx_buffer[6];
        handler_id_t target_id = string_to_handler_id(handler_type);

        ESP_LOGI(TAG, "Downlink DATA: handler=%s, len=%u", handler_type,
                 payload_len);

        send_ack_to_wan(ACK_TYPE_RECEIVED_OK);
        dispatch_downlink_to_handler(
            target_id, &rx_buffer[DATA_PACKET_HEADER_SIZE], payload_len);

        got_valid_response = true;

      }
      // Check for CONFIG packet: [C][F][...]
      else if (rx_buffer[0] == 'C' && rx_buffer[1] == 'F') {

        // Config query request: [C][F][C][Q]
        if (rx_buffer[2] == 'C' && rx_buffer[3] == 'Q') {
          ESP_LOGI(TAG, "Config query request from WAN MCU");
          send_lan_config_response();
          got_valid_response = true;
        }
        // Config packet: [C][F][length(2)][data]
        else {
          uint16_t config_len = (rx_buffer[2] << 8) | rx_buffer[3];
          bool is_fota =
              (config_len >= 4) && (memcmp(&rx_buffer[4], "CFFW", 4) == 0);

          ESP_LOGI(TAG, "=== CONFIG PACKET RECEIVED ===");
          ESP_LOGI(TAG, "  Header : [0x%02X 0x%02X] = 'CF'", rx_buffer[0], rx_buffer[1]);
          ESP_LOGI(TAG, "  config_len field (rx[2..3]): 0x%02X 0x%02X = %u bytes",
                   rx_buffer[2], rx_buffer[3], config_len);
          ESP_LOGI(TAG, "  is_fota : %d", is_fota);
          ESP_LOGI(TAG, "  buffer_size available: %d bytes", DQ_RESPONSE_SIZE);

          // Bounds check: ensure config_len fits in our buffer
          if (config_len > DQ_RESPONSE_SIZE - 4) {
            ESP_LOGE(TAG, "  CONFIG TOO LARGE: %u > %d (buffer overflow!)",
                     config_len, DQ_RESPONSE_SIZE - 4);
            ESP_LOGE(TAG, "  Increase DQ_RESPONSE_SIZE or split config!");
            got_valid_response = true;  // Don't retry, it won't help
            break;
          }

          if (g_config_callback != NULL) {
            g_config_callback(&rx_buffer[4], config_len, is_fota);
          }

          got_valid_response = true;
        }
      }
      // Ignore polling packet (CF + zeros or all zeros)
      else if ((rx_buffer[0] == 'C' && rx_buffer[1] == 'F' &&
                rx_buffer[2] == 0x00 && rx_buffer[3] == 0x00) ||
               (rx_buffer[0] == 0x00 && rx_buffer[1] == 0x00)) {
        ESP_LOGI(TAG, "Polling packet - no data pending");
        got_valid_response = true; // Not an error, just no data
      }
      // Invalid response - will retry
      else {
        ESP_LOGI(TAG, "Invalid response [0]=0x%02X [1]=0x%02X, retry %d/%d",
                 rx_buffer[0], rx_buffer[1], retry + 1, DQ_RETRY_COUNT);
      }
    } else {
      ESP_LOGI(TAG, "DQ request failed: %d, retry %d/%d", comm_status,
               retry + 1, DQ_RETRY_COUNT);
    }
  }

  if (!got_valid_response) {
    ESP_LOGW(TAG, "DQ retry exhausted after %d attempts", DQ_RETRY_COUNT);
  }

  return got_valid_response;
}

/**
 * @brief Send ACK back to WAN MCU
 * NOTE: Caller must hold g_qspi_mutex
 */
static void send_ack_to_wan(ack_type_t ack_type) {
  uint8_t ack_packet[2];
  ack_packet[0] = FRAME_TYPE_ACK;
  ack_packet[1] = ack_type;

  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, ack_packet, sizeof(ack_packet));

  if (status == WAN_COMM_OK) {
    ESP_LOGI(TAG, "ACK sent to WAN MCU: type=0x%02X", ack_type);
  } else {
    ESP_LOGE(TAG, "Failed to send ACK to WAN MCU");
  }
}

/**
 * @brief Build and send LAN configuration response to WAN MCU
 * NOTE: Caller must hold g_qspi_mutex
 * 
 * New simplified format for Module Base Setting architecture (BLE trial):
 * - g_stack_1_id: "002" (BLE module) or "000" (no module)
 * - g_stack_2_id: "000" (no module) 
 * - rs485_baudrate: If using RS485 module
 * - stack1_json: JSON config for stack 1 (if configured)
 * - stack2_json: JSON config for stack 2 (if configured)
 */
static void send_lan_config_response(void) {
  // Build config response packet: [C][Q][length(2)][config_data]
  uint8_t config_packet[4096];  // Increased buffer for JSON configs
  uint16_t offset = 0;

  // Prefix: CQ (Config Query Response)
  config_packet[offset++] = 'C';
  config_packet[offset++] = 'Q';

  // Reserve 2 bytes for length (will fill later)
  uint16_t length_offset = offset;
  offset += 2;

  // Format: key=value|key=value|...

  // STACK MODULE IDs (from config_global)
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "stack1_id=%s|", config_get_stack_1_id());
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "stack2_id=%s|", config_get_stack_2_id());

  // RS485 CONFIG (if any stack is RS485)
  offset +=
      snprintf((char *)&config_packet[offset], sizeof(config_packet) - offset,
               "rs485_baudrate=%lu|", (unsigned long)config_get_rs485_baudrate());

  // STACK 1 JSON CONFIG (if configured)
  uint16_t stack1_json_len = 0;
  const char* stack1_json = config_get_stack_1_json(&stack1_json_len);
  if (stack1_json_len > 0 && stack1_json_len < (sizeof(config_packet) - offset - 20)) {
    offset += snprintf((char *)&config_packet[offset], 
                      sizeof(config_packet) - offset,
                      "stack1_json_len=%u|", stack1_json_len);
    memcpy(&config_packet[offset], stack1_json, stack1_json_len);
    offset += stack1_json_len;
    config_packet[offset++] = '|';
  } else {
    offset += snprintf((char *)&config_packet[offset], 
                      sizeof(config_packet) - offset,
                      "stack1_json_len=0|");
  }

  // STACK 2 JSON CONFIG (if configured)
  uint16_t stack2_json_len = 0;
  const char* stack2_json = config_get_stack_2_json(&stack2_json_len);
  if (stack2_json_len > 0 && stack2_json_len < (sizeof(config_packet) - offset - 20)) {
    offset += snprintf((char *)&config_packet[offset], 
                      sizeof(config_packet) - offset,
                      "stack2_json_len=%u|", stack2_json_len);
    memcpy(&config_packet[offset], stack2_json, stack2_json_len);
    offset += stack2_json_len;
    config_packet[offset++] = '|';
  } else {
    offset += snprintf((char *)&config_packet[offset], 
                      sizeof(config_packet) - offset,
                      "stack2_json_len=0|");
  }

  // Fill in the length (excluding prefix and length field itself)
  uint16_t data_length = offset - 4;
  config_packet[length_offset] = (data_length >> 8) & 0xFF;
  config_packet[length_offset + 1] = data_length & 0xFF;

  // Send back to WAN MCU via CF frame
  wan_comm_status_t status =
      wan_comm_send_command(g_wan_handle, config_packet, offset);

  if (status == WAN_COMM_OK) {
    ESP_LOGI(TAG, "LAN config response sent to WAN MCU: %u bytes", offset);
    ESP_LOGI(TAG, "  Stack 1 ID: %s", config_get_stack_1_id());
    ESP_LOGI(TAG, "  Stack 2 ID: %s", config_get_stack_2_id());
    ESP_LOGI(TAG, "  RS485 baudrate: %lu", config_get_rs485_baudrate());
    ESP_LOGI(TAG, "  Stack 1 JSON: %u bytes", stack1_json_len);
    ESP_LOGI(TAG, "  Stack 2 JSON: %u bytes", stack2_json_len);
  } else {
    ESP_LOGE(TAG, "Failed to send LAN config response");
  }
}

/**
 * @brief Dispatch downlink data to appropriate handler
 * 
 * For Module Base Setting trial (BLE-only), only RS485 and BLE handlers supported.
 */
static void dispatch_downlink_to_handler(handler_id_t target_id,
                                         const uint8_t *data, uint16_t length) {
  bool success = false;

  switch (target_id) {
  case HANDLER_RS485:
    success = rs485_handler_enqueue_downlink((uint8_t *)data, length);
    break;

    // case HANDLER_BLE:
    //   success = ble_handler_task_enqueue_downlink(data, length);
    //   break;

  default:
    ESP_LOGW(TAG, "Unsupported or unknown target handler: %d (CAN/LoRa/Zigbee not supported in trial)", target_id);
    return;
  }

  if (success) {
    ESP_LOGI(TAG, "Downlink dispatched to handler %s: %u bytes",
             handler_id_to_string(target_id), length);
  } else {
    ESP_LOGW(TAG, "Failed to dispatch downlink to handler %s",
             handler_id_to_string(target_id));
  }
}
// HELPER FUNCTIONS

static handler_id_t string_to_handler_id(const uint8_t *type_str) {
  if (memcmp(type_str, "RS4", 3) == 0)
    return HANDLER_RS485;
  // if (memcmp(type_str, "BLE", 3) == 0)
  //   return HANDLER_BLE;
  
  // CAN, LoRa, Zigbee not supported in Module Base Setting trial
  return HANDLER_UNKNOWN;
}
