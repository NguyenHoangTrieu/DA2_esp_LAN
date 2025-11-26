/**
 * @file mcu_wan_handler.c
 * @brief MCU WAN Communication Handler (Master)
 * 
 * MCU LAN: Handles local area network (LoRa, Thread, Zigbee)
 * Has SD card for data buffering
 * Acts as SPI Master to communicate with MCU WAN (Slave)
 */

#include "mcu_wan_handler.h"
#include "wan_comm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <time.h>

static const char *TAG = "MCU_WAN_HANDLER";

// ===== Configuration =====
#define MCU_WAN_TASK_STACK_SIZE 4096
#define MCU_WAN_TASK_PRIORITY 5
#define MCU_WAN_QUEUE_SIZE 50  // Large queue for WAN data
#define HANDSHAKE_INTERVAL_MS 1000
#define ACK_TIMEOUT_MS 500
#define MAX_RETRY_COUNT 3
#define SD_CARD_BUFFER_SIZE 2048

// ===== Protocol Commands =====
#define CMD_HANDSHAKE_ACK 0x01
#define CMD_DATA_WITH_RTC 0x02

// ===== State Machine =====
typedef enum {
    STATE_INIT,
    STATE_HANDSHAKE,
    STATE_DATA_MODE,
    STATE_ERROR
} wan_handler_state_t;

// ===== Data Structures =====
typedef struct {
    uint8_t *data;
    uint16_t length;
    uint64_t timestamp_ms;
} wan_data_packet_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_time_t;

// ===== Global Variables =====
static wan_comm_handle_t g_wan_handle = NULL;  // SPI Master handle
static TaskHandle_t g_task_handle = NULL;
static QueueHandle_t g_wan_data_queue = NULL;  // Queue from LoRa/Thread/Zigbee handlers
static wan_handler_state_t g_state = STATE_INIT;
static bool g_handler_running = false;
static rtc_time_t g_rtc_from_wan = {0};  // RTC received from WAN MCU
volatile bool data_send_active = false;

// ===== Forward Declarations =====
static void mcu_wan_handler_task(void *pvParameters);
static esp_err_t perform_handshake_master(void);
static esp_err_t send_data_to_wan_with_retry(const uint8_t *data, uint16_t length, const rtc_time_t *rtc);
static esp_err_t save_data_to_sd_card(const uint8_t *data, uint16_t length);
static esp_err_t read_oldest_data_from_sd_card(uint8_t *buffer, uint16_t *length);
static bool sd_card_has_data(void);
static void use_rtc_time(rtc_time_t *rtc);

// ===== Initialization =====
esp_err_t mcu_wan_handler_start(void) {
    if (g_handler_running) {
        ESP_LOGW(TAG, "Handler already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting MCU WAN Handler (Master with SD card)");

    // Initialize WAN communication library (Master mode)
    wan_comm_config_t wan_config = {
        .gpio_sck = 12,
        .gpio_cs = 10,
        .gpio_io0 = 11,  // MOSI
        .gpio_io1 = 13,  // MISO
        .gpio_io2 = -1,
        .gpio_io3 = -1,
        .clock_speed_hz = 10000000,  // 10 MHz
        .mode = 0,
        .host_id = SPI2_HOST,
        .dma_channel = SPI_DMA_CH_AUTO,
        .queue_size = 7,
        .enable_quad_mode = false
    };

    wan_comm_status_t status = wan_comm_init(&wan_config, &g_wan_handle);
    if (status != WAN_COMM_OK) {
        ESP_LOGE(TAG, "Failed to initialize WAN comm: %d", status);
        return ESP_FAIL;
    }

    // Create queue for data from WAN handlers (LoRa, Thread, Zigbee)
    g_wan_data_queue = xQueueCreate(MCU_WAN_QUEUE_SIZE, sizeof(wan_data_packet_t));
    if (g_wan_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create data queue");
        wan_comm_deinit(g_wan_handle);
        return ESP_FAIL;
    }

    // TODO: Initialize SD card
    ESP_LOGI(TAG, "TODO: Initialize SD card for data buffering");

    // Create handler task
    BaseType_t ret = xTaskCreate(
        mcu_wan_handler_task,
        "mcu_wan_task",
        MCU_WAN_TASK_STACK_SIZE,
        NULL,
        MCU_WAN_TASK_PRIORITY,
        &g_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        vQueueDelete(g_wan_data_queue);
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

    if (g_wan_data_queue != NULL) {
        vQueueDelete(g_wan_data_queue);
        g_wan_data_queue = NULL;
    }

    if (g_wan_handle != NULL) {
        wan_comm_deinit(g_wan_handle);
        g_wan_handle = NULL;
    }

    // TODO: Deinitialize SD card

    ESP_LOGI(TAG, "MCU WAN Handler stopped");
    return ESP_OK;
}

// ===== Main Task =====
static void mcu_wan_handler_task(void *pvParameters) {
    ESP_LOGI(TAG, "MCU WAN Handler task started");

    // Phase 1: Handshake
    g_state = STATE_HANDSHAKE;
    while (g_handler_running && g_state == STATE_HANDSHAKE) {
        esp_err_t ret = perform_handshake_master();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Handshake successful, entering DATA_MODE");
            g_state = STATE_DATA_MODE;
            break;
        }
        ESP_LOGW(TAG, "Handshake failed, retrying in 1s...");
        vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_INTERVAL_MS));
    }

    // Phase 2: Main Data Loop
    wan_data_packet_t data_packet;
    
    while (g_handler_running && g_state == STATE_DATA_MODE) {
        // Priority 1: Check if there's real-time data from WAN handlers (LoRa/Thread/Zigbee)
        if (xQueueReceive(g_wan_data_queue, &data_packet, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Received data from WAN handler (LoRa/Thread/Zigbee): %d bytes", 
                     data_packet.length);
            
            // Package data with current RTC (from WAN MCU)
            rtc_time_t current_rtc;
            use_rtc_time(&current_rtc);
            
            // Send data to WAN MCU with retry
            esp_err_t ret = send_data_to_wan_with_retry(
                data_packet.data, 
                data_packet.length,
                &current_rtc
            );
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send after retries, saving to SD card");
                save_data_to_sd_card(data_packet.data, data_packet.length);
            }
            
            // Free data buffer
            free(data_packet.data);
            continue;
        }

        // Priority 2: Check SD Card Recovery (if no real-time data)
        if (sd_card_has_data()) {
            uint8_t sd_buffer[SD_CARD_BUFFER_SIZE];
            uint16_t sd_length = 0;
            
            if (read_oldest_data_from_sd_card(sd_buffer, &sd_length) == ESP_OK) {
                ESP_LOGI(TAG, "Sending SD card data (%d bytes)", sd_length);
                
                rtc_time_t current_rtc;
                use_rtc_time(&current_rtc);
                
                esp_err_t ret = send_data_to_wan_with_retry(sd_buffer, sd_length, &current_rtc);
                
                if (ret == ESP_OK) {
                    // TODO: Delete from SD card after successful send
                    ESP_LOGI(TAG, "SD data sent successfully");
                } else {
                    ESP_LOGW(TAG, "Failed to send SD data, will retry later");
                    break;  // Don't read more until this is sent
                }
            }
        }

        // Small delay
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "MCU WAN Handler task exiting");
    vTaskDelete(NULL);
}

// ===== Handshake Implementation (Master) =====
static esp_err_t perform_handshake_master(void) {
    ESP_LOGI(TAG, "Sending handshake ACK to WAN MCU (Slave)...");
    
    uint8_t handshake_cmd = CMD_HANDSHAKE_ACK;
    
    // Send ACK to WAN MCU
    wan_comm_status_t status = wan_comm_send_command(
        g_wan_handle,
        &handshake_cmd,
        1
    );

    if (status != WAN_COMM_OK) {
        ESP_LOGE(TAG, "Failed to send handshake command");
        return ESP_FAIL;
    }

    // Wait for ACK response from WAN MCU
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

// ===== Send Data with Retry =====
static esp_err_t send_data_to_wan_with_retry(const uint8_t *data, uint16_t length, const rtc_time_t *rtc) {
    if (data == NULL || length == 0 || rtc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Build packet: [RTC timestamp][data]
    // Use 32 bytes for RTC buffer to satisfy compiler (actual: 19 chars + null = 20)
    uint16_t total_length = 19 + length;  // RTC (19 bytes, no null) + data
    uint8_t *tx_buffer = (uint8_t *)malloc(total_length);
    if (tx_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Temporary buffer for snprintf with extra space
    char rtc_str[32];  // Larger buffer to avoid truncation warning
    snprintf(rtc_str, sizeof(rtc_str), "%02d/%02d/%04d-%02d:%02d:%02d",
             (int)(rtc->day % 100),      // Limit to 0-99
             (int)(rtc->month % 100),    // Limit to 0-99
             (int)rtc->year,
             (int)(rtc->hour % 100),     // Limit to 0-99
             (int)(rtc->minute % 100),   // Limit to 0-99
             (int)(rtc->second % 100));  // Limit to 0-99
    
    // Copy only first 19 characters (no null terminator)
    memcpy(tx_buffer, rtc_str, 19);
    
    // Append data
    memcpy(&tx_buffer[19], data, length);

    esp_err_t result = ESP_FAIL;
    
    for (int retry = 0; retry < MAX_RETRY_COUNT; retry++) {
        ESP_LOGI(TAG, "Sending data to WAN MCU (attempt %d/%d)", 
                 retry + 1, MAX_RETRY_COUNT);
        
        // Send data packet
        wan_comm_status_t status = wan_comm_send_data(
            g_wan_handle,
            tx_buffer,
            total_length
        );

        if (status != WAN_COMM_OK) {
            ESP_LOGE(TAG, "Failed to send data");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Wait for ACK
        uint8_t ack_buffer[16] = {0};
        status = wan_comm_request_data(g_wan_handle, ack_buffer, sizeof(ack_buffer));

        if (status == WAN_COMM_OK) {
            uint16_t header = (ack_buffer[0] << 8) | ack_buffer[1];
            
            if (header == WAN_COMM_HEADER_CF) {
                uint8_t ack_type = ack_buffer[2];
                
                if (ack_type == CMD_HANDSHAKE_ACK) {  // Use as generic ACK
                    ESP_LOGI(TAG, "Received ACK from WAN MCU");
                    result = ESP_OK;
                    break;
                }
            }
        }
        
        ESP_LOGW(TAG, "No ACK received, retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    free(tx_buffer);
    return result;
}
// ===== SD Card Operations =====
static esp_err_t save_data_to_sd_card(const uint8_t *data, uint16_t length) {
    // TODO: Implement SD card write
    ESP_LOGI(TAG, "TODO: Saving %d bytes to SD card", length);
    
    /*
     * Pseudo implementation:
     * 1. Open file in append mode
     * 2. Write timestamp + length + data
     * 3. Close file
     * 4. Update index/metadata
     */
    
    return ESP_OK;
}

static esp_err_t read_oldest_data_from_sd_card(uint8_t *buffer, uint16_t *length) {
    // TODO: Implement SD card read
    ESP_LOGD(TAG, "TODO: Reading oldest data from SD card");
    
    /*
     * Pseudo implementation:
     * 1. Check index/metadata for oldest entry
     * 2. Open file and seek to position
     * 3. Read length + data
     * 4. Return data
     */
    
    *length = 0;
    return ESP_ERR_NOT_FOUND;
}

static bool sd_card_has_data(void) {
    // TODO: Implement SD card check
    /*
     * Pseudo implementation:
     * 1. Check index/metadata file
     * 2. Return true if any unprocessed entries exist
     */
    
    return false;
}

// ===== RTC Helper =====
static void use_rtc_time(rtc_time_t *rtc) {
    // Use cached RTC from WAN MCU
    // If RTC hasn't been received yet, use default/system time
    if (g_rtc_from_wan.year == 0) {
        // Default time (system time or fallback)
        time_t now = time(NULL);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        
        rtc->day = timeinfo.tm_mday;
        rtc->month = timeinfo.tm_mon + 1;
        rtc->year = timeinfo.tm_year + 1900;
        rtc->hour = timeinfo.tm_hour;
        rtc->minute = timeinfo.tm_min;
        rtc->second = timeinfo.tm_sec;
    } else {
        // Use RTC from WAN MCU
        memcpy(rtc, &g_rtc_from_wan, sizeof(rtc_time_t));
    }
}

// ===== Queue Data from WAN Handlers (LoRa/Thread/Zigbee) =====
esp_err_t mcu_wan_handler_queue_data(const uint8_t *data, uint16_t length) {
    if (g_wan_data_queue == NULL || data == NULL || length == 0) {
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

    if (xQueueSend(g_wan_data_queue, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
        free(packet.data);
        ESP_LOGE(TAG, "Failed to queue data (queue full)");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Data queued from WAN handler (%d bytes)", length);
    return ESP_OK;
}

// ===== Update RTC from WAN MCU =====
void mcu_wan_handler_update_rtc(const rtc_time_t *rtc) {
    if (rtc != NULL) {
        memcpy(&g_rtc_from_wan, rtc, sizeof(rtc_time_t));
        ESP_LOGI(TAG, "RTC updated: %02d/%02d/%04d %02d:%02d:%02d",
                 rtc->day, rtc->month, rtc->year,
                 rtc->hour, rtc->minute, rtc->second);
    }
}

void mcu_wan_start_timer(void) {
    // Placeholder for timer initialization
    ESP_LOGI(TAG, "Timer started (placeholder)");
}
