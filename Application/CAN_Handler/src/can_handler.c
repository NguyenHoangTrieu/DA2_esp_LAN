/**
 * @file can_handler.c
 * @brief CAN Handler for LAN MCU - Bidirectional Data Flow
 * 
 * Handles:
 * - CAN Bus RX -> Uplink to WAN MCU (via mcu_wan_enqueue_uplink)
 * - Downlink from WAN MCU -> CAN Bus TX
 */
#include "can_handler.h"
#include "can_driver.h"
#include "mcu_wan_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "CAN_HANDLER";

// ===== Configuration =====
#define CAN_HANDLER_TASK_STACK_SIZE     3072
#define CAN_HANDLER_TASK_PRIORITY       4
#define CAN_POLL_DELAY_MS               10
#define BUS_STATUS_CHECK_INTERVAL_MS    1000
#define STATS_LOG_INTERVAL_MS           30000
#define DOWNLINK_QUEUE_SIZE             20
#define MAX_CAN_PAYLOAD_SIZE            64   // CAN FD support

// ===== Downlink Packet =====
typedef struct {
    uint8_t *data_payload;
    uint16_t data_length;
} can_downlink_packet_t;

// ===== Statistics =====
typedef struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t forwarded_count;
    uint32_t tx_failed_count;
    uint32_t uplink_queue_full;
    uint32_t bus_errors;
    uint32_t bus_recoveries;
} can_handler_stats_t;

// ===== Global State =====
static TaskHandle_t g_can_handler_task = NULL;
static QueueHandle_t g_downlink_queue = NULL;
static bool g_handler_running = false;
static can_handler_stats_t g_stats = {0};

can_config_t g_can_config = {
    .baud_rate = 500000,
    .operating_mode = CAN_MODE_NORMAL
};

// ===== Forward Declarations =====
static void can_handler_task(void *pvParameters);
static void check_and_handle_bus_errors(void);
static void log_statistics(void);
static void process_can_rx(void);
static void process_downlink_tx(void);

// ===== Public API =====

esp_err_t can_handler_start(void) {
    if (g_handler_running) {
        ESP_LOGW(TAG, "CAN handler already running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting CAN Handler");
    
    // Initialize CAN driver
    can_status_t can_ret = can_driver_init();
    if (can_ret != CAN_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %d", can_ret);
        return ESP_FAIL;
    }
    
    // Create downlink queue
    g_downlink_queue = xQueueCreate(DOWNLINK_QUEUE_SIZE, sizeof(can_downlink_packet_t));
    if (g_downlink_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create downlink queue");
        can_driver_deinit();
        return ESP_FAIL;
    }
    
    // Create handler task
    BaseType_t ret = xTaskCreate(can_handler_task, "can_handler",
                                  CAN_HANDLER_TASK_STACK_SIZE, NULL,
                                  CAN_HANDLER_TASK_PRIORITY, &g_can_handler_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN handler task");
        vQueueDelete(g_downlink_queue);
        can_driver_deinit();
        return ESP_FAIL;
    }
    
    g_handler_running = true;
    memset(&g_stats, 0, sizeof(g_stats));
    
    ESP_LOGI(TAG, "CAN handler started successfully");
    return ESP_OK;
}

esp_err_t can_handler_stop(void) {
    if (!g_handler_running) return ESP_OK;
    
    ESP_LOGI(TAG, "Stopping CAN handler");
    g_handler_running = false;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clean up downlink queue
    if (g_downlink_queue != NULL) {
        can_downlink_packet_t pkt;
        while (xQueueReceive(g_downlink_queue, &pkt, 0) == pdTRUE) {
            if (pkt.data_payload) free(pkt.data_payload);
        }
        vQueueDelete(g_downlink_queue);
        g_downlink_queue = NULL;
    }
    
    can_driver_deinit();
    
    ESP_LOGI(TAG, "CAN handler stopped");
    return ESP_OK;
}

bool can_handler_enqueue_downlink(uint8_t *data, uint16_t len) {
    if (g_downlink_queue == NULL || data == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid downlink parameters");
        return false;
    }
    
    can_downlink_packet_t packet;
    packet.data_payload = (uint8_t *)malloc(len);
    if (packet.data_payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate downlink memory");
        return false;
    }
    
    memcpy(packet.data_payload, data, len);
    packet.data_length = len;
    
    if (xQueueSend(g_downlink_queue, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
        free(packet.data_payload);
        ESP_LOGW(TAG, "Downlink queue full");
        return false;
    }
    
    ESP_LOGD(TAG, "Downlink packet enqueued (%d bytes)", len);
    return true;
}

void can_handler_get_stats(uint32_t *rx_count, uint32_t *tx_count, uint32_t *errors) {
    if (rx_count) *rx_count = g_stats.rx_count;
    if (tx_count) *tx_count = g_stats.tx_count;
    if (errors) *errors = g_stats.bus_errors;
}

// ===== Main Task =====

static void can_handler_task(void *pvParameters) {
    ESP_LOGI(TAG, "CAN handler task started");
    
    TickType_t last_bus_check = xTaskGetTickCount();
    TickType_t last_stats_log = xTaskGetTickCount();
    
    while (g_handler_running) {
        TickType_t current_tick = xTaskGetTickCount();
        
        // ===== Process CAN RX -> Uplink =====
        process_can_rx();
        
        // ===== Process Downlink -> CAN TX =====
        process_downlink_tx();
        
        // ===== Periodic Bus Health Check =====
        if ((current_tick - last_bus_check) >= pdMS_TO_TICKS(BUS_STATUS_CHECK_INTERVAL_MS)) {
            check_and_handle_bus_errors();
            last_bus_check = current_tick;
        }
        
        // ===== Periodic Statistics Log =====
        if ((current_tick - last_stats_log) >= pdMS_TO_TICKS(STATS_LOG_INTERVAL_MS)) {
            log_statistics();
            last_stats_log = current_tick;
        }
        
        vTaskDelay(pdMS_TO_TICKS(CAN_POLL_DELAY_MS));
    }
    
    ESP_LOGI(TAG, "CAN handler task exiting");
    vTaskDelete(NULL);
}

// ===== CAN RX Processing =====
static void process_can_rx(void) {
    can_message_t rx_msg;
    can_status_t status = can_receive(&rx_msg);
    
    if (status == CAN_OK) {
        g_stats.rx_count++;
        
        ESP_LOGI(TAG, "CAN RX - ID: 0x%03lX, DLC: %d", rx_msg.id, rx_msg.len);
        
        // Build uplink packet: [CAN_ID(4)][DLC(1)][DATA(8)]
        uint8_t uplink_data[13];
        uplink_data[0] = (rx_msg.id >> 24) & 0xFF;
        uplink_data[1] = (rx_msg.id >> 16) & 0xFF;
        uplink_data[2] = (rx_msg.id >> 8) & 0xFF;
        uplink_data[3] = rx_msg.id & 0xFF;
        uplink_data[4] = rx_msg.len;
        memcpy(&uplink_data[5], rx_msg.data, rx_msg.len);
        
        // Enqueue to WAN uplink
        if (mcu_wan_enqueue_uplink(HANDLER_CAN, uplink_data, 5 + rx_msg.len)) {
            g_stats.forwarded_count++;
            ESP_LOGD(TAG, "Forwarded to WAN uplink");
        } else {
            g_stats.uplink_queue_full++;
            ESP_LOGW(TAG, "WAN uplink queue full");
        }
    } else if (status != CAN_ERR_RX_NO_DATA) {
        ESP_LOGE(TAG, "CAN receive error: %d", status);
    }
}

// ===== Downlink TX Processing =====
static void process_downlink_tx(void) {
    can_downlink_packet_t tx_packet;
    
    if (xQueueReceive(g_downlink_queue, &tx_packet, 0) != pdTRUE) {
        return;
    }
    
    ESP_LOGI(TAG, "Downlink packet received (%d bytes)", tx_packet.data_length);
    
    // Parse downlink: [CAN_ID(4)][DLC(1)][DATA(up to 8)]
    if (tx_packet.data_length >= 5) {
        uint32_t can_id = (tx_packet.data_payload[0] << 24) |
                          (tx_packet.data_payload[1] << 16) |
                          (tx_packet.data_payload[2] << 8) |
                          tx_packet.data_payload[3];
        uint8_t dlc = tx_packet.data_payload[4];
        
        if (dlc <= 8 && tx_packet.data_length >= (5 + dlc)) {
            can_status_t tx_status = can_transmit(can_id, &tx_packet.data_payload[5], dlc);
            
            if (tx_status == CAN_OK) {
                g_stats.tx_count++;
                ESP_LOGI(TAG, "CAN TX success - ID: 0x%03lX, DLC: %d", can_id, dlc);
            } else {
                g_stats.tx_failed_count++;
                ESP_LOGE(TAG, "CAN TX failed: %d", tx_status);
            }
        } else {
            ESP_LOGE(TAG, "Invalid CAN DLC: %d", dlc);
        }
    } else {
        ESP_LOGE(TAG, "Malformed downlink packet");
    }
    
    free(tx_packet.data_payload);
}

// ===== Bus Health Check =====
static void check_and_handle_bus_errors(void) {
    can_bus_state_t bus_state = can_check_bus_status();
    
    switch (bus_state) {
        case CAN_BUS_RUNNING:
            break;
        case CAN_BUS_WARNING:
            ESP_LOGW(TAG, "CAN Bus Warning");
            g_stats.bus_errors++;
            break;
        case CAN_BUS_ERROR_PASSIVE:
            ESP_LOGW(TAG, "CAN Bus Error Passive");
            g_stats.bus_errors++;
            break;
        case CAN_BUS_BUS_OFF:
            ESP_LOGE(TAG, "CAN Bus-Off! Initiating recovery...");
            g_stats.bus_errors++;
            g_stats.bus_recoveries++;
            can_initiate_recovery();
            break;
        case CAN_BUS_RECOVERING:
            ESP_LOGI(TAG, "CAN Bus recovering...");
            break;
        default:
            ESP_LOGW(TAG, "CAN Bus status unknown");
            break;
    }
}

// ===== Statistics Logging =====
static void log_statistics(void) {
    ESP_LOGI(TAG, "=== CAN Statistics ===");
    ESP_LOGI(TAG, "RX: %lu, Forwarded: %lu", g_stats.rx_count, g_stats.forwarded_count);
    ESP_LOGI(TAG, "TX: %lu, TX Failed: %lu", g_stats.tx_count, g_stats.tx_failed_count);
    ESP_LOGI(TAG, "Uplink Full: %lu, Bus Errors: %lu", g_stats.uplink_queue_full, g_stats.bus_errors);
    ESP_LOGI(TAG, "======================");
}
