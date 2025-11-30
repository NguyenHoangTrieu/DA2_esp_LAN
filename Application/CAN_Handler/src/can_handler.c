/**
 * @file can_handler.c
 * @brief CAN Message Handler Implementation
 */

#include "can_handler.h"
#include "can_driver.h"
#include "mcu_wan_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAN_HANDLER";

// ===== Task Configuration =====
#define CAN_HANDLER_TASK_STACK_SIZE 3072
#define CAN_HANDLER_TASK_PRIORITY 4
#define CAN_POLL_DELAY_MS 10
#define BUS_STATUS_CHECK_INTERVAL_MS 1000
#define STATS_LOG_INTERVAL_MS 30000

// ===== State =====
static TaskHandle_t g_can_handler_task = NULL;
static bool g_handler_running = false;
can_config_t g_can_config = {
    .baud_rate = 500000,            // Default 500 kbps
    .operating_mode = CAN_MODE_NORMAL // Default Normal mode
};

// ===== Statistics (internal) =====
typedef struct {
    uint32_t rx_count;          // Total received
    uint32_t filtered_count;    // Filtered out (done by driver)
    uint32_t forwarded_count;   // Successfully forwarded
    uint32_t wan_queue_full;    // WAN queue full errors
    uint32_t wan_no_mem;        // WAN memory errors
    uint32_t bus_errors;        // Bus error count
    uint32_t bus_recoveries;    // Bus recovery attempts
} can_handler_stats_t;

static can_handler_stats_t g_stats = {0};

// ===== Forward Declarations =====
static void can_handler_task(void *pvParameters);
static void check_and_handle_bus_errors(void);
static void log_statistics(void);

// ===== Public API Implementation =====

esp_err_t can_handler_start(void) {
    if (g_handler_running) {
        ESP_LOGW(TAG, "CAN handler already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting CAN Handler");

    // Phase 1: Initialize CAN driver
    can_status_t can_ret = can_driver_init();
    if (can_ret != CAN_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %d", can_ret);
        ESP_LOGE(TAG, "Check CAN configuration, GPIO pins, and hardware connection");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CAN driver initialized successfully");

    // Create handler task
    BaseType_t ret = xTaskCreate(
        can_handler_task,
        "can_handler",
        CAN_HANDLER_TASK_STACK_SIZE,
        NULL,
        CAN_HANDLER_TASK_PRIORITY,
        &g_can_handler_task
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN handler task");
        can_driver_deinit();
        return ESP_FAIL;
    }

    g_handler_running = true;
    ESP_LOGI(TAG, "CAN handler task created successfully");
    return ESP_OK;
}

esp_err_t can_handler_stop(void) {
    if (!g_handler_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping CAN handler");
    g_handler_running = false;

    // Wait briefly for task to exit gracefully
    vTaskDelay(pdMS_TO_TICKS(100));

    if (g_can_handler_task != NULL) {
        vTaskDelete(g_can_handler_task);
        g_can_handler_task = NULL;
    }

    can_driver_deinit();
    
    // Log final statistics
    ESP_LOGI(TAG, "Final Stats - RX: %lu, Forwarded: %lu, Errors: %lu",
             g_stats.rx_count, g_stats.forwarded_count,
             g_stats.wan_queue_full + g_stats.wan_no_mem);
    
    ESP_LOGI(TAG, "CAN handler stopped");
    return ESP_OK;
}

// ===== Internal Task Implementation =====

/**
 * @brief CAN Handler Main Task
 * 
 * Implements the flowchart logic with additional error handling:
 * 1. Receive CAN messages (non-blocking)
 * 2. Forward valid messages to WAN handler
 * 3. Monitor bus health periodically
 * 4. Log statistics periodically
 */
static void can_handler_task(void *pvParameters) {
    ESP_LOGI(TAG, "CAN handler task started");

    can_message_t rx_msg;
    TickType_t last_bus_check = xTaskGetTickCount();
    TickType_t last_stats_log = xTaskGetTickCount();

    // Main loop
    while (g_handler_running) {
        // Phase 2: Receive & Filter
        // Call can_receive (non-blocking, filtering done in driver)
        can_status_t status = can_receive(&rx_msg);

        if (status == CAN_OK) {
            // Message received and validated (frame type + whitelist already filtered)
            g_stats.rx_count++;

            ESP_LOGI(TAG, "CAN RX - ID: 0x%03X, DLC: %d", rx_msg.id, rx_msg.len);

            // Phase 3: Forward to WAN Handler
            esp_err_t wan_ret = mcu_wan_handler_queue_data(rx_msg.data, rx_msg.len);

            if (wan_ret == ESP_OK) {
                // Successfully queued to WAN handler
                g_stats.forwarded_count++;
                ESP_LOGI(TAG, "Forwarded to WAN handler");
            } else if (wan_ret == ESP_ERR_NO_MEM) {
                // Memory allocation failed
                g_stats.wan_no_mem++;
                ESP_LOGE(TAG, "WAN forward failed: Out of memory");
            } else if (wan_ret == ESP_FAIL) {
                // WAN queue full
                g_stats.wan_queue_full++;
                ESP_LOGW(TAG, "WAN forward failed: Queue full");
            } else {
                // Other errors
                ESP_LOGE(TAG, "WAN forward failed: %d", wan_ret);
            }
        } else if (status == CAN_ERR_RX_NO_DATA) {
            // No data in driver RX queue - yield CPU
            vTaskDelay(pdMS_TO_TICKS(CAN_POLL_DELAY_MS));
        } else {
            // Other errors (unlikely - driver should handle most issues)
            ESP_LOGE(TAG, "CAN receive error: %d", status);
            vTaskDelay(pdMS_TO_TICKS(CAN_POLL_DELAY_MS));
        }

        // Periodic bus health check
        TickType_t current_tick = xTaskGetTickCount();
        if ((current_tick - last_bus_check) >= pdMS_TO_TICKS(BUS_STATUS_CHECK_INTERVAL_MS)) {
            check_and_handle_bus_errors();
            last_bus_check = current_tick;
        }

        // Periodic statistics logging
        if ((current_tick - last_stats_log) >= pdMS_TO_TICKS(STATS_LOG_INTERVAL_MS)) {
            log_statistics();
            last_stats_log = current_tick;
        }
    }

    ESP_LOGI(TAG, "CAN handler task exiting");
    vTaskDelete(NULL);
}

/**
 * @brief Check CAN bus status and handle errors
 */
static void check_and_handle_bus_errors(void) {
    can_bus_state_t bus_state = can_check_bus_status();

    switch (bus_state) {
        case CAN_BUS_RUNNING:
            // Normal operation - no action needed
            break;

        case CAN_BUS_WARNING:
            ESP_LOGW(TAG, "CAN Bus Warning - Error counters elevated (TEC/REC >= 96)");
            g_stats.bus_errors++;
            break;

        case CAN_BUS_ERROR_PASSIVE:
            ESP_LOGW(TAG, "CAN Bus Error Passive - High error rate (TEC/REC >= 128)");
            g_stats.bus_errors++;
            break;

        case CAN_BUS_BUS_OFF:
            ESP_LOGE(TAG, "CAN Bus-Off detected! Initiating recovery...");
            g_stats.bus_errors++;
            g_stats.bus_recoveries++;
            
            can_status_t recovery_status = can_initiate_recovery();
            if (recovery_status == CAN_OK) {
                ESP_LOGI(TAG, "Bus-off recovery initiated successfully");
            } else {
                ESP_LOGE(TAG, "Bus-off recovery failed: %d", recovery_status);
            }
            break;

        case CAN_BUS_RECOVERING:
            ESP_LOGI(TAG, "CAN Bus recovering...");
            break;

        case CAN_BUS_UNKNOWN:
        default:
            ESP_LOGW(TAG, "CAN Bus status unknown");
            break;
    }
}

/**
 * @brief Log internal statistics
 */
static void log_statistics(void) {
    ESP_LOGI(TAG, "=== CAN Handler Statistics ===");
    ESP_LOGI(TAG, "Messages Received:    %lu", g_stats.rx_count);
    ESP_LOGI(TAG, "Messages Forwarded:   %lu", g_stats.forwarded_count);
    ESP_LOGI(TAG, "WAN Queue Full:       %lu", g_stats.wan_queue_full);
    ESP_LOGI(TAG, "WAN Out of Memory:    %lu", g_stats.wan_no_mem);
    ESP_LOGI(TAG, "Bus Errors Detected:  %lu", g_stats.bus_errors);
    ESP_LOGI(TAG, "Bus Recoveries:       %lu", g_stats.bus_recoveries);
    ESP_LOGI(TAG, "==============================");
}
