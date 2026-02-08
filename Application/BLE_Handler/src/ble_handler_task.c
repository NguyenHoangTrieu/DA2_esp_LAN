/**
 * @file ble_handler_task.c
 * @brief BLE Handler Task Implementation with Multi-Stack Support
 */

#include "ble_handler_task.h"
#include "ble_handler.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BLE_TASK";

/* ===== Configuration ===== */

#define BLE_UPLINK_TASK_STACK_SIZE  4096
#define BLE_DOWNLINK_TASK_STACK_SIZE 4096
#define BLE_UPLINK_TASK_PRIORITY    5
#define BLE_DOWNLINK_TASK_PRIORITY  6
#define BLE_UPLINK_QUEUE_SIZE       20
#define BLE_DOWNLINK_QUEUE_SIZE     20
#define BLE_MAX_STACKS              2       // Stack 0 and Stack 1
#define MAX_CONNECTED_DEVICES       5
#define MAX_DISCOVERED_DEVICES      20
#define BLE_UPLINK_BATCH_MAX        8
#define BLE_UPLINK_BATCH_FLUSH_MS   50
#define BLE_DEVICE_IDLE_TIMEOUT_MS  60000
#define BLE_UPLINK_RETRY_COUNT      3       // Retry count for enqueue failures

/* ===== Static Data ===== */

static struct {
    bool running[BLE_MAX_STACKS];                   // Running state per stack
    TaskHandle_t uplink_task_handle[BLE_MAX_STACKS];
    TaskHandle_t downlink_task_handle[BLE_MAX_STACKS];
    QueueHandle_t uplink_queue[BLE_MAX_STACKS];
    QueueHandle_t downlink_queue[BLE_MAX_STACKS];
    uint32_t uplink_lost_count[BLE_MAX_STACKS];     // Track lost packets for monitoring
} g_ble_task = {0};

/**
 * @brief Connected BLE devices tracking (per stack)
 */
static struct {
    uint8_t mac_address[6];
    uint32_t connected_time_ms;
    uint32_t last_activity_ms;
    int8_t rssi;
} g_connected_devices[BLE_MAX_STACKS][MAX_CONNECTED_DEVICES] = {0};
static uint8_t g_connected_count[BLE_MAX_STACKS] = {0};
static SemaphoreHandle_t g_devices_mutex[BLE_MAX_STACKS] = {NULL};

/**
 * @brief Discovered devices from last scan (per stack)
 */
static struct {
    uint8_t mac_address[6];
    int8_t rssi;
} g_discovered_devices[BLE_MAX_STACKS][MAX_DISCOVERED_DEVICES] = {0};
static uint8_t g_discovered_count[BLE_MAX_STACKS] = {0};

/**
 * @brief Task context to pass stack_id to task functions
 */
typedef struct {
    uint8_t stack_id;
} ble_task_context_t;

/* ===== Helper Functions ===== */

/**
 * @brief Validate stack ID
 */
static inline bool ble_is_valid_stack(uint8_t stack_id) {
    return (stack_id < BLE_MAX_STACKS);
}

/**
 * @brief Find connected device by MAC address
 */
static int ble_find_connected_device(uint8_t stack_id, const uint8_t *mac_address) {
    if (!ble_is_valid_stack(stack_id) || !mac_address) {
        return -1;
    }
    
    for (int i = 0; i < g_connected_count[stack_id]; i++) {
        if (memcmp(g_connected_devices[stack_id][i].mac_address, mac_address, 6) == 0) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Add device to connected list
 */
static bool ble_add_connected_device(uint8_t stack_id, const uint8_t *mac_address) {
    if (!ble_is_valid_stack(stack_id) || !mac_address) {
        return false;
    }
    
    if (g_connected_count[stack_id] >= MAX_CONNECTED_DEVICES) {
        ESP_LOGW(TAG, "[Stack %d] Connected devices list full", stack_id);
        return false;
    }

    if (ble_find_connected_device(stack_id, mac_address) >= 0) {
        ESP_LOGD(TAG, "[Stack %d] Device already in connected list", stack_id);
        return false;
    }

    uint8_t idx = g_connected_count[stack_id];
    memcpy(g_connected_devices[stack_id][idx].mac_address, mac_address, 6);
    g_connected_devices[stack_id][idx].connected_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_connected_devices[stack_id][idx].last_activity_ms = g_connected_devices[stack_id][idx].connected_time_ms;
    g_connected_devices[stack_id][idx].rssi = 0;
    g_connected_count[stack_id]++;

    ESP_LOGI(TAG, "[Stack %d] Device added (total: %d)", stack_id, g_connected_count[stack_id]);
    return true;
}

/**
 * @brief Remove device from connected list
 */
static bool ble_remove_connected_device(uint8_t stack_id, const uint8_t *mac_address) {
    if (!ble_is_valid_stack(stack_id) || !mac_address) {
        return false;
    }
    
    int idx = ble_find_connected_device(stack_id, mac_address);
    if (idx < 0) {
        return false;
    }

    // Shift remaining devices
    for (int i = idx; i < g_connected_count[stack_id] - 1; i++) {
        memcpy(&g_connected_devices[stack_id][i],
               &g_connected_devices[stack_id][i + 1],
               sizeof(g_connected_devices[stack_id][0]));
    }
    g_connected_count[stack_id]--;

    ESP_LOGI(TAG, "[Stack %d] Device removed (remaining: %d)", stack_id, g_connected_count[stack_id]);
    return true;
}

/**
 * @brief Update device activity timestamp
 */
static void ble_update_activity(uint8_t stack_id, const uint8_t *mac_address) {
    if (!ble_is_valid_stack(stack_id) || !mac_address || !g_devices_mutex[stack_id]) {
        return;
    }

    if (xSemaphoreTake(g_devices_mutex[stack_id], pdMS_TO_TICKS(50)) == pdTRUE) {
        int idx = ble_find_connected_device(stack_id, mac_address);
        if (idx >= 0) {
            g_connected_devices[stack_id][idx].last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        xSemaphoreGive(g_devices_mutex[stack_id]);
    }
}

/**
 * @brief Cleanup idle devices (APPROVED: Fix timeout calculation)
 */
static void ble_cleanup_idle_devices(uint8_t stack_id) {
    if (!ble_is_valid_stack(stack_id) || !g_devices_mutex[stack_id]) {
        return;
    }

    uint8_t idle_macs[MAX_CONNECTED_DEVICES][6];
    uint8_t idle_count = 0;

    if (xSemaphoreTake(g_devices_mutex[stack_id], pdMS_TO_TICKS(50)) == pdTRUE) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        for (int i = g_connected_count[stack_id] - 1; i >= 0; i--) {
            // FIXED: Correct timestamp calculation (both in ms now)
            uint32_t idle_ms = now_ms - g_connected_devices[stack_id][i].last_activity_ms;
            
            if (idle_ms > BLE_DEVICE_IDLE_TIMEOUT_MS && idle_count < MAX_CONNECTED_DEVICES) {
                memcpy(idle_macs[idle_count], g_connected_devices[stack_id][i].mac_address, 6);
                idle_count++;
                ESP_LOGW(TAG, "[Stack %d] Device idle for %lu ms, disconnecting", stack_id, idle_ms);
            }
        }
        xSemaphoreGive(g_devices_mutex[stack_id]);
    }

    // Disconnect and remove idle devices
    for (uint8_t i = 0; i < idle_count; i++) {
        ble_handler_disconnect(stack_id);
        if (xSemaphoreTake(g_devices_mutex[stack_id], pdMS_TO_TICKS(50)) == pdTRUE) {
            ble_remove_connected_device(stack_id, idle_macs[i]);
            xSemaphoreGive(g_devices_mutex[stack_id]);
        }
    }
}

/* ===== Task Implementations ===== */

/**
 * @brief Uplink task - collect data from BLE devices and forward to WAN MCU
 * 
 * APPROVED Enhancements:
 * - Retry logic when enqueue fails (max 3 attempts)
 * - Lost packet tracking and logging
 */
static void ble_uplink_task(void *pvParameters) {
    ble_task_context_t *ctx = (ble_task_context_t *)pvParameters;
    uint8_t stack_id = ctx->stack_id;
    
    ESP_LOGI(TAG, "[Stack %d] BLE uplink task started", stack_id);

    ble_uplink_packet_t batch[BLE_UPLINK_BATCH_MAX];
    uint8_t batch_count = 0;
    TickType_t last_flush = xTaskGetTickCount();
    TickType_t last_health = xTaskGetTickCount();

    while (g_ble_task.running[stack_id]) {
        ble_uplink_packet_t uplink;
        
        if (xQueueReceive(g_ble_task.uplink_queue[stack_id], &uplink,
                          pdMS_TO_TICKS(BLE_UPLINK_BATCH_FLUSH_MS)) == pdTRUE) {
            
            // Add to batch
            if (batch_count < BLE_UPLINK_BATCH_MAX) {
                batch[batch_count++] = uplink;
                ble_update_activity(stack_id, uplink.device_address);
            }
        }

        TickType_t now = xTaskGetTickCount();
        
        // Flush batch if full or timeout reached
        if (batch_count > 0 &&
            ((now - last_flush) >= pdMS_TO_TICKS(BLE_UPLINK_BATCH_FLUSH_MS) ||
             batch_count >= BLE_UPLINK_BATCH_MAX)) {

            for (uint8_t i = 0; i < batch_count; i++) {
                uint8_t packet[1 + 6 + 256];
                packet[0] = stack_id;  // Stack ID prefix
                memcpy(&packet[1], batch[i].device_address, 6);
                memcpy(&packet[7], batch[i].payload, batch[i].payload_len);

                // APPROVED: Retry logic for enqueue failures
                bool enqueued = false;
                for (int retry = 0; retry < BLE_UPLINK_RETRY_COUNT; retry++) {
                    enqueued = mcu_wan_enqueue_uplink(HANDLER_BLE,
                                                     packet,
                                                     1 + 6 + batch[i].payload_len);
                    if (enqueued) {
                        break;
                    }
                    
                    ESP_LOGW(TAG, "[Stack %d] Uplink enqueue failed (attempt %d/%d)", 
                            stack_id, retry + 1, BLE_UPLINK_RETRY_COUNT);
                    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between retries
                }
                
                if (!enqueued) {
                    g_ble_task.uplink_lost_count[stack_id]++;
                    ESP_LOGE(TAG, "[Stack %d] Uplink packet LOST (total lost: %lu)", 
                            stack_id, g_ble_task.uplink_lost_count[stack_id]);
                }
            }

            batch_count = 0;
            last_flush = now;
        }

        // Periodic health check (1 second)
        if ((now - last_health) >= pdMS_TO_TICKS(1000)) {
            last_health = now;
            ble_cleanup_idle_devices(stack_id);
            
            // Log stats if any packets were lost
            if (g_ble_task.uplink_lost_count[stack_id] > 0) {
                ESP_LOGW(TAG, "[Stack %d] Uplink stats: %lu packets lost", 
                        stack_id, g_ble_task.uplink_lost_count[stack_id]);
            }
        }
    }

    ESP_LOGI(TAG, "[Stack %d] BLE uplink task exiting", stack_id);
    free(ctx);
    vTaskDelete(NULL);
}

/**
 * @brief Downlink task - send commands/data to BLE devices
 */
static void ble_downlink_task(void *pvParameters) {
    ble_task_context_t *ctx = (ble_task_context_t *)pvParameters;
    uint8_t stack_id = ctx->stack_id;
    
    ESP_LOGI(TAG, "[Stack %d] BLE downlink task started", stack_id);

    while (g_ble_task.running[stack_id]) {
        ble_downlink_packet_t downlink;
        
        if (xQueueReceive(g_ble_task.downlink_queue[stack_id], &downlink, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!g_ble_task.running[stack_id]) {
            break;
        }

        // Verify device is connected
        bool connected = false;
        if (xSemaphoreTake(g_devices_mutex[stack_id], pdMS_TO_TICKS(50)) == pdTRUE) {
            connected = (ble_find_connected_device(stack_id, downlink.device_address) >= 0);
            xSemaphoreGive(g_devices_mutex[stack_id]);
        }

        if (!connected) {
            ESP_LOGW(TAG, "[Stack %d] Target device not connected, dropping downlink", stack_id);
            continue;
        }

        // Send data to device
        esp_err_t ret = ble_handler_send_data(stack_id, downlink.payload, downlink.payload_len);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[Stack %d] Downlink sent successfully", stack_id);
            ble_update_activity(stack_id, downlink.device_address);
        } else {
            ESP_LOGE(TAG, "[Stack %d] Failed to send downlink: %s", stack_id, esp_err_to_name(ret));
        }
    }

    ESP_LOGI(TAG, "[Stack %d] BLE downlink task exiting", stack_id);
    free(ctx);
    vTaskDelete(NULL);
}

/* ===== Public API Implementation ===== */

esp_err_t ble_handler_task_start(uint8_t stack_id) {
    if (!ble_is_valid_stack(stack_id)) {
        ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_ble_task.running[stack_id]) {
        ESP_LOGW(TAG, "[Stack %d] BLE handler task already running", stack_id);
        return ESP_OK;
    }

    // Create queues
    if (!g_ble_task.uplink_queue[stack_id]) {
        g_ble_task.uplink_queue[stack_id] = xQueueCreate(BLE_UPLINK_QUEUE_SIZE, 
                                                         sizeof(ble_uplink_packet_t));
        if (!g_ble_task.uplink_queue[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to create uplink queue", stack_id);
            return ESP_ERR_NO_MEM;
        }
    }

    if (!g_ble_task.downlink_queue[stack_id]) {
        g_ble_task.downlink_queue[stack_id] = xQueueCreate(BLE_DOWNLINK_QUEUE_SIZE,
                                                           sizeof(ble_downlink_packet_t));
        if (!g_ble_task.downlink_queue[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to create downlink queue", stack_id);
            return ESP_ERR_NO_MEM;
        }
    }

    // Create mutex for device list
    if (!g_devices_mutex[stack_id]) {
        g_devices_mutex[stack_id] = xSemaphoreCreateMutex();
        if (!g_devices_mutex[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to create mutex", stack_id);
            return ESP_ERR_NO_MEM;
        }
    }

    // Initialize BLE handler middleware (once)
    static bool middleware_init = false;
    if (!middleware_init) {
        if (ble_handler_init() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize BLE middleware");
            return ESP_FAIL;
        }
        middleware_init = true;
    }

    // Create task context
    ble_task_context_t *uplink_ctx = (ble_task_context_t *)malloc(sizeof(ble_task_context_t));
    ble_task_context_t *downlink_ctx = (ble_task_context_t *)malloc(sizeof(ble_task_context_t));
    
    if (!uplink_ctx || !downlink_ctx) {
        ESP_LOGE(TAG, "[Stack %d] Failed to allocate task contexts", stack_id);
        free(uplink_ctx);
        free(downlink_ctx);
        return ESP_ERR_NO_MEM;
    }
    
    uplink_ctx->stack_id = stack_id;
    downlink_ctx->stack_id = stack_id;

    // Create tasks
    g_ble_task.running[stack_id] = true;
    g_ble_task.uplink_lost_count[stack_id] = 0;
    
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "ble_ul_s%d", stack_id);
    
    BaseType_t ret = xTaskCreate(ble_uplink_task,
                                 task_name,
                                 BLE_UPLINK_TASK_STACK_SIZE,
                                 uplink_ctx,
                                 BLE_UPLINK_TASK_PRIORITY,
                                 &g_ble_task.uplink_task_handle[stack_id]);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "[Stack %d] Failed to create BLE uplink task", stack_id);
        g_ble_task.running[stack_id] = false;
        free(uplink_ctx);
        free(downlink_ctx);
        return ESP_FAIL;
    }

    snprintf(task_name, sizeof(task_name), "ble_dl_s%d", stack_id);
    ret = xTaskCreate(ble_downlink_task,
                      task_name,
                      BLE_DOWNLINK_TASK_STACK_SIZE,
                      downlink_ctx,
                      BLE_DOWNLINK_TASK_PRIORITY,
                      &g_ble_task.downlink_task_handle[stack_id]);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "[Stack %d] Failed to create BLE downlink task", stack_id);
        vTaskDelete(g_ble_task.uplink_task_handle[stack_id]);
        g_ble_task.uplink_task_handle[stack_id] = NULL;
        g_ble_task.running[stack_id] = false;
        free(downlink_ctx);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "[Stack %d] BLE handler tasks started", stack_id);
    return ESP_OK;
}

esp_err_t ble_handler_task_stop(uint8_t stack_id) {
    if (!ble_is_valid_stack(stack_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_ble_task.running[stack_id]) {
        ESP_LOGW(TAG, "[Stack %d] BLE handler task not running", stack_id);
        return ESP_OK;
    }

    g_ble_task.running[stack_id] = false;

    // Delete tasks
    if (g_ble_task.uplink_task_handle[stack_id]) {
        vTaskDelete(g_ble_task.uplink_task_handle[stack_id]);
        g_ble_task.uplink_task_handle[stack_id] = NULL;
    }

    if (g_ble_task.downlink_task_handle[stack_id]) {
        vTaskDelete(g_ble_task.downlink_task_handle[stack_id]);
        g_ble_task.downlink_task_handle[stack_id] = NULL;
    }

    // Cleanup queues
    if (g_ble_task.uplink_queue[stack_id]) {
        vQueueDelete(g_ble_task.uplink_queue[stack_id]);
        g_ble_task.uplink_queue[stack_id] = NULL;
    }

    if (g_ble_task.downlink_queue[stack_id]) {
        vQueueDelete(g_ble_task.downlink_queue[stack_id]);
        g_ble_task.downlink_queue[stack_id] = NULL;
    }

    if (g_devices_mutex[stack_id]) {
        vSemaphoreDelete(g_devices_mutex[stack_id]);
        g_devices_mutex[stack_id] = NULL;
    }

    g_connected_count[stack_id] = 0;
    g_discovered_count[stack_id] = 0;

    ESP_LOGI(TAG, "[Stack %d] BLE handler tasks stopped", stack_id);
    return ESP_OK;
}

esp_err_t ble_handler_task_load_config(uint8_t stack_id, const char *json_config, uint16_t len) {
    if (!ble_is_valid_stack(stack_id) || !json_config || len == 0) {
        ESP_LOGE(TAG, "Invalid config parameters");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "[Stack %d] Loading JSON config (%d bytes)", stack_id, len);

    // Pass to middleware to parse and apply
    esp_err_t ret = ble_handler_load_config(stack_id, json_config, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Stack %d] Failed to load config: %s", stack_id, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "[Stack %d] Configuration loaded successfully", stack_id);

    // Execute core startup sequence after config is applied
    ble_handler_hw_reset(stack_id);
    vTaskDelay(pdMS_TO_TICKS(100));
    ble_handler_enter_cmd_mode(stack_id);
    ble_handler_enter_data_mode(stack_id);
    ble_handler_start_broadcast(stack_id);
    
    return ESP_OK;
}

bool ble_handler_task_enqueue_uplink(uint8_t stack_id,
                                      const uint8_t *device_address,
                                      const uint8_t *data,
                                      uint16_t len) {
    if (!ble_is_valid_stack(stack_id) || 
        !g_ble_task.running[stack_id] || 
        !g_ble_task.uplink_queue[stack_id] || 
        !device_address || 
        !data || 
        len == 0) {
        return false;
    }

    ble_uplink_packet_t packet = {
        .stack_id = stack_id,
        .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS,
        .payload_len = (len > sizeof(packet.payload)) ? sizeof(packet.payload) : len
    };
    
    memcpy(packet.device_address, device_address, 6);
    memcpy(packet.payload, data, packet.payload_len);

    return (xQueueSend(g_ble_task.uplink_queue[stack_id], &packet, pdMS_TO_TICKS(100)) == pdTRUE);
}

bool ble_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len) {
    // Format: [Stack ID (1B)][Target MAC (6B)][Data (NB)]
    if (!data || len < 7) {
        ESP_LOGE(TAG, "Invalid downlink data (need >= 7 bytes)");
        return false;
    }

    uint8_t stack_id = data[0];
    if (!ble_is_valid_stack(stack_id) || 
        !g_ble_task.running[stack_id] || 
        !g_ble_task.downlink_queue[stack_id]) {
        ESP_LOGE(TAG, "Stack %d not running or invalid", stack_id);
        return false;
    }

    ble_downlink_packet_t packet = {
        .stack_id = stack_id,
        .timeout_ms = 1000,
        .payload_len = (len - 7 > sizeof(packet.payload)) ? sizeof(packet.payload) : (len - 7)
    };
    
    memcpy(packet.device_address, &data[1], 6);
    memcpy(packet.payload, &data[7], packet.payload_len);

    return (xQueueSend(g_ble_task.downlink_queue[stack_id], &packet, pdMS_TO_TICKS(100)) == pdTRUE);
}

uint8_t ble_handler_task_get_connected_devices(uint8_t stack_id,
                                                uint8_t *device_count,
                                                uint8_t devices[][6]) {
    if (!ble_is_valid_stack(stack_id) || !device_count || !devices) {
        return 0;
    }

    uint8_t count = 0;

    if (xSemaphoreTake(g_devices_mutex[stack_id], pdMS_TO_TICKS(500)) == pdTRUE) {
        count = (g_connected_count[stack_id] < *device_count) ? g_connected_count[stack_id] : *device_count;
        for (int i = 0; i < count; i++) {
            memcpy(devices[i], g_connected_devices[stack_id][i].mac_address, 6);
        }
        *device_count = count;
        xSemaphoreGive(g_devices_mutex[stack_id]);
    }

    return count;
}

esp_err_t ble_handler_task_start_discovery(uint8_t stack_id, uint32_t scan_duration_ms) {
    if (!ble_is_valid_stack(stack_id) || !g_ble_task.running[stack_id]) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "[Stack %d] Starting device discovery for %lu ms", stack_id, scan_duration_ms);

    g_discovered_count[stack_id] = 0;
    return ble_handler_start_discovery(stack_id);
}

uint8_t ble_handler_task_get_discovered_devices(uint8_t stack_id,
                                                 uint8_t devices[][6],
                                                 uint8_t max_count) {
    if (!ble_is_valid_stack(stack_id) || !devices) {
        return 0;
    }

    uint8_t count = (g_discovered_count[stack_id] < max_count) ? g_discovered_count[stack_id] : max_count;
    for (int i = 0; i < count; i++) {
        memcpy(devices[i], g_discovered_devices[stack_id][i].mac_address, 6);
    }

    return count;
}
