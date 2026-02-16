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
#define BLE_COMMAND_QUEUE_SIZE      10
#define BLE_MAX_STACKS              2       // Stack 0 and Stack 1
#define BLE_UPLINK_BATCH_MAX        8
#define BLE_UPLINK_BATCH_FLUSH_MS   50

/* ===== Static Data ===== */

static struct {
    bool running[BLE_MAX_STACKS];                   // Running state per stack
    TaskHandle_t uplink_task_handle[BLE_MAX_STACKS];
    TaskHandle_t downlink_task_handle[BLE_MAX_STACKS];
    QueueHandle_t uplink_queue[BLE_MAX_STACKS];
    QueueHandle_t downlink_queue[BLE_MAX_STACKS];
    QueueHandle_t command_queue[BLE_MAX_STACKS];
} g_ble_task = {0};

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

/* ===== Task Implementations ===== */

/**
 * @brief Uplink task - collect data from BLE devices and forward to WAN MCU
 */
static void ble_uplink_task(void *pvParameters) {
    ble_task_context_t *ctx = (ble_task_context_t *)pvParameters;
    uint8_t stack_id = ctx->stack_id;
    
    ESP_LOGI(TAG, "[Stack %d] BLE uplink task started", stack_id);

    ble_uplink_packet_t batch[BLE_UPLINK_BATCH_MAX];
    uint8_t batch_count = 0;
    TickType_t last_flush = xTaskGetTickCount();

    while (g_ble_task.running[stack_id]) {
        ble_uplink_packet_t uplink;
        
        if (xQueueReceive(g_ble_task.uplink_queue[stack_id], &uplink,
                          pdMS_TO_TICKS(BLE_UPLINK_BATCH_FLUSH_MS)) == pdTRUE) {
            // Add to batch
            if (batch_count < BLE_UPLINK_BATCH_MAX) {
                batch[batch_count++] = uplink;
            }
        }

        TickType_t now = xTaskGetTickCount();
        
        // Flush batch if full or timeout reached
        if (batch_count > 0 &&
            ((now - last_flush) >= pdMS_TO_TICKS(BLE_UPLINK_BATCH_FLUSH_MS) ||
             batch_count >= BLE_UPLINK_BATCH_MAX)) {

            for (uint8_t i = 0; i < batch_count; i++) {
                // Format: [Stack ID (1B)][Payload (NB)]
                uint8_t packet[1 + 256];
                packet[0] = stack_id;  // Stack ID prefix
                memcpy(&packet[1], batch[i].payload, batch[i].payload_len);

                if (!mcu_wan_enqueue_uplink(HANDLER_BLE, packet, 1 + batch[i].payload_len)) {
                    ESP_LOGW(TAG, "[Stack %d] Uplink enqueue failed", stack_id);
                }
            }

            batch_count = 0;
            last_flush = now;
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
        // Process command queue first (higher priority)
        ble_command_request_t cmd_req;
        if (xQueueReceive(g_ble_task.command_queue[stack_id], &cmd_req, 0) == pdTRUE) {
            if (!g_ble_task.running[stack_id]) {
                break;
            }
            
            ESP_LOGI(TAG, "[Stack %d] Processing command: %s", stack_id, cmd_req.command);
            
            // Execute command with pre-matched function config (GPIO/delays from JSON)
            ble_exec_result_t result = {0};
            esp_err_t ret = ble_handler_execute_command_with_config(stack_id, 
                                                                    cmd_req.command, 
                                                                    &cmd_req.func_config, 
                                                                    &result);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "[Stack %d] Command executed successfully: %s", 
                        stack_id, result.response);
            } else {
                ESP_LOGE(TAG, "[Stack %d] Command execution failed: %s (status=%s)", 
                        stack_id, result.response, esp_err_to_name(ret));
            }
            
            continue;
        }
        
        // Process downlink data packets
        ble_downlink_packet_t downlink;
        
        if (xQueueReceive(g_ble_task.downlink_queue[stack_id], &downlink, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }

        if (!g_ble_task.running[stack_id]) {
            break;
        }

        // Send data to device
        esp_err_t ret = ble_handler_send_binary_command(stack_id, 
                                                         downlink.payload, 
                                                         downlink.payload_len,
                                                         NULL, 0, 0);
        
        if (ret != ESP_OK) {
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

    // Create command queue
    if (!g_ble_task.command_queue[stack_id]) {
        g_ble_task.command_queue[stack_id] = xQueueCreate(BLE_COMMAND_QUEUE_SIZE,
                                                          sizeof(ble_command_request_t));
        if (!g_ble_task.command_queue[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to create command queue", stack_id);
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

    if (g_ble_task.command_queue[stack_id]) {
        vQueueDelete(g_ble_task.command_queue[stack_id]);
        g_ble_task.command_queue[stack_id] = NULL;
    }

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
    
    return ESP_OK;
}

bool ble_handler_task_enqueue_uplink(uint8_t stack_id,
                                      const uint8_t *data,
                                      uint16_t len) {
    if (!ble_is_valid_stack(stack_id) || 
        !g_ble_task.running[stack_id] || 
        !g_ble_task.uplink_queue[stack_id] || 
        !data || 
        len == 0) {
        return false;
    }

    ble_uplink_packet_t packet = {
        .stack_id = stack_id,
        .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS,
        .payload_len = (len > sizeof(packet.payload)) ? sizeof(packet.payload) : len
    };
    
    memcpy(packet.payload, data, packet.payload_len);

    return (xQueueSend(g_ble_task.uplink_queue[stack_id], &packet, pdMS_TO_TICKS(100)) == pdTRUE);
}

bool ble_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len) {
    // Format: [Stack ID (1B)][Data (NB)]
    if (!data || len < 1) {
        ESP_LOGE(TAG, "Invalid downlink data (need >= 1 byte)");
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
        .payload_len = (len - 1 > sizeof(packet.payload)) ? sizeof(packet.payload) : (len - 1)
    };
    
    memcpy(packet.payload, &data[1], packet.payload_len);

    return (xQueueSend(g_ble_task.downlink_queue[stack_id], &packet, pdMS_TO_TICKS(100)) == pdTRUE);
}

esp_err_t ble_handler_task_execute_command(const ble_command_request_t *request) {
    if (!request || request->stack_id >= BLE_MAX_STACKS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t stack_id = request->stack_id;
    
    if (!g_ble_task.running[stack_id] || !g_ble_task.command_queue[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Task not running or command queue not initialized", stack_id);
        return ESP_ERR_INVALID_STATE;
    }

    // Enqueue command for processing
    if (xQueueSend(g_ble_task.command_queue[stack_id], request, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "[Stack %d] Command queue full, dropping command", stack_id);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "[Stack %d] Command enqueued: %s", stack_id, request->command);
    return ESP_OK;
}
