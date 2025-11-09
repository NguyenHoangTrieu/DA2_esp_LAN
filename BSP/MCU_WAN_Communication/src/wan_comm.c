/**
 * @file wan_comm.c
 * @brief WAN MCU Communication Library Implementation
 */

#include "wan_comm.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_attr.h"

static const char* TAG = "WAN_COMM";

/**
 * @brief Transaction completion event
 */
typedef struct {
    size_t trans_len;           // Transaction length
    uint8_t* buffer;            // Buffer pointer
} wan_comm_trans_event_t;

/**
 * @brief Double buffer pool structure
 */
typedef struct {
    uint8_t* buffer_a;              // First buffer
    uint8_t* buffer_b;              // Second buffer
    uint16_t buffer_size;           // Size of each buffer
    uint8_t active_buffer;          // 0 for A, 1 for B
    SemaphoreHandle_t swap_mutex;   // Mutex for buffer swapping
} wan_comm_buffer_pool_t;

/**
 * @brief Internal handle structure
 */
struct wan_comm_handle_s {
    // Configuration
    wan_comm_config_t config;
    
    // SPI slave
    spi_slave_transaction_t spi_trans;
    
    // Buffers
    wan_comm_buffer_pool_t rx_buffer_pool;
    uint8_t* tx_buffer;
    size_t tx_buffer_len;
    SemaphoreHandle_t tx_buffer_mutex;
    
    // Processing
    TaskHandle_t processing_task_handle;
    
    // State
    bool is_initialized;
    bool is_running;
    bool is_tx_only;
    wan_comm_status_t last_error;
    
    // Statistics
    uint32_t commands_received;
    uint32_t data_packets_received;
    uint32_t error_count;
};

// Forward declarations
static void wan_comm_processing_task(void* arg);
static wan_comm_status_t wan_comm_parse_packet(uint8_t* buffer, size_t length, 
                                               uint16_t* header_type, 
                                               uint8_t** payload, 
                                               uint16_t* payload_length);
static wan_comm_status_t wan_comm_start_receive_transaction(wan_comm_handle_t handle);
static uint8_t* wan_comm_get_active_rx_buffer(wan_comm_handle_t handle);
static uint8_t* wan_comm_swap_rx_buffers(wan_comm_handle_t handle);
static void wan_comm_report_error(wan_comm_handle_t handle, wan_comm_status_t error, const char* context);

/**
 * @brief Initialize WAN communication library
 */
wan_comm_status_t wan_comm_init(const wan_comm_config_t* config, wan_comm_handle_t* handle) {
    if (config == NULL || handle == NULL) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    if (config->on_command_received == NULL || config->on_data_received == NULL) {
        ESP_LOGE(TAG, "Callbacks are mandatory");
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing WAN communication library (Slave mode)");
    
    // Allocate handle
    wan_comm_handle_t h = (wan_comm_handle_t)calloc(1, sizeof(struct wan_comm_handle_s));
    if (h == NULL) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return WAN_COMM_ERR_NO_MEM;
    }
    
    // Copy configuration
    memcpy(&h->config, config, sizeof(wan_comm_config_t));
    
    // Set defaults
    if (h->config.rx_buffer_size == 0) {
        h->config.rx_buffer_size = WAN_COMM_DEFAULT_RX_BUFFER_SIZE;
    }
    if (h->config.tx_buffer_size == 0) {
        h->config.tx_buffer_size = WAN_COMM_DEFAULT_TX_BUFFER_SIZE;
    }
    if (h->config.dma_channel == 0) {
        h->config.dma_channel = SPI_DMA_CH_AUTO;
    }
    
    // Allocate double RX buffers (DMA-capable memory)
    h->rx_buffer_pool.buffer_size = h->config.rx_buffer_size;
    h->rx_buffer_pool.buffer_a = (uint8_t*)heap_caps_malloc(h->config.rx_buffer_size, MALLOC_CAP_DMA);
    h->rx_buffer_pool.buffer_b = (uint8_t*)heap_caps_malloc(h->config.rx_buffer_size, MALLOC_CAP_DMA);
    h->rx_buffer_pool.active_buffer = 0;
    h->rx_buffer_pool.swap_mutex = xSemaphoreCreateMutex();
    
    // Allocate TX buffer (DMA-capable memory)
    h->tx_buffer = (uint8_t*)heap_caps_malloc(h->config.tx_buffer_size, MALLOC_CAP_DMA);
    h->tx_buffer_mutex = xSemaphoreCreateMutex();
    h->tx_buffer_len = 0;
    
    if (h->rx_buffer_pool.buffer_a == NULL || h->rx_buffer_pool.buffer_b == NULL || 
        h->tx_buffer == NULL || h->rx_buffer_pool.swap_mutex == NULL || 
        h->tx_buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers or synchronization primitives");
        free(h->rx_buffer_pool.buffer_a);
        free(h->rx_buffer_pool.buffer_b);
        free(h->tx_buffer);
        if (h->rx_buffer_pool.swap_mutex) vSemaphoreDelete(h->rx_buffer_pool.swap_mutex);
        if (h->tx_buffer_mutex) vSemaphoreDelete(h->tx_buffer_mutex);
        free(h);
        return WAN_COMM_ERR_NO_MEM;
    }
    
    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->gpio_io0,
        .miso_io_num = config->gpio_io1,
        .sclk_io_num = config->gpio_sck,
        .quadwp_io_num = config->enable_quad_mode ? config->gpio_io2 : -1,
        .quadhd_io_num = config->enable_quad_mode ? config->gpio_io3 : -1,
        .max_transfer_sz = h->config.rx_buffer_size,
        .flags = 0
    };
    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = config->gpio_cs,
        .flags = 0,
        .queue_size = WAN_COMM_TRANS_QUEUE_SIZE,  // From ESP-IDF example
        .mode = config->mode,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };
    
    esp_err_t ret = spi_slave_initialize(config->host_id, &bus_cfg, &slave_cfg, config->dma_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        free(h->rx_buffer_pool.buffer_a);
        free(h->rx_buffer_pool.buffer_b);
        free(h->tx_buffer);
        vSemaphoreDelete(h->rx_buffer_pool.swap_mutex);
        vSemaphoreDelete(h->tx_buffer_mutex);
        free(h);
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    // Configure GPIO for CS
    gpio_set_pull_mode(config->gpio_cs, GPIO_PULLUP_ONLY);
    
    // Initialize state
    h->is_initialized = true;
    h->is_running = true;
    h->is_tx_only = false;
    h->last_error = WAN_COMM_OK;
    h->commands_received = 0;
    h->data_packets_received = 0;
    h->error_count = 0;
    
    // Create processing task
    BaseType_t task_ret = xTaskCreate(
        wan_comm_processing_task,
        "wan_comm_proc",
        WAN_COMM_PROCESSING_TASK_STACK_SIZE,
        h,
        WAN_COMM_PROCESSING_TASK_PRIORITY,
        &h->processing_task_handle
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processing task");
        spi_slave_free(config->host_id);
        free(h->rx_buffer_pool.buffer_a);
        free(h->rx_buffer_pool.buffer_b);
        free(h->tx_buffer);
        vSemaphoreDelete(h->rx_buffer_pool.swap_mutex);
        vSemaphoreDelete(h->tx_buffer_mutex);
        free(h);
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    // Start first receive transaction
    wan_comm_start_receive_transaction(h);
    
    *handle = h;
    
    ESP_LOGI(TAG, "WAN communication initialized successfully");
    ESP_LOGI(TAG, "Mode: %d, RX Buffer: %d bytes (double), TX Buffer: %d bytes", 
             config->mode, h->config.rx_buffer_size, h->config.tx_buffer_size);
    
    return WAN_COMM_OK;
}

/**
 * @brief Deinitialize WAN communication library
 */
wan_comm_status_t wan_comm_deinit(wan_comm_handle_t handle) {
    if (handle == NULL || !handle->is_initialized) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Deinitializing WAN communication library");
    
    // Stop processing task
    handle->is_running = false;
    if (handle->processing_task_handle) {
        vTaskDelete(handle->processing_task_handle);
        handle->processing_task_handle = NULL;
    }
    
    // Free SPI slave
    spi_slave_free(handle->config.host_id);
    
    // Free resources
    free(handle->rx_buffer_pool.buffer_a);
    free(handle->rx_buffer_pool.buffer_b);
    free(handle->tx_buffer);
    vSemaphoreDelete(handle->rx_buffer_pool.swap_mutex);
    vSemaphoreDelete(handle->tx_buffer_mutex);
    
    handle->is_initialized = false;
    free(handle);
    
    ESP_LOGI(TAG, "WAN communication deinitialized");
    
    return WAN_COMM_OK;
}

/**
 * @brief Load TX data
 */
wan_comm_status_t wan_comm_load_tx_data(wan_comm_handle_t handle, 
                                        const uint8_t* data_to_send, 
                                        uint16_t length) {
    if (handle == NULL || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (data_to_send == NULL || length == 0) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    if (length > handle->config.tx_buffer_size) {
        ESP_LOGE(TAG, "TX data length %d exceeds buffer size %d", length, handle->config.tx_buffer_size);
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    // Lock TX buffer
    if (xSemaphoreTake(handle->tx_buffer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "load_tx_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    // Copy data to TX buffer
    memcpy(handle->tx_buffer, data_to_send, length);
    handle->tx_buffer_len = length;
    
    xSemaphoreGive(handle->tx_buffer_mutex);
    // Set TX only flag
    handle->is_tx_only = true;
    ESP_LOGI(TAG, "TX data loaded: %d bytes", length);
    
    return WAN_COMM_OK;
}

/**
 * @brief Get last error
 */
wan_comm_status_t wan_comm_get_last_error(wan_comm_handle_t handle) {
    if (handle == NULL) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    return handle->last_error;
}

/**
 * @brief Get statistics
 */
wan_comm_status_t wan_comm_get_statistics(wan_comm_handle_t handle,
                                          uint32_t* commands_received,
                                          uint32_t* data_packets_received,
                                          uint32_t* errors) {
    if (handle == NULL || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (commands_received) *commands_received = handle->commands_received;
    if (data_packets_received) *data_packets_received = handle->data_packets_received;
    if (errors) *errors = handle->error_count;
    
    return WAN_COMM_OK;
}

/**
 * @brief Clear statistics
 */
wan_comm_status_t wan_comm_clear_statistics(wan_comm_handle_t handle) {
    if (handle == NULL || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    handle->commands_received = 0;
    handle->data_packets_received = 0;
    handle->error_count = 0;
    
    return WAN_COMM_OK;
}

// ===== Internal Functions =====

/**
 * @brief Start receive transaction
 */
static wan_comm_status_t wan_comm_start_receive_transaction(wan_comm_handle_t handle) {
    if (!handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    // Get active RX buffer
    uint8_t* rx_buf = wan_comm_get_active_rx_buffer(handle);
    
    // Setup transaction
    memset(&handle->spi_trans, 0, sizeof(spi_slave_transaction_t));
    handle->spi_trans.length = handle->config.rx_buffer_size * 8;  // in bits
    handle->spi_trans.rx_buffer = rx_buf;
    handle->spi_trans.tx_buffer = handle->tx_buffer;
    handle->spi_trans.user = handle;
    
    // Queue transaction
    esp_err_t ret = spi_slave_queue_trans(handle->config.host_id, &handle->spi_trans, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue SPI transaction: %s", esp_err_to_name(ret));
        return WAN_COMM_ERR_BUS_BUSY;
    }
    
    return WAN_COMM_OK;
}

/**
 * @brief Get active RX buffer
 */
static uint8_t* wan_comm_get_active_rx_buffer(wan_comm_handle_t handle) {
    return (handle->rx_buffer_pool.active_buffer == 0) ? 
           handle->rx_buffer_pool.buffer_a : 
           handle->rx_buffer_pool.buffer_b;
}

/**
 * @brief Swap RX buffers
 */
static uint8_t* wan_comm_swap_rx_buffers(wan_comm_handle_t handle) {
    if (xSemaphoreTake(handle->rx_buffer_pool.swap_mutex, 0) != pdTRUE) {
        return NULL;
    }
    
    // Get current processing buffer before swap
    uint8_t* processing_buffer = (handle->rx_buffer_pool.active_buffer == 0) ? 
                                  handle->rx_buffer_pool.buffer_a : 
                                  handle->rx_buffer_pool.buffer_b;
    
    // Swap active buffer
    handle->rx_buffer_pool.active_buffer = (handle->rx_buffer_pool.active_buffer == 0) ? 1 : 0;
    
    xSemaphoreGive(handle->rx_buffer_pool.swap_mutex);
    
    return processing_buffer;
}

/**
 * @brief Processing task
 */
static void wan_comm_processing_task(void* arg) {
    wan_comm_handle_t handle = (wan_comm_handle_t)arg;
    spi_slave_transaction_t* trans;
    
    ESP_LOGI(TAG, "Processing task started");
    
    while (handle->is_running) {
        // Wait for transaction completion
        esp_err_t ret = spi_slave_get_trans_result(handle->config.host_id, &trans, portMAX_DELAY);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get transaction result: %s", esp_err_to_name(ret));
            continue;
        }

        // Check if this was a TX-only transaction
        if (handle->is_tx_only) {
            ESP_LOGI(TAG, "TX transaction complete, skipping parse");
            handle->is_tx_only = false;  // Reset flag
            wan_comm_start_receive_transaction(handle);  // Back to RX
            continue;
        }
        
        if (trans->trans_len == 0) {
            // Empty transaction, restart
            wan_comm_start_receive_transaction(handle);
            continue;
        }
        
        // Convert trans_len from bits to bytes
        size_t received_bytes = trans->trans_len / 8;
        
        ESP_LOGI(TAG, "Transaction complete: %d bytes received", received_bytes);
        
        // Swap buffers to allow next transaction to start immediately
        uint8_t* processing_buffer = wan_comm_swap_rx_buffers(handle);
        
        // Start next receive transaction immediately (double buffering)
        wan_comm_start_receive_transaction(handle);
        
        if (processing_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to swap buffers");
            continue;
        }
        
        // Parse packet
        uint16_t header_type;
        uint8_t* payload;
        uint16_t payload_length;
        
        wan_comm_status_t status = wan_comm_parse_packet(
            processing_buffer, 
            received_bytes, 
            &header_type, 
            &payload, 
            &payload_length
        );
        
        if (status != WAN_COMM_OK) {
            wan_comm_report_error(handle, status, "packet parse error");
            continue;
        }
        
        // Dispatch to appropriate callback
        if (header_type == WAN_COMM_HEADER_CF) {
            // Command packet
            handle->commands_received++;
            if (handle->config.on_command_received) {
                handle->config.on_command_received(payload, payload_length, handle->config.user_data);
            }
        } else if (header_type == WAN_COMM_HEADER_DT) {
            // Data packet
            handle->data_packets_received++;
            if (handle->config.on_data_received) {
                handle->config.on_data_received(payload, payload_length, handle->config.user_data);
            }
        } else {
            wan_comm_report_error(handle, WAN_COMM_ERR_INVALID_HEADER, "unknown header type");
        }
    }
    
    ESP_LOGI(TAG, "Processing task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Parse packet
 */
static wan_comm_status_t wan_comm_parse_packet(uint8_t* buffer, size_t length, 
                                               uint16_t* header_type, 
                                               uint8_t** payload, 
                                               uint16_t* payload_length) {
    if (buffer == NULL || length < WAN_COMM_HEADER_SIZE) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    // Extract header (first 2 bytes)
    *header_type = (buffer[0] << 8) | buffer[1];
    
    // Validate header
    if (*header_type != WAN_COMM_HEADER_CF && *header_type != WAN_COMM_HEADER_DT) {
        ESP_LOGE(TAG, "Invalid header: 0x%04X", *header_type);
        return WAN_COMM_ERR_INVALID_HEADER;
    }
    
    // Extract payload
    *payload = &buffer[WAN_COMM_HEADER_SIZE];
    *payload_length = length - WAN_COMM_HEADER_SIZE;
    
    ESP_LOGI(TAG, "Packet parsed: Header=0x%04X, Payload=%d bytes", *header_type, *payload_length);
    
    return WAN_COMM_OK;
}

/**
 * @brief Report error
 */
static void wan_comm_report_error(wan_comm_handle_t handle, 
                                 wan_comm_status_t error, 
                                 const char* context) {
    if (handle == NULL) {
        return;
    }
    
    handle->last_error = error;
    handle->error_count++;
    
    ESP_LOGE(TAG, "Error: %d, Context: %s", error, context);
    
    // Call user error callback if registered
    if (handle->config.error_callback != NULL) {
        handle->config.error_callback(error, context, handle->config.user_data);
    }
}
