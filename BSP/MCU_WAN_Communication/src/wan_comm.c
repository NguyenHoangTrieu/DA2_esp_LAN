/**
 * @file wan_comm.c
 * @brief WAN MCU Communication Library Implementation (SPI Slave)
 */

#include "wan_comm.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"

static const char* TAG = "WAN_COMM";

/**
 * @brief Internal handle structure
 */
struct wan_comm_handle_s {
    // Configuration
    wan_comm_config_t config;
    
    // Buffers (DMA-capable)
    uint8_t* rx_buffer;
    uint8_t* tx_buffer;
    size_t buffer_size;
    SemaphoreHandle_t tx_buffer_mutex;
    
    // Processing
    TaskHandle_t processing_task_handle;
    
    // State
    bool is_initialized;
    bool is_running;
    wan_comm_status_t last_error;
    
    // Statistics
    uint32_t commands_received;
    uint32_t data_packets_received;
    uint32_t error_count;
};

// Global handle for callbacks
static wan_comm_handle_t g_wan_handle = NULL;

// Forward declarations
static void wan_comm_processing_task(void* arg);
static wan_comm_status_t wan_comm_parse_packet(uint8_t* buffer, size_t length,
                                               uint16_t* header_type,
                                               uint8_t** payload,
                                               uint16_t* payload_length);
static void wan_comm_report_error(wan_comm_handle_t handle, wan_comm_status_t error, const char* context);

/**
 * @brief Post-setup callback - signal slave is ready
 */
static void IRAM_ATTR wan_slave_post_setup_cb(spi_slave_transaction_t *trans) {
    if (g_wan_handle != NULL && g_wan_handle->config.gpio_handshake != GPIO_NUM_NC) {
        gpio_set_level(g_wan_handle->config.gpio_handshake, 1);  // Ready
    }
}

/**
 * @brief Post-transaction callback - signal slave is busy
 */
static void IRAM_ATTR wan_slave_post_trans_cb(spi_slave_transaction_t *trans) {
    if (g_wan_handle != NULL && g_wan_handle->config.gpio_handshake != GPIO_NUM_NC) {
        gpio_set_level(g_wan_handle->config.gpio_handshake, 0);  // Busy
    }
}

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
    
    ESP_LOGI(TAG, "Initializing WAN communication library (Slave mode with spi_slave_transmit)");
    
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
        h->config.rx_buffer_size = WAN_COMM_FIXED_TRANSFER_SIZE;
    }
    if (h->config.tx_buffer_size == 0) {
        h->config.tx_buffer_size = WAN_COMM_FIXED_TRANSFER_SIZE;
    }
    if (h->config.dma_channel == 0) {
        h->config.dma_channel = SPI_DMA_CH_AUTO;
    }
    
    h->buffer_size = WAN_COMM_FIXED_TRANSFER_SIZE;
    
    // Allocate DMA-capable buffers
    h->rx_buffer = (uint8_t*)heap_caps_malloc(h->buffer_size, MALLOC_CAP_DMA);
    h->tx_buffer = (uint8_t*)heap_caps_malloc(h->buffer_size, MALLOC_CAP_DMA);
    h->tx_buffer_mutex = xSemaphoreCreateMutex();
    
    if (h->rx_buffer == NULL || h->tx_buffer == NULL || h->tx_buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers or mutex");
        free(h->rx_buffer);
        free(h->tx_buffer);
        if (h->tx_buffer_mutex) vSemaphoreDelete(h->tx_buffer_mutex);
        free(h);
        return WAN_COMM_ERR_NO_MEM;
    }
    
    // Initialize TX buffer with default pattern
    memset(h->tx_buffer, 0xA5, h->buffer_size);
    
    // Configure handshake GPIO as output
    if (config->gpio_handshake != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << config->gpio_handshake),
            .pull_down_en = 0,
            .pull_up_en = 0
        };
        gpio_config(&io_conf);
        gpio_set_level(config->gpio_handshake, 0);  // Start LOW (busy)
        
        ESP_LOGI(TAG, "Handshake GPIO %d configured as OUTPUT", config->gpio_handshake);
    }
    
    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->gpio_io0,
        .miso_io_num = config->gpio_io1,
        .sclk_io_num = config->gpio_sck,
        .quadwp_io_num = config->enable_quad_mode ? config->gpio_io2 : -1,
        .quadhd_io_num = config->enable_quad_mode ? config->gpio_io3 : -1,
        .max_transfer_sz = h->buffer_size,
        .flags = 0
    };
    
    // Enable pull-ups on SPI lines
    gpio_set_pull_mode(config->gpio_io0, GPIO_PULLUP_ONLY);  // MOSI
    gpio_set_pull_mode(config->gpio_sck, GPIO_PULLUP_ONLY);  // SCK
    gpio_set_pull_mode(config->gpio_cs, GPIO_PULLUP_ONLY);   // CS
    
    // Configure SPI slave with callbacks
    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = config->gpio_cs,
        .flags = 0,
        .queue_size = 3,
        .mode = config->mode,
        .post_setup_cb = wan_slave_post_setup_cb,
        .post_trans_cb = wan_slave_post_trans_cb
    };
    
    esp_err_t ret = spi_slave_initialize(config->host_id, &bus_cfg, &slave_cfg, config->dma_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        free(h->rx_buffer);
        free(h->tx_buffer);
        vSemaphoreDelete(h->tx_buffer_mutex);
        free(h);
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    // Initialize state
    h->is_initialized = true;
    h->is_running = false;
    h->last_error = WAN_COMM_OK;
    h->commands_received = 0;
    h->data_packets_received = 0;
    h->error_count = 0;
    
    // Set global handle for callbacks
    g_wan_handle = h;
    
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
        free(h->rx_buffer);
        free(h->tx_buffer);
        vSemaphoreDelete(h->tx_buffer_mutex);
        free(h);
        g_wan_handle = NULL;
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    h->is_running = true;
    
    *handle = h;
    
    ESP_LOGI(TAG, "WAN communication initialized successfully");
    ESP_LOGI(TAG, "Mode: %d, Buffer size: %d bytes", config->mode, h->buffer_size);
    
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
    free(handle->rx_buffer);
    free(handle->tx_buffer);
    vSemaphoreDelete(handle->tx_buffer_mutex);
    
    handle->is_initialized = false;
    g_wan_handle = NULL;
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
    
    if (length > handle->buffer_size) {
        ESP_LOGE(TAG, "TX data length %d exceeds buffer size %d", length, handle->buffer_size);
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    // Lock TX buffer
    if (xSemaphoreTake(handle->tx_buffer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "load_tx_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    // Copy data to TX buffer and pad with zeros
    memset(handle->tx_buffer, 0, handle->buffer_size);
    memcpy(handle->tx_buffer, data_to_send, length);
    
    xSemaphoreGive(handle->tx_buffer_mutex);
    
    ESP_LOGD(TAG, "TX data loaded: %d bytes", length);
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
 * @brief Processing task
 */
static void wan_comm_processing_task(void* arg) {
    wan_comm_handle_t handle = (wan_comm_handle_t)arg;
    
    ESP_LOGI(TAG, "Processing task started (using spi_slave_transmit)");
    
    while (handle->is_running) {
        // Clear RX buffer with pattern
        memset(handle->rx_buffer, 0xA5, handle->buffer_size);
        
        // Setup transaction structure
        spi_slave_transaction_t trans = {
            .length = handle->buffer_size * 8,    // in bits
            .trans_len = 0,                        // will be filled after transaction
            .tx_buffer = handle->tx_buffer,
            .rx_buffer = handle->rx_buffer,
            .user = handle
        };

        esp_err_t ret = spi_slave_transmit(handle->config.host_id, &trans, portMAX_DELAY);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(ret));
            wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "spi_slave_transmit error");
            vTaskDelay(pdMS_TO_TICKS(10));  // Small delay before retry
            continue;
        }
        
        // Check if any data was received
        if (trans.trans_len == 0) {
            ESP_LOGW(TAG, "Empty transaction received");
            continue;
        }
        
        // Convert trans_len from bits to bytes
        size_t received_bytes = trans.trans_len / 8;
        ESP_LOGD(TAG, "Transaction complete: %d bytes received", received_bytes);
        
        // Parse packet
        uint16_t header_type;
        uint8_t* payload;
        uint16_t payload_length;
        
        wan_comm_status_t status = wan_comm_parse_packet(
            handle->rx_buffer,
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
    
    ESP_LOGD(TAG, "Packet parsed: Header=0x%04X, Payload=%d bytes", *header_type, *payload_length);
    
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
    
    ESP_LOGE(TAG, "Error #%lu: %d, Context: %s", handle->error_count, error, context);
    
    // Call user error callback if registered
    if (handle->config.error_callback != NULL) {
        handle->config.error_callback(error, context, handle->config.user_data);
    }
}
