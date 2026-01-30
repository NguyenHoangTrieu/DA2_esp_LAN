#include "wan_comm.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "WAN_COMM_MASTER";

/**
 * @brief DMA TX Buffer Structure
 * Accumulates multiple frames before transmission
 */
typedef struct {
    uint8_t buffer[WAN_COMM_DMA_BUFFER_SIZE];  // 4096 bytes
    size_t used;                                // Current write position
    uint32_t frame_count;                       // Frames in buffer
} dma_tx_buffer_t;

/**
 * @brief Internal handle structure
 */
struct wan_comm_handle_s {
    // Configuration
    wan_comm_config_t config;
    
    // SPI master device handle
    spi_device_handle_t spi_device;
    
    // Legacy DMA-aligned buffers for RX only
    uint8_t *rx_buffer;
    size_t rx_buffer_size_aligned;
    
    // DMA TX Buffer - replaces legacy tx_buffer
    dma_tx_buffer_t dma_tx;
    
    // Synchronization
    SemaphoreHandle_t transfer_mutex;
    
    // State
    bool is_initialized;
    wan_comm_status_t last_error;
    
    // Statistics
    uint32_t packets_sent;
    uint32_t dma_flushes;
    uint32_t error_count;
    
    // GPIO ISR
    bool gpio_isr_configured;
    wan_comm_data_ready_callback_t data_ready_callback;
    void *callback_user_arg;
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static wan_comm_status_t wan_comm_validate_transaction(wan_comm_handle_t handle, uint16_t length);
static void wan_comm_report_error(wan_comm_handle_t handle, wan_comm_status_t error, const char *context);
static bool is_dma_aligned(const void *ptr, size_t size);
static size_t calculate_dma_descriptors(size_t buffer_size);
static esp_err_t setup_data_ready_isr(wan_comm_handle_t handle, int gpio_pin);
static esp_err_t dma_buffer_add_frame(wan_comm_handle_t handle, const uint8_t *frame, size_t len);
static esp_err_t dma_buffer_flush(wan_comm_handle_t handle);

// ============================================================================
// GPIO ISR HANDLER
// ============================================================================

static void IRAM_ATTR wan_comm_gpio_isr_handler(void *arg) {
    wan_comm_handle_t handle = (wan_comm_handle_t)arg;
    if (handle && handle->data_ready_callback) {
        handle->data_ready_callback(handle->callback_user_arg);
    }
}

// ============================================================================
// HELPER MACROS
// ============================================================================

#define CLEANUP_INIT(handle) do { \
    if (handle) { \
        if (handle->transfer_mutex) vSemaphoreDelete(handle->transfer_mutex); \
        if (handle->rx_buffer) heap_caps_free(handle->rx_buffer); \
        if (handle->spi_device) spi_bus_remove_device(handle->spi_device); \
        spi_bus_free(handle->config.host_id); \
        free(handle); \
    } \
} while(0)

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

wan_comm_status_t wan_comm_init(const wan_comm_config_t *config, wan_comm_handle_t *handle) {
    if (!config || !handle) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "QSPI Master Initialization (LAN MCU)");
    ESP_LOGI(TAG, "============================================");
    
    // Validate GPIO pins
    if (config->gpio_sck < 0 || config->gpio_cs < 0 || 
        config->gpio_io0 < 0 || config->gpio_io1 < 0) {
        ESP_LOGE(TAG, "Invalid GPIO configuration");
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    if (config->enable_quad_mode && (config->gpio_io2 < 0 || config->gpio_io3 < 0)) {
        ESP_LOGE(TAG, "QSPI mode requires IO2 and IO3 pins");
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    // Allocate handle
    wan_comm_handle_t h = (wan_comm_handle_t)calloc(1, sizeof(struct wan_comm_handle_s));
    if (!h) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return WAN_COMM_ERR_NOMEM;
    }
    
    // Copy configuration
    memcpy(&h->config, config, sizeof(wan_comm_config_t));
    
    // Set defaults
    if (h->config.clock_speed_hz == 0) h->config.clock_speed_hz = WAN_COMM_QSPI_CLOCK_HZ;
    if (h->config.queue_size == 0) h->config.queue_size = WAN_COMM_TRANS_QUEUE_SIZE;
    if (h->config.dma_channel == 0) h->config.dma_channel = SPI_DMA_CH_AUTO;
    if (h->config.rx_buffer_size == 0) h->config.rx_buffer_size = WAN_COMM_DEFAULT_RX_BUFFER;
    
    // Warn if clock speed != 40 MHz
    if (h->config.clock_speed_hz != WAN_COMM_QSPI_CLOCK_HZ) {
        ESP_LOGW(TAG, "Clock speed %lu Hz differs from design doc (40 MHz)", h->config.clock_speed_hz);
    }
    
    // Auto-align RX buffer size
    h->rx_buffer_size_aligned = DMA_ALIGN_SIZE(h->config.rx_buffer_size);
    
    // Validate DMA descriptor count for RX only (TX uses fixed 4KB)
    size_t rx_desc_count = calculate_dma_descriptors(h->rx_buffer_size_aligned);
    if (rx_desc_count > WAN_COMM_MAX_DMA_DESCRIPTORS) {
        ESP_LOGE(TAG, "RX buffer requires %zu descriptors (max %d)", 
                 rx_desc_count, WAN_COMM_MAX_DMA_DESCRIPTORS);
        free(h);
        return WAN_COMM_ERR_NOMEM;
    }
    
    ESP_LOGI(TAG, "Buffer Configuration:");
    ESP_LOGI(TAG, "  DMA TX: %d bytes (fixed, buffered)", WAN_COMM_DMA_BUFFER_SIZE);
    ESP_LOGI(TAG, "  RX: %zu -> %zu bytes aligned, %zu DMA descriptors",
             h->config.rx_buffer_size, h->rx_buffer_size_aligned, rx_desc_count);
    
    // Allocate RX buffer only (DMA-aligned)
    h->rx_buffer = (uint8_t*)heap_caps_aligned_alloc(DMA_ALIGNMENT, 
                                                      h->rx_buffer_size_aligned, 
                                                      MALLOC_CAP_DMA);
    if (!h->rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX DMA buffer");
        free(h);
        return WAN_COMM_ERR_NOMEM;
    }
    
    // Verify DMA alignment
    if (!is_dma_aligned(h->rx_buffer, h->rx_buffer_size_aligned)) {
        ESP_LOGE(TAG, "RX buffer alignment verification FAILED");
        CLEANUP_INIT(h);
        return WAN_COMM_ERR_DMA_ALIGN;
    }
    
    ESP_LOGI(TAG, "DMA Buffers Allocated:");
    ESP_LOGI(TAG, "  TX: static buffer (4KB, accumulation)");
    ESP_LOGI(TAG, "  RX: %p (4-byte aligned)", h->rx_buffer);
    
    // Initialize DMA TX buffer
    memset(&h->dma_tx, 0, sizeof(dma_tx_buffer_t));
    
    // Clear RX buffer
    memset(h->rx_buffer, 0, h->rx_buffer_size_aligned);
    
    // Create transfer mutex
    h->transfer_mutex = xSemaphoreCreateMutex();
    if (!h->transfer_mutex) {
        ESP_LOGE(TAG, "Failed to create transfer mutex");
        CLEANUP_INIT(h);
        return WAN_COMM_ERR_NOMEM;
    }
    
    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->gpio_io0,
        .miso_io_num = config->gpio_io1,
        .sclk_io_num = config->gpio_sck,
        .quadwp_io_num = config->enable_quad_mode ? config->gpio_io2 : -1,
        .quadhd_io_num = config->enable_quad_mode ? config->gpio_io3 : -1,
        .max_transfer_sz = WAN_COMM_DMA_BUFFER_SIZE,  // Fixed 4KB for DMA buffer
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS
    };
    
    ESP_LOGI(TAG, "SPI Master Configuration:");
    ESP_LOGI(TAG, "  Host: SPI%d, Mode: %d, Queue Size: %d",
             config->host_id + 1, config->mode, config->queue_size);
    ESP_LOGI(TAG, "  GPIO: CLK=%d, CS=%d, IO0=%d, IO1=%d, IO2=%d, IO3=%d",
             config->gpio_sck, config->gpio_cs, config->gpio_io0, 
             config->gpio_io1, config->gpio_io2, config->gpio_io3);
    ESP_LOGI(TAG, "  Clock: %lu Hz (%.1f MHz)",
             h->config.clock_speed_hz, h->config.clock_speed_hz / 1000000.0);
    ESP_LOGI(TAG, "  QSPI Mode: %s (4-bit parallel)",
             config->enable_quad_mode ? "Enabled" : "Disabled");
    
    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(config->host_id, &bus_cfg, config->dma_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        CLEANUP_INIT(h);
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    // Configure SPI device (master)
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = config->mode,
        .clock_speed_hz = h->config.clock_speed_hz,
        .spics_io_num = config->gpio_cs,
        .queue_size = config->queue_size,
        .flags = config->enable_quad_mode ? 0 : 0,  // QIO set per transaction
        .pre_cb = NULL,
        .post_cb = NULL,
        .input_delay_ns = 0
    };
    
    // Add device to bus
    ret = spi_bus_add_device(config->host_id, &dev_cfg, &h->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(config->host_id);
        CLEANUP_INIT(h);
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    // Initialize state
    h->is_initialized = true;
    h->last_error = WAN_COMM_OK;
    h->packets_sent = 0;
    h->dma_flushes = 0;
    h->error_count = 0;
    
    *handle = h;
    
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "QSPI Master Ready - Driving 40 MHz Clock");
    ESP_LOGI(TAG, "============================================");
    
    // Calculate theoretical throughput
    uint32_t bits_per_transfer = config->enable_quad_mode ? 4 : 1;
    uint32_t theoretical_mbps = (h->config.clock_speed_hz * bits_per_transfer) / 1000000;
    ESP_LOGI(TAG, "Theoretical Throughput: %lu Mbps (%.1f MB/s)",
             theoretical_mbps, theoretical_mbps / 8.0);
    ESP_LOGI(TAG, "Expected Practical: 8-12 MB/s (accounting for overhead)");
    ESP_LOGI(TAG, "Timing: ACK=%dms, DQ retry=%dms×%d",
             WAN_COMM_ACK_TIMEOUT_MS, WAN_COMM_DQ_RETRY_MS, WAN_COMM_DQ_RETRY_COUNT);
    
    // Setup data-ready ISR
    h->gpio_isr_configured = false;
    h->data_ready_callback = NULL;
    h->callback_user_arg = NULL;
    
    if (config->gpio_data_ready_input >= 0) {
        if (setup_data_ready_isr(h, config->gpio_data_ready_input) == ESP_OK) {
            h->gpio_isr_configured = true;
            ESP_LOGI(TAG, "Data-Ready ISR:");
            ESP_LOGI(TAG, "  Pin: GPIO%d, Trigger: Rising Edge, Latency: <5ms",
                     config->gpio_data_ready_input);
        } else {
            ESP_LOGW(TAG, "Failed to setup GPIO%d ISR, data-ready disabled",
                     config->gpio_data_ready_input);
        }
    } else {
        ESP_LOGI(TAG, "Data-Ready ISR: Disabled (gpio_data_ready_input = -1)");
    }
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_deinit(wan_comm_handle_t handle) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Deinitializing QSPI master");
    
    // Flush any pending DMA buffer
    if (handle->dma_tx.used > 0) {
        ESP_LOGI(TAG, "Flushing pending DMA buffer (%zu bytes)", handle->dma_tx.used);
        dma_buffer_flush(handle);
    }
    
    // Remove ISR handler
    if (handle->gpio_isr_configured && handle->config.gpio_data_ready_input >= 0) {
        gpio_isr_handler_remove(handle->config.gpio_data_ready_input);
        ESP_LOGI(TAG, "GPIO%d ISR handler removed", handle->config.gpio_data_ready_input);
    }
    
    // Remove SPI device
    if (handle->spi_device) {
        spi_bus_remove_device(handle->spi_device);
    }
    
    // Free SPI bus
    spi_bus_free(handle->config.host_id);
    
    // Free resources
    if (handle->rx_buffer) {
        heap_caps_free(handle->rx_buffer);
    }
    
    if (handle->transfer_mutex) {
        vSemaphoreDelete(handle->transfer_mutex);
    }
    
    // Print final statistics
    ESP_LOGI(TAG, "Final Statistics:");
    ESP_LOGI(TAG, "  Packets TX: %lu, DMA Flushes: %lu, Errors: %lu",
             handle->packets_sent, handle->dma_flushes, handle->error_count);
    
    handle->is_initialized = false;
    free(handle);
    
    ESP_LOGI(TAG, "QSPI master deinitialized");
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_send_command(wan_comm_handle_t handle, 
                                         const uint8_t *command_payload, 
                                         uint16_t length) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (!command_payload || length == 0) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    wan_comm_status_t status = wan_comm_validate_transaction(handle, length);
    if (status != WAN_COMM_OK) {
        return status;
    }
    
    // Take mutex
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "send_command mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    // Build frame: [CF header][payload]
    uint8_t frame[WAN_COMM_HEADER_SIZE + length];
    frame[0] = (WAN_COMM_HEADER_CF >> 8) & 0xFF;
    frame[1] = WAN_COMM_HEADER_CF & 0xFF;
    memcpy(&frame[WAN_COMM_HEADER_SIZE], command_payload, length);
    
    uint16_t total_length = length + WAN_COMM_HEADER_SIZE;
    
    // Add to DMA buffer
    esp_err_t ret = dma_buffer_add_frame(handle, frame, total_length);
    
    xSemaphoreGive(handle->transfer_mutex);
    
    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "send_command DMA error");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    
    handle->packets_sent++;
    ESP_LOGD(TAG, "QSPI TX: CF %u bytes (total=%lu)", total_length, handle->packets_sent);
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_send_data(wan_comm_handle_t handle, 
                                      const uint8_t *data_payload, 
                                      uint16_t length) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (!data_payload || length == 0) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    wan_comm_status_t status = wan_comm_validate_transaction(handle, length);
    if (status != WAN_COMM_OK) {
        return status;
    }
    
    // Take mutex
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "send_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    // Build frame: [DT header][payload]
    uint8_t frame[WAN_COMM_HEADER_SIZE + length];
    frame[0] = (WAN_COMM_HEADER_DT >> 8) & 0xFF;
    frame[1] = WAN_COMM_HEADER_DT & 0xFF;
    memcpy(&frame[WAN_COMM_HEADER_SIZE], data_payload, length);
    
    uint16_t total_length = length + WAN_COMM_HEADER_SIZE;
    
    // Add to DMA buffer
    esp_err_t ret = dma_buffer_add_frame(handle, frame, total_length);
    
    xSemaphoreGive(handle->transfer_mutex);
    
    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "send_data DMA error");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    
    handle->packets_sent++;
    ESP_LOGD(TAG, "QSPI TX: DT %u bytes (total=%lu)", total_length, handle->packets_sent);
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_request_data(wan_comm_handle_t handle, 
                                         uint8_t *rx_buffer, 
                                         uint16_t length_to_read) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (!rx_buffer || length_to_read == 0) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    wan_comm_status_t status = wan_comm_validate_transaction(handle, length_to_read);
    if (status != WAN_COMM_OK) {
        return status;
    }
    
    // Take mutex
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "request_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    // Build dummy TX packet: [DQ header][zeros] - polling
    memset(handle->rx_buffer, 0, length_to_read);
    handle->rx_buffer[0] = (WAN_COMM_HEADER_DQ >> 8) & 0xFF;
    handle->rx_buffer[1] = WAN_COMM_HEADER_DQ & 0xFF;
    
    // Setup full-duplex transaction (QIO mode if enabled)
    spi_transaction_t trans = {0};
    trans.flags = handle->config.enable_quad_mode ? SPI_TRANS_MODE_QIO : 0;
    trans.length = length_to_read * 8;      // Total bits to transfer
    trans.rxlength = length_to_read * 8;    // Bits to receive
    trans.tx_buffer = handle->rx_buffer;    // TX: DQ header + zeros (reuse RX buffer)
    trans.rx_buffer = handle->rx_buffer;    // RX: Slave response
    
    // Transmit (blocking, full-duplex)
    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    
    if (ret == ESP_OK) {
        // Copy received data to user buffer
        memcpy(rx_buffer, handle->rx_buffer, length_to_read);
        ESP_LOGD(TAG, "QSPI RX: DQ %u bytes", length_to_read);
    }
    
    xSemaphoreGive(handle->transfer_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QSPI RX failed: %s", esp_err_to_name(ret));
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "request_data SPI error");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_transceive(wan_comm_handle_t handle, 
                                       const uint8_t *tx_data, 
                                       uint16_t tx_length,
                                       uint8_t *rx_buffer, 
                                       uint16_t rx_length) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (!tx_data || !rx_buffer || tx_length == 0 || rx_length == 0) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    uint16_t max_length = (tx_length > rx_length) ? tx_length : rx_length;
    wan_comm_status_t status = wan_comm_validate_transaction(handle, max_length);
    if (status != WAN_COMM_OK) {
        return status;
    }
    
    // Take mutex
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "transceive mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    // Copy TX data to internal buffer (reuse RX buffer for TX)
    memset(handle->rx_buffer, 0, handle->rx_buffer_size_aligned);
    memcpy(handle->rx_buffer, tx_data, tx_length);
    
    // Setup full-duplex transaction (QIO mode if enabled)
    spi_transaction_t trans = {0};
    trans.flags = handle->config.enable_quad_mode ? SPI_TRANS_MODE_QIO : 0;
    trans.length = max_length * 8;
    trans.rxlength = rx_length * 8;
    trans.tx_buffer = handle->rx_buffer;
    trans.rx_buffer = handle->rx_buffer;
    
    // Transmit (blocking, full-duplex)
    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    
    if (ret == ESP_OK) {
        memcpy(rx_buffer, handle->rx_buffer, rx_length);
        handle->packets_sent++;
        ESP_LOGD(TAG, "QSPI Transceive: TX=%u, RX=%u bytes", tx_length, rx_length);
    }
    
    xSemaphoreGive(handle->transfer_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QSPI transceive failed: %s", esp_err_to_name(ret));
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "transceive SPI error");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_flush_dma_buffer(wan_comm_handle_t handle) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    // Take mutex
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        return WAN_COMM_ERR_TIMEOUT;
    }
    
    esp_err_t ret = dma_buffer_flush(handle);
    
    xSemaphoreGive(handle->transfer_mutex);
    
    return (ret == ESP_OK) ? WAN_COMM_OK : WAN_COMM_ERR_BUS_BUSY;
}

wan_comm_status_t wan_comm_register_data_ready_callback(wan_comm_handle_t handle, 
                                                         wan_comm_data_ready_callback_t callback,
                                                         void *user_arg) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (!handle->gpio_isr_configured) {
        ESP_LOGE(TAG, "GPIO ISR not configured");
        return WAN_COMM_ERR_INVALID_STATE;
    }
    
    handle->data_ready_callback = callback;
    handle->callback_user_arg = user_arg;
    
    ESP_LOGI(TAG, "Data-ready callback %s", callback ? "registered" : "unregistered");
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_get_last_error(wan_comm_handle_t handle) {
    if (!handle) {
        return WAN_COMM_ERR_INVALID_ARG;
    }
    return handle->last_error;
}

wan_comm_status_t wan_comm_get_statistics(wan_comm_handle_t handle, 
                                           uint32_t *packets_sent, 
                                           uint32_t *errors) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    if (packets_sent) *packets_sent = handle->packets_sent;
    if (errors) *errors = handle->error_count;
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_clear_error_count(wan_comm_handle_t handle) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    
    handle->error_count = 0;
    ESP_LOGI(TAG, "Error count cleared");
    return WAN_COMM_OK;
}

/**
 * @brief Add frame to DMA buffer with automatic padding
 * 
 * @param handle Handle
 * @param frame Complete frame (header + payload)
 * @param len Frame length
 * @return ESP_OK on success
 */
static esp_err_t dma_buffer_add_frame(wan_comm_handle_t handle, const uint8_t *frame, size_t len) {
    // Check if frame fits in remaining buffer space
    if (handle->dma_tx.used + len <= WAN_COMM_DMA_BUFFER_SIZE) {
        // Add frame to buffer
        memcpy(&handle->dma_tx.buffer[handle->dma_tx.used], frame, len);
        handle->dma_tx.used += len;
        handle->dma_tx.frame_count++;
        
        ESP_LOGV(TAG, "Frame added to DMA buffer: %zu bytes (%zu/%d used, %lu frames)",
                 len, handle->dma_tx.used, WAN_COMM_DMA_BUFFER_SIZE, handle->dma_tx.frame_count);
        
        return ESP_OK;
    } else {
        // Buffer full - pad with 0x00 and flush
        size_t padding = WAN_COMM_DMA_BUFFER_SIZE - handle->dma_tx.used;
        memset(&handle->dma_tx.buffer[handle->dma_tx.used], 0x00, padding);  // Dummy bytes
        handle->dma_tx.used = WAN_COMM_DMA_BUFFER_SIZE;
        
        ESP_LOGD(TAG, "DMA buffer full - flushing with %zu bytes 0x00 padding", padding);
        
        // Flush buffer
        esp_err_t ret = dma_buffer_flush(handle);
        if (ret != ESP_OK) {
            return ret;
        }
        
        // Start new buffer with current frame
        memcpy(handle->dma_tx.buffer, frame, len);
        handle->dma_tx.used = len;
        handle->dma_tx.frame_count = 1;
        
        ESP_LOGV(TAG, "New DMA buffer started: %zu bytes", len);
        
        return ESP_OK;
    }
}

/**
 * @brief Flush DMA buffer to QSPI hardware
 * 
 * @param handle Handle
 * @return ESP_OK on success
 */
static esp_err_t dma_buffer_flush(wan_comm_handle_t handle) {
    if (handle->dma_tx.used == 0) {
        ESP_LOGV(TAG, "DMA buffer empty, nothing to flush");
        return ESP_OK;
    }
    
    ESP_LOGD(TAG, "Flushing DMA buffer: %zu bytes, %lu frames",
             handle->dma_tx.used, handle->dma_tx.frame_count);
    
    // Pad to 4-byte alignment if needed
    size_t aligned_size = DMA_ALIGN_SIZE(handle->dma_tx.used);
    if (aligned_size > handle->dma_tx.used) {
        size_t padding = aligned_size - handle->dma_tx.used;
        memset(&handle->dma_tx.buffer[handle->dma_tx.used], 0x00, padding);
        ESP_LOGV(TAG, "Added %zu bytes padding for DMA alignment", padding);
    }
    
    // Setup QSPI transaction (QIO mode if enabled)
    spi_transaction_t trans = {0};
    trans.flags = handle->config.enable_quad_mode ? SPI_TRANS_MODE_QIO : 0;
    trans.length = aligned_size * 8;  // Bits
    trans.tx_buffer = handle->dma_tx.buffer;
    trans.rx_buffer = NULL;  // TX-only for flushing accumulated frames
    
    // Execute transaction (blocking)
    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    
    if (ret == ESP_OK) {
        handle->dma_flushes++;
        ESP_LOGD(TAG, "DMA buffer flushed successfully (flush #%lu)", handle->dma_flushes);
        
        // Reset buffer
        handle->dma_tx.used = 0;
        handle->dma_tx.frame_count = 0;
    } else {
        ESP_LOGE(TAG, "DMA buffer flush failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// HELPER FUNCTIONS

static wan_comm_status_t wan_comm_validate_transaction(wan_comm_handle_t handle, uint16_t length) {
    if (length > WAN_COMM_MAX_TRANSFER_SIZE) {
        ESP_LOGE(TAG, "Transfer size %u exceeds maximum %d", length, WAN_COMM_MAX_TRANSFER_SIZE);
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    // Check against RX buffer size (TX uses DMA buffer)
    if (length > handle->rx_buffer_size_aligned) {
        ESP_LOGE(TAG, "Transfer size %u exceeds RX buffer size %zu",
                 length, handle->rx_buffer_size_aligned);
        return WAN_COMM_ERR_INVALID_ARG;
    }
    
    return WAN_COMM_OK;
}

static void wan_comm_report_error(wan_comm_handle_t handle, wan_comm_status_t error, const char *context) {
    if (!handle) return;
    
    handle->last_error = error;
    handle->error_count++;
    
    ESP_LOGE(TAG, "Error #%lu (code=%d): %s", handle->error_count, error, context);
}

static bool is_dma_aligned(const void *ptr, size_t size) {
    uintptr_t addr = (uintptr_t)ptr;
    return (addr % DMA_ALIGNMENT == 0) && (size % DMA_ALIGNMENT == 0);
}

static size_t calculate_dma_descriptors(size_t buffer_size) {
    return (buffer_size + WAN_COMM_DMA_DESCRIPTOR_SIZE - 1) / WAN_COMM_DMA_DESCRIPTOR_SIZE;
}

static esp_err_t setup_data_ready_isr(wan_comm_handle_t handle, int gpio_pin) {
    // Configure GPIO as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull-down for idle LOW
        .intr_type = GPIO_INTR_POSEDGE         // Rising edge trigger
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO%d: %s", gpio_pin, esp_err_to_name(ret));
        return ret;
    }
    
    // Install ISR service if not already installed
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add ISR handler
    ret = gpio_isr_handler_add(gpio_pin, wan_comm_gpio_isr_handler, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "GPIO%d ISR configured (rising edge, <5ms response)", gpio_pin);
    return ESP_OK;
}
