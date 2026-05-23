#include "wan_comm.h"
#include "spi_framing.h"
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

    // Persistent DMA TX buffer for DQ polling requests (avoids malloc fragmentation)
    uint8_t *tx_request_buffer;

    // DMA TX scratch — single frame is built here per call, then transmitted.
    // Legacy "accumulation" model retired in P1.
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

    // P1 framing
    uint8_t  tx_seq;              /* rolling sequence used for outgoing frames  */
    uint16_t rx_prev_seq;         /* last RX seq seen (0xFFFF = none)            */
    spi_frame_stats_t frame_stats;

    /* P3.b cumulative ACK from slave. SPI_FRAME_ACK_NONE means "no ack yet".
     * Updated each time we parse a slave→master frame that carries a valid
     * ack_for field. Means: every master seq up to and including this value
     * has been received by the slave OK. Read atomically (16-bit aligned
     * loads are atomic on Xtensa); writes only happen under transfer_mutex. */
    volatile uint16_t last_acked_seq;

    /* P3.d: full-duplex flush RX scratch. flush_dma_locked() captures the
     * slave's MISO content here and walks it with spi_frame_parse_stream() to
     * dispatch each parsed frame to rx_frame_cb. Allocated DMA-capable. */
    uint8_t                *flush_rx_buffer;
    wan_comm_rx_frame_cb_t  rx_frame_cb;
    void                   *rx_frame_cb_user;

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

/* P1 framing helpers --------------------------------------------------------
 * transmit_framed_locked() expects the caller to already hold transfer_mutex.
 * It builds a SPI frame whose payload is [inner_hdr][inner_payload], pads to
 * 4-byte DMA alignment, and ships it via spi_device_transmit (TX-only).
 *
 * inner_hdr2 is the 2-byte application header (CF/DT/DQ in big-endian as on
 * the legacy wire); pass 0 to omit it.
 */
static esp_err_t transmit_framed_locked(wan_comm_handle_t handle,
                                        uint16_t inner_hdr2,
                                        const uint8_t *inner_payload,
                                        uint16_t inner_payload_len);

/* P3.c: flushes the dma_tx batch accumulator as a single TX-only transaction.
 * Caller must hold transfer_mutex. Defined later in this file alongside
 * transmit_framed_locked(). */
static esp_err_t flush_dma_locked(wan_comm_handle_t handle);

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
        if (handle->tx_request_buffer) heap_caps_free(handle->tx_request_buffer); \
        if (handle->flush_rx_buffer) heap_caps_free(handle->flush_rx_buffer); \
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
    ESP_LOGI(TAG, "SPI Master Initialization (LAN MCU)");
    ESP_LOGI(TAG, "============================================");
    
    // Validate GPIO pins
    if (config->gpio_sck < 0 || config->gpio_cs < 0 || 
        config->gpio_io0 < 0 || config->gpio_io1 < 0) {
        ESP_LOGE(TAG, "Invalid GPIO configuration");
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
    if (h->config.clock_speed_hz == 0) h->config.clock_speed_hz = WAN_COMM_SPI_CLOCK_HZ;
    if (h->config.queue_size == 0) h->config.queue_size = WAN_COMM_TRANS_QUEUE_SIZE;
    if (h->config.dma_channel == 0) h->config.dma_channel = SPI_DMA_CH_AUTO;
    if (h->config.rx_buffer_size == 0) h->config.rx_buffer_size = WAN_COMM_DEFAULT_RX_BUFFER;
    
    // Warn if clock speed != 40 MHz
    if (h->config.clock_speed_hz != WAN_COMM_SPI_CLOCK_HZ) {
        ESP_LOGW(TAG, "Clock speed %lu Hz differs from default (40 MHz)", h->config.clock_speed_hz);
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
    ESP_LOGI(TAG, "  Fixed transfer length: %u bytes", WAN_COMM_FIXED_XFER_LEN);
    
    // Allocate RX buffer only (DMA-aligned)
    h->rx_buffer = (uint8_t*)heap_caps_aligned_alloc(DMA_ALIGNMENT, 
                                                      h->rx_buffer_size_aligned, 
                                                      MALLOC_CAP_DMA);
    if (!h->rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX DMA buffer");
        free(h);
        return WAN_COMM_ERR_NOMEM;
    }
    
    // Allocate persistent TX request buffer for DQ polls (16KB, DMA-aligned)
    h->tx_request_buffer = (uint8_t*)heap_caps_aligned_alloc(DMA_ALIGNMENT,
                                                              WAN_COMM_DMA_BUFFER_SIZE,
                                                              MALLOC_CAP_DMA);
    if (!h->tx_request_buffer) {
        ESP_LOGE(TAG, "Failed to allocate TX request DMA buffer");
        heap_caps_free(h->rx_buffer);
        free(h);
        return WAN_COMM_ERR_NOMEM;
    }

    /* P3.d: full-duplex flush scratch (DMA-capable). */
    h->flush_rx_buffer = (uint8_t*)heap_caps_aligned_alloc(DMA_ALIGNMENT,
                                                            WAN_COMM_DMA_BUFFER_SIZE,
                                                            MALLOC_CAP_DMA);
    if (!h->flush_rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate flush RX DMA buffer");
        heap_caps_free(h->tx_request_buffer);
        heap_caps_free(h->rx_buffer);
        free(h);
        return WAN_COMM_ERR_NOMEM;
    }
    h->rx_frame_cb       = NULL;
    h->rx_frame_cb_user  = NULL;
    
    // Verify DMA alignment
    if (!is_dma_aligned(h->rx_buffer, h->rx_buffer_size_aligned)) {
        ESP_LOGE(TAG, "RX buffer alignment verification FAILED");
        CLEANUP_INIT(h);
        return WAN_COMM_ERR_DMA_ALIGN;
    }
    
    if (!is_dma_aligned(h->tx_request_buffer, WAN_COMM_DMA_BUFFER_SIZE)) {
        ESP_LOGE(TAG, "TX request buffer alignment verification FAILED");
        CLEANUP_INIT(h);
        return WAN_COMM_ERR_DMA_ALIGN;
    }
    
    ESP_LOGI(TAG, "DMA Buffers Allocated:");
    ESP_LOGI(TAG, "  TX: static buffer (16KB, accumulation)");
    ESP_LOGI(TAG, "  TX Request: %p (16KB, DQ polling)", h->tx_request_buffer);
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
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = WAN_COMM_DMA_BUFFER_SIZE,  // Fixed 4KB for DMA buffer
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS
    };
    
    ESP_LOGI(TAG, "SPI Master Configuration:");
    ESP_LOGI(TAG, "  Host: SPI%d, Mode: %d, Queue Size: %d",
             config->host_id + 1, config->mode, config->queue_size);
    ESP_LOGI(TAG, "  GPIO: CLK=%d, CS=%d, IO0=%d, IO1=%d",
             config->gpio_sck, config->gpio_cs, config->gpio_io0,
             config->gpio_io1);
    ESP_LOGI(TAG, "  Clock: %lu Hz (%.1f MHz)",
             h->config.clock_speed_hz, h->config.clock_speed_hz / 1000000.0);
    
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
        .flags = 0,
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
    h->tx_seq = 0;
    h->rx_prev_seq = 0xFFFFu;
    h->last_acked_seq = SPI_FRAME_ACK_NONE;
    memset(&h->frame_stats, 0, sizeof(h->frame_stats));

    *handle = h;
    
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "SPI Master Ready - Driving 40 MHz Clock");
    ESP_LOGI(TAG, "============================================");
    
    // Calculate theoretical throughput
    uint32_t theoretical_mbps = h->config.clock_speed_hz / 1000000;
    ESP_LOGI(TAG, "Theoretical Throughput: %lu Mbps (%.1f MB/s)",
             theoretical_mbps, theoretical_mbps / 8.0);
    ESP_LOGI(TAG, "Expected Practical: depends on payload and timing");
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
    
    ESP_LOGI(TAG, "Deinitializing SPI master");
    
    /* P1: legacy accumulation buffer retired. dma_tx is scratch only and
     * carries no pending data after the last send_*() call returned. */
    handle->dma_tx.used = 0;
    handle->dma_tx.frame_count = 0;


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
    if (handle->tx_request_buffer) {
        heap_caps_free(handle->tx_request_buffer);
    }
    if (handle->flush_rx_buffer) {
        heap_caps_free(handle->flush_rx_buffer);
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
    
    ESP_LOGI(TAG, "SPI master deinitialized");
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

    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "send_command mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }

    esp_err_t ret = transmit_framed_locked(handle, WAN_COMM_HEADER_CF,
                                            command_payload, length);
    xSemaphoreGive(handle->transfer_mutex);

    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "send_command transmit failed");
        return WAN_COMM_ERR_BUS_BUSY;
    }

    handle->packets_sent++;
    ESP_LOGD(TAG, "SPI TX framed: CF inner=%u (total tx=%lu)",
             length, handle->packets_sent);
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

    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "send_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }

    esp_err_t ret = transmit_framed_locked(handle, WAN_COMM_HEADER_DT,
                                            data_payload, length);
    xSemaphoreGive(handle->transfer_mutex);

    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "send_data transmit failed");
        return WAN_COMM_ERR_BUS_BUSY;
    }

    handle->packets_sent++;
    ESP_LOGD(TAG, "SPI TX framed: DT inner=%u (total tx=%lu)",
             length, handle->packets_sent);
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

    /* The slave will clock back at most (rx_buffer_size) bytes; size the
     * full-duplex transaction to cover both the DQ frame we send and the
     * maximum framed response we want to capture.  Honour the caller's
     * length_to_read as the *inner payload* budget. */
    size_t inner_budget = length_to_read;
    size_t transfer_len = SPI_FRAME_OVERHEAD + WAN_COMM_HEADER_SIZE + inner_budget;
    /* round up to 4-byte DMA alignment */
    transfer_len = (transfer_len + 3u) & ~((size_t)3u);
    if (transfer_len < SPI_FRAME_OVERHEAD + WAN_COMM_HEADER_SIZE) {
        transfer_len = SPI_FRAME_OVERHEAD + WAN_COMM_HEADER_SIZE;
    }
    if (transfer_len > handle->rx_buffer_size_aligned) {
        transfer_len = handle->rx_buffer_size_aligned;
    }
    if (transfer_len > WAN_COMM_DMA_BUFFER_SIZE) {
        transfer_len = WAN_COMM_DMA_BUFFER_SIZE;
    }

    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "request_data mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }

    /* P3.c: flush any pending batched master→slave frames first. Otherwise
     * the slave wouldn't have received them yet, and its ack_for response
     * would lag our newest seq → DQ poll loop spins for nothing. */
    {
        esp_err_t fret = flush_dma_locked(handle);
        if (fret != ESP_OK) {
            xSemaphoreGive(handle->transfer_mutex);
            wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "request_data pre-flush failed");
            return WAN_COMM_ERR_BUS_BUSY;
        }
    }

    uint8_t *tx_buffer = handle->tx_request_buffer;
    memset(tx_buffer, 0, transfer_len);

    /* Build framed DQ request: payload = [DQ_hi][DQ_lo]. */
    uint8_t dq_inner[WAN_COMM_HEADER_SIZE] = {
        (uint8_t)((WAN_COMM_HEADER_DQ >> 8) & 0xFFu),
        (uint8_t)(WAN_COMM_HEADER_DQ & 0xFFu),
    };
    /* DQ request: master doesn't ack the slave today, so ack_for=NONE. */
    size_t built = spi_frame_build(tx_buffer, transfer_len,
                                    SPI_FT_USER_BLOB, handle->tx_seq++,
                                    SPI_FRAME_ACK_NONE,
                                    dq_inner, sizeof(dq_inner));
    if (built == 0) {
        xSemaphoreGive(handle->transfer_mutex);
        wan_comm_report_error(handle, WAN_COMM_ERR_INVALID_ARG, "request_data frame build failed");
        return WAN_COMM_ERR_INVALID_ARG;
    }
    /* Bytes past the built frame are already zero; the slave will simply
     * clock them out while we read its response in the same transaction. */

    memset(handle->rx_buffer, 0, transfer_len);

    spi_transaction_t trans = {0};
    trans.flags = 0;
    trans.length   = transfer_len * 8;
    trans.rxlength = transfer_len * 8;
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = handle->rx_buffer;

    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);

    if (ret == ESP_OK) {
        /* Parse a single frame out of the RX buffer; if no SOF is found, fall
         * back to a raw copy so callers using a *very* old WAN firmware still
         * see something — but bump the resync counter so it shows up in stats.
         */
        spi_frame_view_t view;
        spi_frame_status_t st;
        size_t consumed = 0;
        bool ok = spi_frame_find(handle->rx_buffer, transfer_len,
                                  &view, &st, &handle->frame_stats, &consumed);
        if (ok) {
            spi_frame_track_seq(&handle->rx_prev_seq, view.seq, &handle->frame_stats);
            /* P3.b: harvest piggyback ack. Slave puts its max-master-seq-seen
             * in view.ack_for; we record it so callers can poll for ack. */
            if (view.ack_for != SPI_FRAME_ACK_NONE) {
                handle->last_acked_seq = view.ack_for;
            }
            uint16_t copy_len = (view.len < length_to_read) ? view.len : length_to_read;
            if (copy_len > 0 && view.payload) {
                memcpy(rx_buffer, view.payload, copy_len);
            }
            if (copy_len < length_to_read) {
                memset(&rx_buffer[copy_len], 0, length_to_read - copy_len);
            }
            ESP_LOGD(TAG, "SPI RX framed: seq=%u type=0x%02X ack_for=0x%04X inner=%u",
                     view.seq, view.type, view.ack_for, view.len);
        } else {
            /* No valid frame in the response window — common while the slave
             * has nothing pending to send (TX buffer still all zeros).  Hand
             * back zeros and let the application-level polling logic retry. */
            memset(rx_buffer, 0, length_to_read);
            ESP_LOGV(TAG, "SPI RX framed: no valid frame (status=%d)", (int)st);
        }
    }

    xSemaphoreGive(handle->transfer_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI RX failed: %s", esp_err_to_name(ret));
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
    
    // Setup full-duplex transaction
    spi_transaction_t trans = {0};
    trans.flags = 0;
    trans.length = max_length * 8;
    trans.rxlength = rx_length * 8;
    trans.tx_buffer = handle->rx_buffer;
    trans.rx_buffer = handle->rx_buffer;
    
    // Transmit (blocking, full-duplex)
    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    
    if (ret == ESP_OK) {
        memcpy(rx_buffer, handle->rx_buffer, rx_length);
        handle->packets_sent++;
        ESP_LOGI(TAG, "SPI Transceive: TX=%u, RX=%u bytes", tx_length, rx_length);
    }
    
    xSemaphoreGive(handle->transfer_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transceive failed: %s", esp_err_to_name(ret));
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "transceive SPI error");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_flush_dma_buffer(wan_comm_handle_t handle) {
    /* P3.c: actually flush. transmit_framed_locked() now appends to dma_tx
     * and only auto-flushes at WAN_COMM_BATCH_MAX_FRAMES; callers must invoke
     * this to push small batches out before going idle (uplink handler does
     * so every INTER_MCU_BATCH_INTERVAL_MS). */
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "flush mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }
    esp_err_t ret = flush_dma_locked(handle);
    xSemaphoreGive(handle->transfer_mutex);
    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "flush failed");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    return WAN_COMM_OK;
}

wan_comm_status_t wan_comm_get_framing_stats(wan_comm_handle_t handle,
                                              uint32_t *rx_frames_ok,
                                              uint32_t *rx_hdr_crc_fail,
                                              uint32_t *rx_payload_crc_fail,
                                              uint32_t *rx_resync_bytes,
                                              uint32_t *rx_seq_gap) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    if (rx_frames_ok)        *rx_frames_ok        = handle->frame_stats.frames_ok;
    if (rx_hdr_crc_fail)     *rx_hdr_crc_fail     = handle->frame_stats.hdr_crc_fail;
    if (rx_payload_crc_fail) *rx_payload_crc_fail = handle->frame_stats.payload_crc_fail;
    if (rx_resync_bytes)     *rx_resync_bytes     = handle->frame_stats.resync_bytes;
    if (rx_seq_gap)          *rx_seq_gap          = handle->frame_stats.seq_gap;
    return WAN_COMM_OK;
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

// ============================================================================
// P3.b — cumulative-ACK API
// ============================================================================

wan_comm_status_t wan_comm_send_data_get_seq(wan_comm_handle_t handle,
                                              const uint8_t *data_payload,
                                              uint16_t length,
                                              uint8_t *out_seq) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    if (!data_payload || length == 0 || !out_seq) {
        return WAN_COMM_ERR_INVALID_ARG;
    }

    wan_comm_status_t status = wan_comm_validate_transaction(handle, length);
    if (status != WAN_COMM_OK) {
        return status;
    }

    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        wan_comm_report_error(handle, WAN_COMM_ERR_TIMEOUT, "send_data_get_seq mutex timeout");
        return WAN_COMM_ERR_TIMEOUT;
    }

    /* Snapshot the seq that transmit_framed_locked is about to consume. */
    *out_seq = handle->tx_seq;
    esp_err_t ret = transmit_framed_locked(handle, WAN_COMM_HEADER_DT,
                                            data_payload, length);
    xSemaphoreGive(handle->transfer_mutex);

    if (ret != ESP_OK) {
        wan_comm_report_error(handle, WAN_COMM_ERR_BUS_BUSY, "send_data_get_seq transmit failed");
        return WAN_COMM_ERR_BUS_BUSY;
    }
    handle->packets_sent++;
    return WAN_COMM_OK;
}

uint16_t wan_comm_get_last_acked_seq(wan_comm_handle_t handle) {
    if (!handle || !handle->is_initialized) {
        return SPI_FRAME_ACK_NONE;
    }
    return handle->last_acked_seq;   /* volatile 16-bit read is atomic on Xtensa */
}

bool wan_comm_was_seq_acked(wan_comm_handle_t handle, uint8_t seq) {
    if (!handle || !handle->is_initialized) {
        return false;
    }
    uint16_t ack = handle->last_acked_seq;
    if (ack == SPI_FRAME_ACK_NONE) {
        return false;
    }
    /* Signed 8-bit modular comparison. As long as the outstanding window
     * stays under 128, (int8_t)(ack - seq) >= 0 iff ack covers seq. */
    int8_t delta = (int8_t)((uint8_t)ack - seq);
    return delta >= 0;
}

wan_comm_status_t wan_comm_register_rx_frame_callback(wan_comm_handle_t handle,
                                                       wan_comm_rx_frame_cb_t cb,
                                                       void *user) {
    if (!handle || !handle->is_initialized) {
        return WAN_COMM_ERR_NOT_INITIALIZED;
    }
    if (xSemaphoreTake(handle->transfer_mutex, pdMS_TO_TICKS(WAN_COMM_TIMEOUT_MS)) != pdTRUE) {
        return WAN_COMM_ERR_TIMEOUT;
    }
    handle->rx_frame_cb      = cb;
    handle->rx_frame_cb_user = user;
    xSemaphoreGive(handle->transfer_mutex);
    ESP_LOGI(TAG, "RX frame callback %s", cb ? "registered" : "unregistered");
    return WAN_COMM_OK;
}

/* P3.d: stream callback that bridges spi_frame_parse_stream() into the
 * handle's per-frame user callback, and also harvests piggyback ack_for. */
static void wan_comm_rx_stream_cb(const spi_frame_view_t *view, void *user) {
    wan_comm_handle_t handle = (wan_comm_handle_t)user;
    if (!handle) return;
    /* Update last_acked_seq from every parsed slave→master frame. */
    if (view->ack_for != SPI_FRAME_ACK_NONE) {
        handle->last_acked_seq = view->ack_for;
    }
    /* Track seq for stats. */
    spi_frame_track_seq(&handle->rx_prev_seq, view->seq, &handle->frame_stats);
    /* Fire the registered user callback. */
    if (handle->rx_frame_cb) {
        handle->rx_frame_cb(view, handle->rx_frame_cb_user);
    }
}

/* P3.c batching: how many frames to accumulate before forcing a flush.
 * Larger = better throughput (amortises per-transaction ~250 µs overhead),
 * worse first-frame latency. 8 × 2 KB payload = ~16 KB per transaction —
 * matches WAN_COMM_DMA_BUFFER_SIZE so we essentially fill the buffer.        */
#ifndef WAN_COMM_BATCH_MAX_FRAMES
#define WAN_COMM_BATCH_MAX_FRAMES   8
#endif

/**
 * @brief Issue the accumulated dma_tx.buffer as a single FULL-DUPLEX
 *        transaction. Capture the slave's MISO content into flush_rx_buffer,
 *        walk it with the framing parser, and dispatch each parsed frame to
 *        the registered RX callback (also updating last_acked_seq).
 *
 *        Caller must hold transfer_mutex. No-op when nothing is queued.
 *
 * P3.d note: the bench (bench_throughput.c) registers a callback that counts
 * BNC inner frames to provide the WAN→LAN throughput number. Production
 * traffic (handshake response, RTC response, etc.) is also visible to the
 * callback but bench-side filters by inner header.
 */
static esp_err_t flush_dma_locked(wan_comm_handle_t handle) {
    if (handle->dma_tx.used == 0) {
        return ESP_OK;
    }
    size_t used = handle->dma_tx.used;

    spi_transaction_t trans = {0};
    trans.length    = used * 8;
    trans.rxlength  = used * 8;
    trans.tx_buffer = handle->dma_tx.buffer;
    trans.rx_buffer = handle->flush_rx_buffer;  /* P3.d: full-duplex capture */

    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);

    /* Reset accumulator regardless of outcome — leaving stale frames queued
     * after an error would replay them next flush. */
    handle->dma_tx.used = 0;
    handle->dma_tx.frame_count = 0;

    if (ret == ESP_OK) {
        handle->dma_flushes++;
        /* P3.d: walk the captured RX buffer for any slave→master frames.
         * Updates last_acked_seq via wan_comm_rx_stream_cb; the registered
         * user callback (if any) gets fired per frame. */
        spi_frame_parse_stream(handle->flush_rx_buffer, used,
                               wan_comm_rx_stream_cb, handle,
                               &handle->frame_stats);
    } else {
        ESP_LOGE(TAG, "framed transmit failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Build a SPI frame and APPEND it to the dma_tx accumulator. Flushes
 *        first if the new frame wouldn't fit, then flushes again at the end
 *        if WAN_COMM_BATCH_MAX_FRAMES is reached. Caller must hold
 *        transfer_mutex.
 *
 *        With batching, a single spi_device_transmit() carries up to N
 *        back-to-back framed payloads — slave's parser already walks them
 *        via spi_frame_find()'s SOF-hunt. Amortises the per-transaction
 *        mutex+DMA+ISR cost (~250 µs) across N frames.
 */
static esp_err_t transmit_framed_locked(wan_comm_handle_t handle,
                                        uint16_t inner_hdr2,
                                        const uint8_t *inner_payload,
                                        uint16_t inner_payload_len) {
    size_t inner_len = (size_t)inner_payload_len + (inner_hdr2 != 0 ? 2u : 0u);
    if (inner_len > SPI_FRAME_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t frame_size = inner_len + SPI_FRAME_OVERHEAD;
    /* round each frame up to 4-byte alignment so subsequent frames start at
     * a DMA-friendly offset within dma_tx.buffer.                            */
    size_t aligned = (frame_size + 3u) & ~((size_t)3u);
    if (aligned > WAN_COMM_DMA_BUFFER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* If next frame would overflow the accumulator, flush first. */
    if (handle->dma_tx.used + aligned > WAN_COMM_DMA_BUFFER_SIZE) {
        esp_err_t fret = flush_dma_locked(handle);
        if (fret != ESP_OK) {
            return fret;
        }
    }

    uint8_t *buf = handle->dma_tx.buffer + handle->dma_tx.used;
    /* SOF / TYPE / SEQ / ACK_FOR / LEN / HDR_CRC (per spi_framing.h v2) */
    buf[0] = SPI_FRAME_SOF_LO;
    buf[1] = SPI_FRAME_SOF_HI;
    buf[2] = (uint8_t)SPI_FT_USER_BLOB;
    buf[3] = handle->tx_seq++;
    /* Master doesn't ack the slave today, so ack_for = NONE (0xFFFF). */
    buf[4] = (uint8_t)(SPI_FRAME_ACK_NONE & 0xFFu);
    buf[5] = (uint8_t)((SPI_FRAME_ACK_NONE >> 8) & 0xFFu);
    buf[6] = (uint8_t)(inner_len & 0xFFu);
    buf[7] = (uint8_t)((inner_len >> 8) & 0xFFu);
    buf[8] = spi_frame_crc8(buf, 8);

    /* Inner header + payload (payload starts at offset 9 = SPI_FRAME_HDR_SIZE) */
    uint8_t *p = &buf[SPI_FRAME_HDR_SIZE];
    if (inner_hdr2 != 0) {
        *p++ = (uint8_t)((inner_hdr2 >> 8) & 0xFFu);
        *p++ = (uint8_t)(inner_hdr2 & 0xFFu);
    }
    if (inner_payload_len > 0 && inner_payload) {
        memcpy(p, inner_payload, inner_payload_len);
    }

    /* CRC16 over bytes 2..8+inner_len (type..end of inner payload) */
    uint16_t crc = spi_frame_crc16(&buf[2], 7u + inner_len);
    buf[SPI_FRAME_HDR_SIZE + inner_len]      = (uint8_t)(crc & 0xFFu);
    buf[SPI_FRAME_HDR_SIZE + inner_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    /* Zero-pad to alignment so the next frame (if any) starts deterministic.
     * Slave parser skips these as resync bytes — harmless.                   */
    if (aligned > frame_size) {
        memset(&buf[frame_size], 0, aligned - frame_size);
    }

    handle->dma_tx.used        += aligned;
    handle->dma_tx.frame_count += 1;

    /* Auto-flush at batch threshold. */
    if (handle->dma_tx.frame_count >= WAN_COMM_BATCH_MAX_FRAMES) {
        return flush_dma_locked(handle);
    }
    return ESP_OK;
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
