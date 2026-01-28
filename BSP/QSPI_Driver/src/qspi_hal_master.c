// BSP/QSPI_Driver/src/qspi_hal_master.c

#include "qspi_hal.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "QSPI_MASTER";

// Context
static spi_device_handle_t g_spi_device = NULL;
static qspi_hal_config_t g_config;
static qspi_hal_stats_t g_stats;
static QueueHandle_t g_tx_queue;
static TaskHandle_t g_stream_task;
static volatile bool g_running = false;

// Dual-buffer ping-pong
static uint8_t g_tx_buf[QSPI_HAL_NUM_BUFFERS][QSPI_HAL_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t g_rx_buf[QSPI_HAL_NUM_BUFFERS][QSPI_HAL_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t g_active_buf = 0;

// RX callback
static qspi_hal_rx_callback_t g_rx_callback = NULL;
static void *g_rx_callback_arg = NULL;

typedef struct {
    const uint8_t *data;
    size_t len;
} tx_item_t;

// Forward declarations
static void qspi_stream_task(void *arg);
static esp_err_t qspi_master_transaction(const uint8_t *tx_data, size_t tx_len,
                                          uint8_t *rx_data, size_t rx_len);

esp_err_t qspi_hal_init(const qspi_hal_config_t *cfg) {
    if (!cfg || !cfg->is_master) return ESP_ERR_INVALID_ARG;
    
    memcpy(&g_config, cfg, sizeof(qspi_hal_config_t));
    memset(&g_stats, 0, sizeof(qspi_hal_stats_t));
    
    // Configure bus
    spi_bus_config_t bus_cfg = {
        .data0_io_num = cfg->pins.gpio_d0,
        .data1_io_num = cfg->pins.gpio_d1,
        .data2_io_num = cfg->pins.gpio_d2,
        .data3_io_num = cfg->pins.gpio_d3,
        .sclk_io_num = cfg->pins.gpio_clk,
        .max_transfer_sz = QSPI_HAL_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(cfg->host, &bus_cfg, SPI_DMA_CH_AUTO));
    
    // Configure device
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 8,
        .address_bits = 0,
        .dummy_bits = QSPI_HAL_DUMMY_BITS,
        .mode = 0,
        .clock_speed_hz = cfg->freq_mhz * 1000000,
        .spics_io_num = cfg->pins.gpio_cs,
        .queue_size = QSPI_HAL_NUM_BUFFERS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2
    };
    
    ESP_ERROR_CHECK(spi_bus_add_device(cfg->host, &dev_cfg, &g_spi_device));
    
    // Configure DR GPIO (output for master)
    gpio_config_t io_cfg = {
        .pin_bit_mask = BIT64(cfg->pins.gpio_dr),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_cfg);
    gpio_set_level(cfg->pins.gpio_dr, 0);
    
    // Create TX queue
    g_tx_queue = xQueueCreate(32, sizeof(tx_item_t));
    
    // Start streaming task
    g_running = true;
    xTaskCreate(qspi_stream_task, "qspi_stream", 4096, NULL, 10, &g_stream_task);
    
    ESP_LOGI(TAG, "Initialized @ %lu MHz", cfg->freq_mhz);
    return ESP_OK;
}

esp_err_t qspi_hal_deinit(void) {
    g_running = false;
    if (g_stream_task) {
        vTaskDelete(g_stream_task);
        g_stream_task = NULL;
    }
    if (g_tx_queue) {
        vQueueDelete(g_tx_queue);
        g_tx_queue = NULL;
    }
    if (g_spi_device) {
        spi_bus_remove_device(g_spi_device);
        g_spi_device = NULL;
    }
    spi_bus_free(g_config.host);
    return ESP_OK;
}

esp_err_t qspi_hal_stream_write(const uint8_t *data, size_t len, uint32_t timeout_ms) {
    tx_item_t item = {.data = data, .len = len};
    return (xQueueSend(g_tx_queue, &item, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) 
           ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t qspi_hal_register_rx_callback(qspi_hal_rx_callback_t callback, void *arg) {
    g_rx_callback = callback;
    g_rx_callback_arg = arg;
    return ESP_OK;
}

esp_err_t qspi_hal_assert_data_ready(void) {
    gpio_set_level(g_config.pins.gpio_dr, 1);
    return ESP_OK;
}

esp_err_t qspi_hal_deassert_data_ready(void) {
    gpio_set_level(g_config.pins.gpio_dr, 0);
    return ESP_OK;
}

esp_err_t qspi_hal_get_stats(qspi_hal_stats_t *stats) {
    if (!stats) return ESP_ERR_INVALID_ARG;
    memcpy(stats, &g_stats, sizeof(qspi_hal_stats_t));
    return ESP_OK;
}

// Internal functions

static esp_err_t qspi_master_transaction(const uint8_t *tx_data, size_t tx_len,
                                          uint8_t *rx_data, size_t rx_len) {
    spi_transaction_ext_t trans = {
        .base = {
            .cmd = (rx_data != NULL) ? QSPI_HAL_CMD_READ : QSPI_HAL_CMD_WRITE,
            .flags = SPI_TRANS_MODE_QIO,    // Quad I/O for data phase
            .length = tx_len * 8,
            .rxlength = rx_len * 8,
            .tx_buffer = tx_data,
            .rx_buffer = rx_data
        }
    };
    
    esp_err_t ret = spi_device_transmit(g_spi_device, (spi_transaction_t*)&trans);
    if (ret == ESP_OK) {
        if (tx_data) g_stats.bytes_tx += tx_len;
        if (rx_data) g_stats.bytes_rx += rx_len;
    } else {
        g_stats.dma_errors++;
    }
    return ret;
}

static void qspi_stream_task(void *arg) {
    tx_item_t tx_item;
    
    while (g_running) {
        // Check TX queue
        if (xQueueReceive(g_tx_queue, &tx_item, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint8_t buf_idx = g_active_buf;
            size_t chunk_len = (tx_item.len > QSPI_HAL_BUFFER_SIZE) 
                               ? QSPI_HAL_BUFFER_SIZE : tx_item.len;
            
            memcpy(g_tx_buf[buf_idx], tx_item.data, chunk_len);
            
            // Assert DR before TX
            qspi_hal_assert_data_ready();
            
            // Execute WRITE transaction
            qspi_master_transaction(g_tx_buf[buf_idx], chunk_len, NULL, 0);
            
            // Deassert DR after TX
            qspi_hal_deassert_data_ready();
            
            // Toggle buffer
            g_active_buf = (g_active_buf + 1) % QSPI_HAL_NUM_BUFFERS;
        }
        
        // Continuous RX polling (READ from slave)
        uint8_t buf_idx = g_active_buf;
        esp_err_t ret = qspi_master_transaction(NULL, 0, 
                                                 g_rx_buf[buf_idx], 
                                                 QSPI_HAL_BUFFER_SIZE);
        
        if (ret == ESP_OK && g_rx_callback) {
            g_rx_callback(g_rx_buf[buf_idx], QSPI_HAL_BUFFER_SIZE, g_rx_callback_arg);
        }
    }
    
    vTaskDelete(NULL);
}
