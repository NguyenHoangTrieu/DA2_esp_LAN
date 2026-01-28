// BSP/QSPI_Driver/src/qspi_hal_slave.c

#include "qspi_hal.h"
#include "driver/spi_slave_hd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "QSPI_SLAVE";

// Context
static spi_host_device_t g_host;
static qspi_hal_config_t g_config;
static qspi_hal_stats_t g_stats;
static qspi_hal_rx_callback_t g_rx_callback = NULL;
static void *g_rx_callback_arg = NULL;

// Dual-buffer ping-pong
static uint8_t g_tx_buf[QSPI_HAL_NUM_BUFFERS][QSPI_HAL_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t g_rx_buf[QSPI_HAL_NUM_BUFFERS][QSPI_HAL_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t g_active_tx_buf = 0;

static TaskHandle_t g_rx_task;
static QueueHandle_t g_tx_queue;
static volatile bool g_running = false;

typedef struct {
    const uint8_t *data;
    size_t len;
} tx_item_t;

// Forward declarations
static void qspi_slave_rx_task(void *arg);
static void qspi_slave_tx_task(void *arg);

esp_err_t qspi_hal_init(const qspi_hal_config_t *cfg) {
    if (!cfg || cfg->is_master) return ESP_ERR_INVALID_ARG;
    
    memcpy(&g_config, cfg, sizeof(qspi_hal_config_t));
    memset(&g_stats, 0, sizeof(qspi_hal_stats_t));
    g_host = cfg->host;
    
    // Configure bus
    spi_bus_config_t bus_cfg = {
        .data0_io_num = cfg->pins.gpio_d0,
        .data1_io_num = cfg->pins.gpio_d1,
        .data2_io_num = cfg->pins.gpio_d2,
        .data3_io_num = cfg->pins.gpio_d3,
        .sclk_io_num = cfg->pins.gpio_clk,
        .max_transfer_sz = QSPI_HAL_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SLAVE | SPICOMMON_BUSFLAG_GPIO_PINS
    };
    
    // Configure slave HD
    spi_slave_hd_slot_config_t slave_cfg = {
        .spics_io_num = cfg->pins.gpio_cs,
        .mode = 0,
        .command_bits = 8,
        .address_bits = 0,
        .dummy_bits = QSPI_HAL_DUMMY_BITS,
        .queue_size = QSPI_HAL_NUM_BUFFERS * 4,
        .dma_chan = SPI_DMA_CH_AUTO,
        .flags = 0
    };
    
    ESP_ERROR_CHECK(spi_slave_hd_init(g_host, &bus_cfg, &slave_cfg));
    
    // Pre-queue RX transactions
    for (int i = 0; i < QSPI_HAL_NUM_BUFFERS; i++) {
        spi_slave_hd_data_t trans = {
            .data = g_rx_buf[i],
            .len = QSPI_HAL_BUFFER_SIZE
        };
        spi_slave_hd_queue_trans(g_host, SPI_SLAVE_CHAN_RX, &trans, 0);
    }
    
    // Create TX queue
    g_tx_queue = xQueueCreate(32, sizeof(tx_item_t));
    
    // Start tasks
    g_running = true;
    xTaskCreate(qspi_slave_rx_task, "qspi_rx", 4096, NULL, 10, &g_rx_task);
    xTaskCreate(qspi_slave_tx_task, "qspi_tx", 4096, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "Initialized (Slave HD)");
    return ESP_OK;
}

esp_err_t qspi_hal_deinit(void) {
    g_running = false;
    if (g_rx_task) {
        vTaskDelete(g_rx_task);
        g_rx_task = NULL;
    }
    if (g_tx_queue) {
        vQueueDelete(g_tx_queue);
        g_tx_queue = NULL;
    }
    spi_slave_hd_deinit(g_host);
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

esp_err_t qspi_hal_get_stats(qspi_hal_stats_t *stats) {
    if (!stats) return ESP_ERR_INVALID_ARG;
    memcpy(stats, &g_stats, sizeof(qspi_hal_stats_t));
    return ESP_OK;
}

// Internal functions

static void qspi_slave_rx_task(void *arg) {
    spi_slave_hd_data_t *ret_trans;
    
    while (g_running) {
        // Wait for RX transaction completion
        esp_err_t ret = spi_slave_hd_get_trans_res(g_host, SPI_SLAVE_CHAN_RX,
                                                     &ret_trans, pdMS_TO_TICKS(100));
        
        if (ret == ESP_OK && ret_trans->trans_len > 0) {
            g_stats.bytes_rx += ret_trans->trans_len;
            
            // Invoke callback
            if (g_rx_callback) {
                g_rx_callback(ret_trans->data, ret_trans->trans_len, g_rx_callback_arg);
            }
            
            // Re-queue RX buffer
            spi_slave_hd_queue_trans(g_host, SPI_SLAVE_CHAN_RX, ret_trans, 0);
        }
    }
    
    vTaskDelete(NULL);
}

static void qspi_slave_tx_task(void *arg) {
    tx_item_t tx_item;
    
    while (g_running) {
        if (xQueueReceive(g_tx_queue, &tx_item, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint8_t buf_idx = g_active_tx_buf;
            size_t chunk_len = (tx_item.len > QSPI_HAL_BUFFER_SIZE) 
                               ? QSPI_HAL_BUFFER_SIZE : tx_item.len;
            
            memcpy(g_tx_buf[buf_idx], tx_item.data, chunk_len);
            
            spi_slave_hd_data_t trans = {
                .data = g_tx_buf[buf_idx],
                .len = chunk_len
            };
            
            esp_err_t ret = spi_slave_hd_queue_trans(g_host, SPI_SLAVE_CHAN_TX, 
                                                      &trans, pdMS_TO_TICKS(100));
            
            if (ret == ESP_OK) {
                g_stats.bytes_tx += chunk_len;
                g_active_tx_buf = (g_active_tx_buf + 1) % QSPI_HAL_NUM_BUFFERS;
            } else {
                g_stats.dma_errors++;
            }
        }
    }
    
    vTaskDelete(NULL);
}
