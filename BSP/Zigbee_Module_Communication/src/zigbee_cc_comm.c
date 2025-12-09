/**
 * @file zigbee_cc_comm.c
 * @brief CC2530 Zigbee UART Communication Driver Implementation
 * 
 * Simple BSP driver implementation for CC2530 Zigbee module.
 * Provides transparent UART communication with the module.
 */

#include "zigbee_cc_comm.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ZIGBEE_CC_COMM";

/* ===== Internal Handle Structure ===== */
struct zigbee_cc_comm_handle_s {
    int uart_port;              /**< UART port number */
    int baud_rate;              /**< Configured baud rate */
    bool is_initialized;        /**< Initialization flag */
};

/* ===== API Implementation ===== */
/* Global handle for auto-init */
zigbee_cc_comm_handle_t g_zigbee_cc_handle = NULL;

/* Default UART config */
static zigbee_cc_uart_config_t g_default_uart_config = {
    .baud_rate = 38400,
    .rx_buffer_size = 1024,
    .tx_buffer_size = 512
};

esp_err_t zigbee_cc_comm_auto_init_default(void) {
    static bool s_initialized = false;
    
    if (s_initialized) {
        ESP_LOGI("ZIGBEE_CC_COMM", "Already initialized");
        return ESP_OK;
    }
    
    esp_err_t ret = zigbee_cc_comm_init(&g_default_uart_config, 
                                         &g_zigbee_cc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("ZIGBEE_CC_COMM", "Auto init failed: %s", 
                 esp_err_to_name(ret));
        return ret;
    }
    
    s_initialized = true;
    ESP_LOGI("ZIGBEE_CC_COMM", "Auto init successful");
    return ESP_OK;
}


esp_err_t zigbee_cc_comm_init(const zigbee_cc_uart_config_t *config,
                               zigbee_cc_comm_handle_t *handle)
{
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing CC2530 Zigbee UART driver");

    /* Allocate handle */
    zigbee_cc_comm_handle_t h = (zigbee_cc_comm_handle_t)calloc(1, 
                                  sizeof(struct zigbee_cc_comm_handle_s));
    if (h == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for handle");
        return ESP_ERR_NO_MEM;
    }

    /* Use default baud rate if not specified */
    int baud = (config->baud_rate > 0) ? config->baud_rate : ZIGBEE_CC_DEFAULT_BAUD_RATE;
    int rx_buf_size = (config->rx_buffer_size > 0) ? config->rx_buffer_size : ZIGBEE_CC_RX_BUFFER_SIZE;
    int tx_buf_size = (config->tx_buffer_size > 0) ? config->tx_buffer_size : ZIGBEE_CC_TX_BUFFER_SIZE;

    /* Configure UART parameters */
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Configure UART */
    esp_err_t ret = uart_param_config(ZIGBEE_CC_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        free(h);
        return ret;
    }

    /* Set UART pins */
    ret = uart_set_pin(ZIGBEE_CC_UART_PORT,
                       ZIGBEE_CC_UART_TX_PIN,
                       ZIGBEE_CC_UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        free(h);
        return ret;
    }

    /* Install UART driver */
    ret = uart_driver_install(ZIGBEE_CC_UART_PORT,
                             rx_buf_size,
                             tx_buf_size,
                             0,
                             NULL,
                             0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        free(h);
        return ret;
    }

    /* Initialize handle */
    h->uart_port = ZIGBEE_CC_UART_PORT;
    h->baud_rate = baud;
    h->is_initialized = true;

    *handle = h;

    ESP_LOGI(TAG, "CC2530 UART initialized: port=%d, baud=%d, TX=%d, RX=%d",
             ZIGBEE_CC_UART_PORT, baud, ZIGBEE_CC_UART_TX_PIN, ZIGBEE_CC_UART_RX_PIN);

    return ESP_OK;
}

esp_err_t zigbee_cc_comm_deinit(zigbee_cc_comm_handle_t handle)
{
    if (handle == NULL || !handle->is_initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Deinitializing CC2530 UART driver");

    /* Delete UART driver */
    esp_err_t ret = uart_driver_delete(handle->uart_port);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART driver delete warning: %s", esp_err_to_name(ret));
    }

    /* Free handle */
    handle->is_initialized = false;
    free(handle);

    ESP_LOGI(TAG, "CC2530 UART driver deinitialized");

    return ESP_OK;
}

esp_err_t zigbee_cc_comm_write(zigbee_cc_comm_handle_t handle,
                                const uint8_t *data,
                                size_t length,
                                uint32_t timeout_ms)
{
    if (handle == NULL || !handle->is_initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0) {
        ESP_LOGE(TAG, "Invalid data or length");
        return ESP_ERR_INVALID_ARG;
    }

    /* Write data to UART */
    int written = uart_write_bytes(handle->uart_port, data, length);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return ESP_FAIL;
    }

    /* Wait for transmission to complete */
    esp_err_t ret = uart_wait_tx_done(handle->uart_port, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART TX timeout");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGD(TAG, "Written %d bytes to CC2530", written);

    return ESP_OK;
}

esp_err_t zigbee_cc_comm_read(zigbee_cc_comm_handle_t handle,
                               uint8_t *buffer,
                               size_t length,
                               size_t *actual_length,
                               uint32_t timeout_ms)
{
    if (handle == NULL || !handle->is_initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (buffer == NULL || actual_length == NULL) {
        ESP_LOGE(TAG, "Invalid buffer or actual_length pointer");
        return ESP_ERR_INVALID_ARG;
    }

    /* Read data from UART */
    int len = uart_read_bytes(handle->uart_port,
                             buffer,
                             length,
                             pdMS_TO_TICKS(timeout_ms));

    if (len < 0) {
        *actual_length = 0;
        ESP_LOGE(TAG, "UART read failed");
        return ESP_FAIL;
    }

    *actual_length = (size_t)len;

    if (len > 0) {
        ESP_LOGD(TAG, "Read %d bytes from CC2530", len);
        return ESP_OK;
    } else {
        /* Timeout - no data available */
        return ESP_ERR_TIMEOUT;
    }
}

size_t zigbee_cc_comm_available(zigbee_cc_comm_handle_t handle)
{
    if (handle == NULL || !handle->is_initialized) {
        ESP_LOGW(TAG, "Invalid handle or not initialized");
        return 0;
    }

    size_t available = 0;
    esp_err_t ret = uart_get_buffered_data_len(handle->uart_port, &available);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get buffered data length");
        return 0;
    }

    return available;
}

esp_err_t zigbee_cc_comm_flush(zigbee_cc_comm_handle_t handle)
{
    if (handle == NULL || !handle->is_initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Flush UART RX buffer */
    esp_err_t ret = uart_flush_input(handle->uart_port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART flush failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "CC2530 UART RX buffer flushed");

    return ESP_OK;
}
