/**
 * @file mcu_wan_handler.c
 * @brief MCU WAN Communication Handler Implementation
 */

#include "mcu_wan_handler.h"
#include "wan_comm.h"
#include "config_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "mcu_wan_handler";

// WAN communication handle (private to this module)
static wan_comm_handle_t g_wan_comm_handle = NULL;

// Handler state
static bool mcu_wan_handler_running = false;

// Initialization flag - ensures init happens ONLY ONCE in lifetime
static bool g_initialized = false;

// Configuration for SPI pins (adjust according to your hardware)
#define MCU_WAN_SPI_SCK         GPIO_NUM_9
#define MCU_WAN_SPI_CS          GPIO_NUM_10
#define MCU_WAN_SPI_MOSI        GPIO_NUM_11
#define MCU_WAN_SPI_MISO        GPIO_NUM_12
#define MCU_WAN_SPI_WP          GPIO_NUM_13  // For Quad mode
#define MCU_WAN_SPI_HD          GPIO_NUM_14  // For Quad mode

/**
 * @brief Command received callback (from LAN MCU)
 * 
 * This callback is called by wan_comm when LAN MCU sends a COMMAND packet
 * Runs in the context of wan_comm processing task
 */
static void mcu_wan_on_command_received(uint8_t* cmd_payload, uint16_t length, void* user_data) {
    ESP_LOGI(TAG, "Command received from LAN MCU: %d bytes", length);
    ESP_LOG_BUFFER_HEXDUMP(TAG, cmd_payload, length, ESP_LOG_INFO);
    
    // Parse command type using config_handler's parser
    config_type_t cmd_type = config_parse_type((char*)cmd_payload, length);
    
    if (cmd_type != CONFIG_TYPE_UNKNOWN) {
        // Create config command structure
        config_command_t config_cmd;
        config_cmd.type = cmd_type;
        config_cmd.data_len = (length < CONFIG_CMD_MAX_LEN) ? length : CONFIG_CMD_MAX_LEN;
        memcpy(config_cmd.raw_data, cmd_payload, config_cmd.data_len);
        
        // Send to config handler queue
        if (g_config_handler_queue) {
            if (xQueueSend(g_config_handler_queue, &config_cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
                ESP_LOGI(TAG, "Command forwarded to config handler queue");
                
                // Prepare acknowledgment response for next master read
                uint8_t ack[] = "CMD_ACK";
                wan_comm_load_tx_data(g_wan_comm_handle, ack, strlen((char*)ack));
            } else {
                ESP_LOGW(TAG, "Failed to send command to config queue (queue full)");
                
                // Prepare error response
                uint8_t nack[] = "CMD_NACK_QUEUE_FULL";
                wan_comm_load_tx_data(g_wan_comm_handle, nack, strlen((char*)nack));
            }
        } else {
            ESP_LOGE(TAG, "Config handler queue not initialized");
            
            // Prepare error response
            uint8_t error[] = "CMD_ERROR_NO_QUEUE";
            wan_comm_load_tx_data(g_wan_comm_handle, error, strlen((char*)error));
        }
    } else {
        ESP_LOGW(TAG, "Unknown command type received from LAN MCU");
        
        // Prepare error response
        uint8_t unknown[] = "CMD_UNKNOWN";
        wan_comm_load_tx_data(g_wan_comm_handle, unknown, strlen((char*)unknown));
    }
}

/**
 * @brief Data received callback (from LAN MCU)
 * 
 * This callback is called by wan_comm when LAN MCU sends a DATA packet
 * Runs in the context of wan_comm processing task
 */
static void mcu_wan_on_data_received(uint8_t* data_payload, uint16_t length, void* user_data) {
    ESP_LOGI(TAG, "Data received from LAN MCU: %d bytes", length);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data_payload, length, ESP_LOG_DEBUG);
    
    // Prepare acknowledgment
    uint8_t ack[] = "DATA_ACK";
    wan_comm_load_tx_data(g_wan_comm_handle, ack, strlen((char*)ack));
}

/**
 * @brief Error callback
 */
static void mcu_wan_error_cb(wan_comm_status_t error, const char* context, void* user_data) {
    ESP_LOGE(TAG, "WAN comm error: %d - %s", error, context);
}

/**
 * @brief Initialize MCU WAN handler (STATIC - called only once in lifetime)
 * 
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t mcu_wan_handler_init(void) {
    // Check if already initialized (init only once in lifetime)
    if (g_initialized) {
        ESP_LOGI(TAG, "MCU WAN handler already initialized (lifetime init)");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing MCU WAN handler (first and only time)");
    
    // Configure WAN communication (Slave mode)
    wan_comm_config_t wan_config = {
        .gpio_sck = MCU_WAN_SPI_SCK,
        .gpio_cs = MCU_WAN_SPI_CS,
        .gpio_io0 = MCU_WAN_SPI_MOSI,
        .gpio_io1 = MCU_WAN_SPI_MISO,
        .gpio_io2 = MCU_WAN_SPI_WP,
        .gpio_io3 = MCU_WAN_SPI_HD,
        
        .mode = 0,  // SPI Mode 0
        .host_id = SPI2_HOST,
        
        .dma_channel = SPI_DMA_CH_AUTO,
        
        .rx_buffer_size = 4096,
        .tx_buffer_size = 4096,
        
        // Register callbacks - these handle incoming commands from LAN MCU
        .on_command_received = mcu_wan_on_command_received,
        .on_data_received = mcu_wan_on_data_received,
        .error_callback = mcu_wan_error_cb,
        .user_data = NULL,
        
        .enable_quad_mode = false  // Set true for QSPI if needed
    };
    
    // Initialize WAN communication library
    wan_comm_status_t status = wan_comm_init(&wan_config, &g_wan_comm_handle);
    if (status != WAN_COMM_OK) {
        ESP_LOGE(TAG, "Failed to initialize WAN communication: %d", status);
        return ESP_FAIL;
    }
    
    // Mark as initialized (will never initialize again)
    g_initialized = true;
    
    ESP_LOGI(TAG, "MCU WAN handler initialized successfully (lifetime init complete)");
    ESP_LOGI(TAG, "Waiting for commands from LAN MCU...");
    
    return ESP_OK;
}

/**
 * @brief Start MCU WAN handler
 */
esp_err_t mcu_wan_handler_start(void) {
    // Initialize if not already done (init only once)
    if (!g_initialized) {
        esp_err_t ret = mcu_wan_handler_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize MCU WAN handler");
            return ret;
        }
    }
    
    // Check if already running
    if (mcu_wan_handler_running) {
        ESP_LOGW(TAG, "MCU WAN handler already running");
        return ESP_OK;
    }
    
    // Check if initialized
    if (g_wan_comm_handle == NULL) {
        ESP_LOGE(TAG, "MCU WAN handler not initialized (internal error)");
        return ESP_FAIL;
    }
    
    mcu_wan_handler_running = true;
    
    ESP_LOGI(TAG, "MCU WAN handler started - listening for commands from LAN MCU");
    
    return ESP_OK;
}

/**
 * @brief Stop MCU WAN handler
 * 
 */
esp_err_t mcu_wan_handler_stop(void) {
    if (!mcu_wan_handler_running) {
        ESP_LOGD(TAG, "MCU WAN handler not running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping MCU WAN handler");
    
    mcu_wan_handler_running = false;
    
    // Stop config handler
    config_handler_task_stop();
    
    ESP_LOGI(TAG, "MCU WAN handler stopped");
    
    return ESP_OK;
}
