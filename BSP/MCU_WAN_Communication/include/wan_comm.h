/**
 * @file wan_comm.h
 * @brief WAN MCU Communication Library (QSPI Slave)
 * 
 * This library provides QSPI slave functionality for communication
 * between LAN MCU (Master) and WAN MCU (Slave) using ESP-IDF SPI Slave driver.
 * Features event-driven architecture with DMA and double buffering.
 */

#ifndef WAN_COMM_H
#define WAN_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Protocol headers
 */
#define WAN_COMM_HEADER_CF 0x4346  // "CF" - Command Frame
#define WAN_COMM_HEADER_DT 0x4454  // "DT" - Data Frame
#define WAN_COMM_HEADER_SIZE 2

/**
 * @brief Default configuration values
 */
#define WAN_COMM_DEFAULT_RX_BUFFER_SIZE 4096
#define WAN_COMM_DEFAULT_TX_BUFFER_SIZE 4096
#define WAN_COMM_PROCESSING_TASK_STACK_SIZE 4096
#define WAN_COMM_PROCESSING_TASK_PRIORITY 5
#define WAN_COMM_TRANS_QUEUE_SIZE 10

/**
 * @brief Status codes
 */
typedef enum {
    WAN_COMM_OK = 0,
    WAN_COMM_ERR_INVALID_ARG,
    WAN_COMM_ERR_TIMEOUT,
    WAN_COMM_ERR_INVALID_STATE,
    WAN_COMM_ERR_NO_MEM,
    WAN_COMM_ERR_INVALID_HEADER,
    WAN_COMM_ERR_BUS_BUSY,
    WAN_COMM_ERR_DMA_FAILURE,
    WAN_COMM_ERR_NOT_INITIALIZED,
    WAN_COMM_ERR_BUFFER_FULL
} wan_comm_status_t;

/**
 * @brief Command received callback type
 * 
 * @param cmd_payload Command payload (without header)
 * @param length Payload length
 * @param user_data User-provided context data
 */
typedef void (*wan_comm_command_cb_t)(uint8_t* cmd_payload, uint16_t length, void* user_data);

/**
 * @brief Data received callback type
 * 
 * @param data_payload Data payload (without header)
 * @param length Payload length
 * @param user_data User-provided context data
 */
typedef void (*wan_comm_data_cb_t)(uint8_t* data_payload, uint16_t length, void* user_data);

/**
 * @brief Error callback type
 * 
 * @param error Error code
 * @param context Error context string
 * @param user_data User-provided context data
 */
typedef void (*wan_comm_error_cb_t)(wan_comm_status_t error, const char* context, void* user_data);

/**
 * @brief Configuration structure
 */
typedef struct {
    // GPIO pins
    int gpio_sck;           // SPI Clock
    int gpio_cs;            // Chip Select
    int gpio_io0;           // MOSI / IO0
    int gpio_io1;           // MISO / IO1
    int gpio_io2;           // WP / IO2 (for Quad mode)
    int gpio_io3;           // HD / IO3 (for Quad mode)
    
    // SPI configuration
    uint8_t mode;               // SPI mode (0-3)
    spi_host_device_t host_id;  // SPI peripheral (SPI2_HOST or SPI3_HOST)
    
    // DMA configuration
    int dma_channel;            // DMA channel (SPI_DMA_CH_AUTO recommended)
    
    // Buffer sizes
    uint16_t rx_buffer_size;    // Size of each RX buffer (double buffering)
    uint16_t tx_buffer_size;    // Size of TX buffer
    
    // Callbacks (mandatory)
    wan_comm_command_cb_t on_command_received;  // Command received callback
    wan_comm_data_cb_t on_data_received;        // Data received callback
    wan_comm_error_cb_t error_callback;         // Error callback (optional)
    void* user_data;                            // User context for callbacks
    
    // Options
    bool enable_quad_mode;      // Enable QSPI (4-bit) mode
} wan_comm_config_t;

/**
 * @brief Handle structure (opaque)
 */
typedef struct wan_comm_handle_s* wan_comm_handle_t;

/**
 * @brief Initialize WAN communication library
 * 
 * @param config Configuration structure
 * @param handle Output handle pointer
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_init(const wan_comm_config_t* config, wan_comm_handle_t* handle);

/**
 * @brief Deinitialize WAN communication library
 * 
 * @param handle Handle to deinitialize
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_deinit(wan_comm_handle_t handle);

/**
 * @brief Load data into TX buffer for master to read
 * 
 * When master calls request_data, this buffer will be sent automatically
 * 
 * @param handle Communication handle
 * @param data_to_send Data to load into TX buffer
 * @param length Data length
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_load_tx_data(wan_comm_handle_t handle, 
                                        const uint8_t* data_to_send, 
                                        uint16_t length);

/**
 * @brief Get last error status
 * 
 * @param handle Communication handle
 * @return wan_comm_status_t Last error code
 */
wan_comm_status_t wan_comm_get_last_error(wan_comm_handle_t handle);

/**
 * @brief Get statistics
 * 
 * @param handle Communication handle
 * @param commands_received Output: number of commands received
 * @param data_packets_received Output: number of data packets received
 * @param errors Output: number of errors
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_get_statistics(wan_comm_handle_t handle,
                                          uint32_t* commands_received,
                                          uint32_t* data_packets_received,
                                          uint32_t* errors);

/**
 * @brief Clear statistics counters
 * 
 * @param handle Communication handle
 * @return wan_comm_status_t Status code
 */
wan_comm_status_t wan_comm_clear_statistics(wan_comm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // WAN_COMM_H
