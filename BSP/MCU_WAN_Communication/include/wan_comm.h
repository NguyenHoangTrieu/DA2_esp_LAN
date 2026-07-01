#ifndef WAN_COMM_H
#define WAN_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "frame_types.h"
#include "spi_framing.h"               /* for spi_frame_view_t */
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// CONFIGURATION - Timing Parameters

#define WAN_COMM_SPI_CLOCK_HZ         40000000  // 40 MHz default
#define WAN_COMM_DEFAULT_TX_BUFFER    16384     // 16KB per design (legacy)
#define WAN_COMM_DEFAULT_RX_BUFFER    16384     // 16KB per design
#define WAN_COMM_DMA_BUFFER_SIZE      16384     // 16KB – large enough for max config JSON
#define WAN_COMM_TRANS_QUEUE_SIZE     7
#define WAN_COMM_ACK_TIMEOUT_MS       200
#define WAN_COMM_DQ_RETRY_MS          50
#define WAN_COMM_DQ_RETRY_COUNT       10
#define WAN_COMM_TIMEOUT_MS           1000
#define WAN_COMM_MAX_TRANSFER_SIZE    16384
#define WAN_COMM_FIXED_XFER_LEN       (INTER_MCU_PAYLOAD_MAX_LEN + WAN_COMM_HEADER_SIZE)
#define WAN_COMM_DMA_DESCRIPTOR_SIZE  4092      // ESP32 max per descriptor
#define WAN_COMM_MAX_DMA_DESCRIPTORS  8

// Frame headers
#define WAN_COMM_HEADER_CF            0x4346    // Command Frame
#define WAN_COMM_HEADER_DT            0x4454    // Data Transfer
#define WAN_COMM_HEADER_DQ            0x4451    // Data Query
#define WAN_COMM_HEADER_SIZE          2

// DMA alignment
#define DMA_ALIGNMENT                 4
#define DMA_ALIGN_SIZE(x)             (((x) + (DMA_ALIGNMENT - 1)) & ~(DMA_ALIGNMENT - 1))

// ERROR CODES

typedef enum {
    WAN_COMM_OK = 0,
    WAN_COMM_ERR_INVALID_ARG,
    WAN_COMM_ERR_NOT_INITIALIZED,
    WAN_COMM_ERR_NOMEM,
    WAN_COMM_ERR_TIMEOUT,
    WAN_COMM_ERR_BUS_BUSY,
    WAN_COMM_ERR_INVALID_STATE,
    WAN_COMM_ERR_DMA_ALIGN
} wan_comm_status_t;

// STRUCTURES

/**
 * @brief SPI Master Configuration
 */
typedef struct {
    // GPIO pins
    int gpio_sck;
    int gpio_cs;
    int gpio_io0;
    int gpio_io1;
    int gpio_data_ready_input; // GPIO46 for ISR, -1 to disable
    
    // SPI settings
    uint32_t clock_speed_hz;   // 40 MHz recommended
    uint8_t mode;              // SPI mode 0-3
    spi_host_device_t host_id; // SPI2_HOST or SPI3_HOST
    int dma_channel;           // SPI_DMA_CH_AUTO recommended
    
    // Buffer sizes (legacy, replaced by DMA buffer internally)
    size_t tx_buffer_size;
    size_t rx_buffer_size;
    
    // Queue
    int queue_size;
} wan_comm_config_t;

/**
 * @brief Default configuration macro
 */
#define WAN_COMM_CONFIG_DEFAULT() { \
    .gpio_sck = 12, \
    .gpio_cs = 10, \
    .gpio_io0 = 11, \
    .gpio_io1 = 13, \
    .gpio_data_ready_input = 46, \
    .clock_speed_hz = WAN_COMM_SPI_CLOCK_HZ, \
    .mode = 0, \
    .host_id = SPI2_HOST, \
    .dma_channel = SPI_DMA_CH_AUTO, \
    .tx_buffer_size = WAN_COMM_DEFAULT_TX_BUFFER, \
    .rx_buffer_size = WAN_COMM_DEFAULT_RX_BUFFER, \
    .queue_size = WAN_COMM_TRANS_QUEUE_SIZE \
}

/**
 * @brief Opaque handle
 */
typedef struct wan_comm_handle_s *wan_comm_handle_t;

/**
 * @brief Data-ready ISR callback
 */
typedef void (*wan_comm_data_ready_callback_t)(void *user_arg);

// PUBLIC API

/**
 * @brief Initialize SPI Master with DMA buffering
 * 
 * @param config Configuration structure
 * @param[out] handle Output handle
 * @return WAN_COMM_OK on success
 */
wan_comm_status_t wan_comm_init(const wan_comm_config_t *config, wan_comm_handle_t *handle);

/**
 * @brief Deinitialize SPI Master
 */
wan_comm_status_t wan_comm_deinit(wan_comm_handle_t handle);

/**
 * @brief Send command frame (CF) - uses DMA buffering
 * 
 * @param handle Handle
 * @param command_payload Payload without header
 * @param length Payload length
 * @return WAN_COMM_OK on success
 */
wan_comm_status_t wan_comm_send_command(wan_comm_handle_t handle, 
                                         const uint8_t *command_payload, 
                                         uint16_t length);

/**
 * @brief Send data frame (DT) - uses DMA buffering
 * 
 * @param handle Handle
 * @param data_payload Payload without header
 * @param length Payload length
 * @return WAN_COMM_OK on success
 */
wan_comm_status_t wan_comm_send_data(wan_comm_handle_t handle, 
                                      const uint8_t *data_payload, 
                                      uint16_t length);

/**
 * @brief Request data from slave (DQ)
 * 
 * @param handle Handle
 * @param rx_buffer Buffer to store received data
 * @param length_to_read Number of bytes to read
 * @return WAN_COMM_OK on success
 */
wan_comm_status_t wan_comm_request_data(wan_comm_handle_t handle, 
                                         uint8_t *rx_buffer, 
                                         uint16_t length_to_read);

/**
 * @brief Full-duplex transceive
 */
wan_comm_status_t wan_comm_transceive(wan_comm_handle_t handle, 
                                       const uint8_t *tx_data, 
                                       uint16_t tx_length,
                                       uint8_t *rx_buffer, 
                                       uint16_t rx_length);

/**
 * @brief Flush DMA TX buffer immediately
 * Forces transmission of accumulated frames with 0x00 padding
 * 
 * @param handle Handle
 * @return WAN_COMM_OK on success
 */
wan_comm_status_t wan_comm_flush_dma_buffer(wan_comm_handle_t handle);

/**
 * @brief Register data-ready ISR callback
 */
wan_comm_status_t wan_comm_register_data_ready_callback(wan_comm_handle_t handle, 
                                                         wan_comm_data_ready_callback_t callback,
                                                         void *user_arg);

/**
 * @brief Get last error code
 */
wan_comm_status_t wan_comm_get_last_error(wan_comm_handle_t handle);

/**
 * @brief Get statistics
 */
wan_comm_status_t wan_comm_get_statistics(wan_comm_handle_t handle,
                                           uint32_t *packets_sent,
                                           uint32_t *errors);

/**
 * @brief Clear error counter
 */
wan_comm_status_t wan_comm_clear_error_count(wan_comm_handle_t handle);

/**
 * @brief P1-framing stats snapshot.
 * @return WAN_COMM_OK on success; fields not requested may be NULL.
 */
wan_comm_status_t wan_comm_get_framing_stats(wan_comm_handle_t handle,
                                              uint32_t *rx_frames_ok,
                                              uint32_t *rx_hdr_crc_fail,
                                              uint32_t *rx_payload_crc_fail,
                                              uint32_t *rx_resync_bytes,
                                              uint32_t *rx_seq_gap);

/* Cumulative-ACK API. Slave piggybacks its max-master-seq-seen via the
 * framing ACK_FOR field; recorded in handle->last_acked_seq. */

/**
 * @brief Send a data (DT) frame and report the seq number used on the wire.
 * Same semantics as wan_comm_send_data, plus exposes the framing seq so the
 * caller can later poll wan_comm_was_seq_acked() on it.
 *
 * @param[out] out_seq seq byte assigned to this frame (rolling 0..255)
 * @return WAN_COMM_OK on success
 */
wan_comm_status_t wan_comm_send_data_get_seq(wan_comm_handle_t handle,
                                              const uint8_t *data_payload,
                                              uint16_t length,
                                              uint8_t *out_seq);

/**
 * @brief Return the highest master-seq the slave has acknowledged via
 *        piggyback ACK_FOR, or SPI_FRAME_ACK_NONE (0xFFFF) if none yet.
 */
uint16_t wan_comm_get_last_acked_seq(wan_comm_handle_t handle);

/**
 * @brief True if @seq is covered by the slave's cumulative ACK.
 * Uses signed 8-bit modular comparison so wraparound works correctly while
 * the outstanding window stays under 128 frames.
 */
bool wan_comm_was_seq_acked(wan_comm_handle_t handle, uint8_t seq);

/* Full-duplex RX dispatch. Each master flush clocks slave tx_buffer back
 * into a scratch RX buffer; the parser fires the registered callback per
 * frame and updates handle->last_acked_seq. */

/**
 * @brief Callback fired once per slave-to-master frame parsed out of the
 *        full-duplex flush RX buffer.
 *
 * Called from the context of wan_comm_flush_dma_buffer (or any internal
 * flush triggered by send_data or request_data), with transfer_mutex held.
 * The callback must not call back into wan_comm or it will deadlock.
 *
 * The view pointer is invalidated when the callback returns - copy what you
 * need before returning.
 */
typedef void (*wan_comm_rx_frame_cb_t)(const spi_frame_view_t *view, void *user);

/**
 * @brief Register (or unregister with cb=NULL) the RX frame callback.
 *        Replaces any previously registered callback.
 */
wan_comm_status_t wan_comm_register_rx_frame_callback(wan_comm_handle_t handle,
                                                       wan_comm_rx_frame_cb_t cb,
                                                       void *user);

#ifdef __cplusplus
}
#endif

#endif // WAN_COMM_H
