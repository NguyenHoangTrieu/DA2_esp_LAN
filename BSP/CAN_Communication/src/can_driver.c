/**
 * @file can_driver.c
 * @brief ESP32-S3 TWAI CAN Driver Implementation (ESP-IDF v6.0)
 * @note Uses new esp_twai and esp_twai_onchip API
 */

#include "can_driver.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "CAN_DRV";
static bool driver_initialized = false;
static twai_node_handle_t twai_handle = NULL;
static QueueHandle_t rx_queue = NULL;

/* Minimal timeout for non-blocking operation (ms) */
#define CAN_TX_TIMEOUT_MS 10
#define CAN_RX_TIMEOUT_MS 0 // Immediate return (0 ms)
#define RX_QUEUE_SIZE 10

/**
 * @brief RX callback for receiving messages
 * @note Called from ISR context when message is received
 */
static bool IRAM_ATTR
twai_rx_done_callback(twai_node_handle_t handle,
                      const twai_rx_done_event_data_t *edata, void *user_ctx) {
  BaseType_t high_task_woken = pdFALSE;

  // Prepare buffer for receiving
  uint8_t recv_buff[8];
  twai_frame_t rx_frame = {
      .buffer = recv_buff,
      .buffer_len = sizeof(recv_buff),
  };

  // Receive message from ISR
  if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
    // Send frame to queue for processing
    if (rx_queue != NULL) {
      xQueueSendFromISR(rx_queue, &rx_frame, &high_task_woken);
    }
  }

  return high_task_woken == pdTRUE;
}

/**
 * @brief Configure hardware acceptance filter based on whitelist
 * @note REQ-INI-002, REQ-INI-003
 */
static esp_err_t configure_acceptance_filter(void) {
  if (g_whitelist_count == 0) {
    // Accept all frames (REQ-INI-003)
    ESP_LOGI(TAG, "Whitelist empty - accepting all frames");
    // No filter configuration needed - accepts all by default
    return ESP_OK;
  } else if (g_whitelist_count == 1) {
    // Single ID - use hardware filter (REQ-INI-002)
    uint16_t id = g_can_whitelist[0];
    twai_mask_filter_config_t mask_cfg = {
        .id = (uint32_t)id, // Standard 11-bit ID
        .mask = 0x7FF,      // Mask all 11 bits (exact match)
        .is_ext = false,    // Standard ID (not extended)
    };

    esp_err_t ret = twai_node_config_mask_filter(twai_handle, 0, &mask_cfg);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure mask filter: %s",
               esp_err_to_name(ret));
      return ret;
    }
    ESP_LOGI(TAG, "Hardware filter: ID=0x%03X", id);
    return ESP_OK;
  } else {
    // Multiple IDs - accept all and filter in software (REQ-INI-003)
    ESP_LOGI(TAG, "Multiple whitelist IDs (%d) - using software filtering",
             g_whitelist_count);
    return ESP_OK;
  }
}

/**
 * @brief Initialize TWAI driver (REQ-INI-001)
 * @note Updated for ESP-IDF v6.0 new esp_twai API
 */
can_status_t can_driver_init(void) {
  if (driver_initialized) {
    ESP_LOGW(TAG, "Driver already initialized");
    return CAN_OK;
  }

  // Validate configuration (REQ-DAT-001)
  if (g_can_config.tx_gpio >= GPIO_NUM_MAX ||
      g_can_config.rx_gpio >= GPIO_NUM_MAX) {
    ESP_LOGE(TAG, "Invalid GPIO configuration");
    return CAN_ERR_INVALID_CONFIG;
  }

  // Create RX queue for callback
  rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(twai_frame_t));
  if (rx_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    return CAN_ERR_DRIVER_INSTALL;
  }

  // Node configuration
  twai_onchip_node_config_t node_config = {
      .io_cfg =
          {
              .tx = g_can_config.tx_gpio,
              .rx = g_can_config.rx_gpio,
              .quanta_clk_out = -1,    // Not used
              .bus_off_indicator = -1, // Not used
          },
      .bit_timing =
          {
              .bitrate = g_can_config.baud_rate,
          },
      .tx_queue_depth = 10,
      .flags =
          {
              .enable_self_test =
                  (g_can_config.operating_mode == CAN_MODE_NO_ACK) ? 1 : 0,
              .enable_listen_only =
                  (g_can_config.operating_mode == CAN_MODE_LOOPBACK) ? 1 : 0,
              .enable_loopback = 0,
              .no_receive_rtr = 0,
          },
  };

  // Create TWAI node
  esp_err_t ret = twai_new_node_onchip(&node_config, &twai_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(ret));
    vQueueDelete(rx_queue);
    rx_queue = NULL;
    return CAN_ERR_DRIVER_INSTALL;
  }

  // Register RX callback
  twai_event_callbacks_t callbacks = {
      .on_rx_done = twai_rx_done_callback,
  };
  ret = twai_node_register_event_callbacks(twai_handle, &callbacks, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
    twai_node_delete(twai_handle);
    vQueueDelete(rx_queue);
    twai_handle = NULL;
    rx_queue = NULL;
    return CAN_ERR_DRIVER_INSTALL;
  }

  // Configure acceptance filter after node creation (REQ-INI-002, REQ-INI-003)
  ret = configure_acceptance_filter();
  if (ret != ESP_OK) {
    twai_node_delete(twai_handle);
    vQueueDelete(rx_queue);
    twai_handle = NULL;
    rx_queue = NULL;
    return CAN_ERR_DRIVER_INSTALL;
  }

  // Enable TWAI node (start)
  ret = twai_node_enable(twai_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable TWAI node: %s", esp_err_to_name(ret));
    twai_node_delete(twai_handle);
    vQueueDelete(rx_queue);
    twai_handle = NULL;
    rx_queue = NULL;
    return CAN_ERR_DRIVER_START;
  }

  driver_initialized = true;
  ESP_LOGI(TAG, "CAN driver initialized: TX=%d, RX=%d, Baud=%lu, Mode=%d",
           g_can_config.tx_gpio, g_can_config.rx_gpio, g_can_config.baud_rate,
           g_can_config.operating_mode);
  return CAN_OK;
}

/**
 * @brief Transmit CAN message (REQ-TX-001, REQ-TX-002, REQ-TX-003, REQ-TX-004)
 */
can_status_t can_transmit(uint16_t id, const uint8_t *data, uint8_t len) {
  if (!driver_initialized || twai_handle == NULL) {
    return CAN_ERR_NOT_INITIALIZED;
  }

  // Validate parameters (REQ-CST-002)
  if (id > 0x7FF) { // Standard frame: 11-bit ID max
    ESP_LOGE(TAG, "Invalid ID: 0x%03X (must be <= 0x7FF)", id);
    return CAN_ERR_INVALID_PARAM;
  }

  if (len > 8) { // CAN 2.0A max DLC
    ESP_LOGE(TAG, "Invalid DLC: %d (must be 0-8)", len);
    return CAN_ERR_INVALID_PARAM;
  }

  if (data == NULL && len > 0) {
    return CAN_ERR_INVALID_PARAM;
  }

  // Construct Standard Frame (REQ-TX-002)
  twai_frame_t tx_frame = {
      .header =
          {
              .id = id,
              .ide = false, // Standard 11-bit ID
              .rtr = false, // Data frame
          },
      .buffer = (uint8_t *)data,
      .buffer_len = len,
  };

  // Non-blocking transmit (REQ-TX-004)
  esp_err_t ret = twai_node_transmit(twai_handle, &tx_frame, CAN_TX_TIMEOUT_MS);
  if (ret == ESP_OK) {
    return CAN_OK;
  } else if (ret == ESP_ERR_TIMEOUT) {
    return CAN_ERR_TX_TIMEOUT; // Buffer full
  } else {
    ESP_LOGE(TAG, "TX failed: %s", esp_err_to_name(ret));
    return CAN_ERR_TX_FAILED;
  }
}

/**
 * @brief Software ID filtering (REQ-RX-002)
 * @note REQ-CST-003 - optimized with early break
 */
static inline bool is_id_whitelisted(uint16_t id) {
  // If whitelist is empty, accept all (REQ-RX-002)
  if (g_whitelist_count == 0) {
    return true;
  }

  // Check against whitelist (optimized loop - REQ-CST-003)
  for (uint8_t i = 0; i < g_whitelist_count; i++) {
    if (g_can_whitelist[i] == id) {
      return true; // Early exit on match
    }
  }
  return false; // No match found
}

/**
 * @brief Receive CAN message with software filtering (REQ-RX-001, REQ-RX-002,
 * REQ-RX-003)
 */
can_status_t can_receive(can_message_t *msg) {
  if (!driver_initialized || twai_handle == NULL) {
    return CAN_ERR_NOT_INITIALIZED;
  }

  if (msg == NULL) {
    return CAN_ERR_INVALID_PARAM;
  }

  twai_frame_t rx_frame;

  // Non-blocking receive from queue (populated by callback)
  if (xQueueReceive(rx_queue, &rx_frame, 0) != pdTRUE) {
    return CAN_ERR_RX_NO_DATA;
  }

  // Only process Standard Frames (REQ-CST-002)
  if (rx_frame.header.ide) {
    ESP_LOGW(TAG, "Extended frame ignored");
    return CAN_ERR_RX_NO_DATA;
  }

  uint16_t rx_id = (uint16_t)(rx_frame.header.id & 0x7FF);

  // Software filtering (REQ-RX-002)
  if (!is_id_whitelisted(rx_id)) {
    // Silently discard non-whitelisted messages
    return CAN_ERR_RX_NO_DATA;
  }

  // Copy message to output structure (REQ-RX-003)
  msg->id = rx_id;
  msg->len = rx_frame.buffer_len;
  msg->rtr = rx_frame.header.rtr;
  memcpy(msg->data, rx_frame.buffer, rx_frame.buffer_len);

  return CAN_OK;
}

/**
 * @brief Check bus status (REQ-ERR-001)
 */
can_bus_state_t can_check_bus_status(void) {
  if (!driver_initialized || twai_handle == NULL) {
    return CAN_BUS_UNKNOWN;
  }

  twai_node_status_t status;
  twai_node_record_t record;

  esp_err_t ret = twai_node_get_info(twai_handle, &status, &record);
  if (ret != ESP_OK) {
    return CAN_BUS_UNKNOWN;
  }

  // Check TEC and REC to determine error state
  // Error Active: TEC < 96 and REC < 96
  // Error Warning: TEC >= 96 or REC >= 96 (but < 128)
  // Error Passive: TEC >= 128 or REC >= 128 (but < 256)
  // Bus Off: TEC >= 256

  if (status.tx_error_count >= 256) {
    return CAN_BUS_BUS_OFF;
  } else if (status.tx_error_count >= 128 || status.rx_error_count >= 128) {
    return CAN_BUS_ERROR_PASSIVE;
  } else if (status.tx_error_count >= 96 || status.rx_error_count >= 96) {
    return CAN_BUS_WARNING;
  } else {
    return CAN_BUS_RUNNING;
  }
}

/**
 * @brief Initiate bus-off recovery (REQ-ERR-002)
 */
can_status_t can_initiate_recovery(void) {
  if (!driver_initialized || twai_handle == NULL) {
    return CAN_ERR_NOT_INITIALIZED;
  }

  // Use twai_node_recover for bus-off recovery
  esp_err_t ret = twai_node_recover(twai_handle);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Bus-off recovery initiated");
    return CAN_OK;
  } else {
    ESP_LOGE(TAG, "Recovery failed: %s", esp_err_to_name(ret));
    return CAN_ERR_BUS_OFF;
  }
}

/**
 * @brief Deinitialize the CAN driver
 */
can_status_t can_driver_deinit(void) {
  if (!driver_initialized) {
    return CAN_OK;
  }

  if (twai_handle != NULL) {
    twai_node_disable(twai_handle);
    twai_node_delete(twai_handle);
    twai_handle = NULL;
  }

  if (rx_queue != NULL) {
    vQueueDelete(rx_queue);
    rx_queue = NULL;
  }

  driver_initialized = false;
  ESP_LOGI(TAG, "CAN driver deinitialized");
  return CAN_OK;
}
