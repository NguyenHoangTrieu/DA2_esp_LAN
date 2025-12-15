/**
 * @file can_driver.c
 */

#include "can_driver.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <string.h>

static const char *TAG = "CAN_DRV";

// Debug GPIO for TX callback visualization
#define DEBUG_TX_GPIO GPIO_NUM_8

// Queue configuration
#define RX_QUEUE_LEN 20
#define TX_TIMEOUT_MS 100

// Default GPIO pins
#define CAN_TX_GPIO_DEFAULT GPIO_NUM_5
#define CAN_RX_GPIO_DEFAULT GPIO_NUM_6

// Queue item with embedded buffer (pattern from ESP-IDF example)
typedef struct {
  can_message_t msg;
  uint8_t buffer[8];
} rx_queue_item_t;

// Driver context structure
typedef struct {
  twai_node_handle_t node_handle;
  QueueHandle_t rx_queue;
  atomic_bool is_initialized;
  twai_onchip_node_config_t driver_config;
  twai_event_callbacks_t callbacks;
} can_driver_ctx_t;

static can_driver_ctx_t g_can_ctx = {0};

// Global configuration (defined externally)
uint16_t g_can_whitelist[MAX_WHITELISTED_IDS] = {0};
uint16_t g_can_whitelist_count = 0;
volatile uint32_t g_counter = 0;

/**
 * @brief Error callback - logs TWAI bus errors
 */
static bool IRAM_ATTR can_error_callback(twai_node_handle_t handle,
                                         const twai_error_event_data_t *edata,
                                         void *user_ctx) {
  ESP_EARLY_LOGW(TAG, "bus error: 0x%x", edata->err_flags.val);
  return false;
}

/**
 * @brief State change callback - detects Bus-Off and recovery
 */
static bool IRAM_ATTR can_state_change_callback(
    twai_node_handle_t handle, const twai_state_change_event_data_t *edata,
    void *user_ctx) {
  const char *twai_state_name[] = {"error_active", "error_warning",
                                   "error_passive", "bus_off"};
  ESP_EARLY_LOGI(TAG, "state changed: %s -> %s",
                 twai_state_name[edata->old_sta],
                 twai_state_name[edata->new_sta]);
  return false;
}

/**
 * @brief TWAI receive callback - store data and signal task
 * @note Pattern from ESP-IDF example: embedded buffer in queue item
 */
static bool IRAM_ATTR
can_rx_done_callback(twai_node_handle_t handle,
                     const twai_rx_done_event_data_t *edata, void *user_ctx) {
  ESP_UNUSED(edata);
  can_driver_ctx_t *ctx = (can_driver_ctx_t *)user_ctx;
  BaseType_t woken = pdFALSE;

  // Validate context - early return to avoid ISR overhead
  if (!ctx || !atomic_load(&ctx->is_initialized) || !ctx->rx_queue) {
    return false;
  }

  // Prepare queue item with embedded buffer
  rx_queue_item_t item = {0};
  twai_frame_t rx_frame = {
      .buffer = item.buffer,
      .buffer_len = sizeof(item.buffer),
  };

  // Receive frame from ISR
  if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
    // Filter: Only standard frames (ignore extended)
    if (rx_frame.header.ide) {
      return false;
    }

    // Convert to user message format
    item.msg.id = (uint16_t)rx_frame.header.id;
    item.msg.len = rx_frame.buffer_len;
    item.msg.rtr = rx_frame.header.rtr;

    if (item.msg.len > 0 && item.msg.len <= 8) {
      memcpy(item.msg.data, rx_frame.buffer, item.msg.len);
    }

    // Send to queue (non-blocking)
    xQueueSendFromISR(ctx->rx_queue, &item, &woken);
  }

  return (woken == pdTRUE);
}

/**
 * @brief TX done callback - increment counter and toggle debug LED
 */
static bool IRAM_ATTR
can_tx_done_callback(twai_node_handle_t handle,
                     const twai_tx_done_event_data_t *edata, void *user_ctx) {
  ESP_UNUSED(handle);
  ESP_UNUSED(user_ctx);

  // Increment global counter
  g_counter++;

  // Toggle debug LED for visualization
  static bool led_state = false;
  led_state = !led_state;
  gpio_set_level(DEBUG_TX_GPIO, led_state);

  // Log failed transmissions
  if (!edata->is_tx_success) {
    ESP_EARLY_LOGW(TAG, "TX failed for ID: 0x%X",
                   edata->done_tx_frame->header.id);
  }

  return false;
}

/**
 * @brief Initialize and configure the TWAI CAN driver
 * @return can_status_t status code
 */
can_status_t can_driver_init(void) {
  esp_err_t ret = ESP_OK;

  // Configure debug GPIO for TX callback visualization
  gpio_set_direction(DEBUG_TX_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DEBUG_TX_GPIO, 0);

  // Check if already initialized
  if (atomic_load(&g_can_ctx.is_initialized)) {
    ESP_LOGW(TAG, "Driver already initialized");
    return CAN_OK;
  }

  // Create RX queue for buffering received frames
  g_can_ctx.rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_queue_item_t));
  if (!g_can_ctx.rx_queue) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    return CAN_ERR_DRIVER_INSTALL;
  }
  ESP_LOGI(TAG, "Buffer initialized: %d slots for RX data", RX_QUEUE_LEN);

  // Configure TWAI node
  g_can_ctx.driver_config = (twai_onchip_node_config_t){
      .io_cfg =
          {
              .tx = CAN_TX_GPIO_DEFAULT,
              .rx = CAN_RX_GPIO_DEFAULT,
              .quanta_clk_out = GPIO_NUM_NC,
              .bus_off_indicator = GPIO_NUM_NC,
          },
      .clk_src = 0, // Default clock source
      .bit_timing =
          {
              .bitrate = g_can_config.baud_rate,
              .sp_permill = 0, // Auto calculate sample point
              .ssp_permill = 0,
          },
      .data_timing =
          {
              .bitrate = 0, // No TWAI-FD
          },
      .fail_retry_cnt = -1, // Infinite retry
      .tx_queue_depth = 10,
      .intr_priority = 0,
      .flags =
          {
              // Enable both self-test and loopback for GPIO physical loopback
              .enable_self_test = 1,
              .enable_loopback = 1, // CRITICAL: Must be 1 for GPIO loopback
              .enable_listen_only = 0,
              .no_receive_rtr = 0,
          },
  };

  // Create TWAI node
  ret = twai_new_node_onchip(&g_can_ctx.driver_config, &g_can_ctx.node_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(ret));
    goto err_queue;
  }
  ESP_LOGI(TAG, "TWAI node created");

  // Register callbacks
  g_can_ctx.callbacks.on_rx_done = can_rx_done_callback;
  g_can_ctx.callbacks.on_tx_done = can_tx_done_callback;
  g_can_ctx.callbacks.on_error = can_error_callback;
  g_can_ctx.callbacks.on_state_change = can_state_change_callback;

  ret = twai_node_register_event_callbacks(g_can_ctx.node_handle,
                                           &g_can_ctx.callbacks, &g_can_ctx);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
    goto err_node;
  }

  // Configure acceptance filter if needed
  if (g_can_whitelist_count == 1) {
    twai_mask_filter_config_t data_filter = {
        .id = (uint32_t)g_can_whitelist[0],
        .mask = 0x7FF,   // Match all 11 bits
        .is_ext = false, // Receive only standard ID
    };
    ret = twai_node_config_mask_filter(g_can_ctx.node_handle, 0, &data_filter);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Filter enabled for ID: 0x%03X Mask: 0x%03X",
               data_filter.id, data_filter.mask);
    } else {
      ESP_LOGW(TAG, "Failed to set hardware filter: %s", esp_err_to_name(ret));
    }
  } else {
    ESP_LOGI(TAG, "Filter: ACCEPT ALL");
  }

  // Enable TWAI node
  ret = twai_node_enable(g_can_ctx.node_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable node: %s", esp_err_to_name(ret));
    goto err_node;
  }

  atomic_store(&g_can_ctx.is_initialized, true);
  ESP_LOGI(TAG, "TWAI start - Mode: %d, Bitrate: %lu bps",
           g_can_config.operating_mode, g_can_config.baud_rate);

  return CAN_OK;

err_node:
  if (g_can_ctx.node_handle) {
    twai_node_delete(g_can_ctx.node_handle);
    g_can_ctx.node_handle = NULL;
  }

err_queue:
  if (g_can_ctx.rx_queue) {
    vQueueDelete(g_can_ctx.rx_queue);
    g_can_ctx.rx_queue = NULL;
  }

  return CAN_ERR_DRIVER_INSTALL;
}

/**
 * @brief Transmit a CAN message (Standard Frame, non-blocking)
 * @param id Standard 11-bit CAN ID
 * @param data Pointer to data buffer
 * @param len Data length (0-8 bytes)
 * @return can_status_t status code
 */
can_status_t can_transmit(uint16_t id, const uint8_t *data, uint8_t len) {
  // Validate driver state
  if (!atomic_load(&g_can_ctx.is_initialized)) {
    ESP_LOGE(TAG, "Driver not initialized");
    return CAN_ERR_NOT_INITIALIZED;
  }

  if (!g_can_ctx.node_handle) {
    ESP_LOGE(TAG, "Invalid node handle");
    return CAN_ERR_NOT_INITIALIZED;
  }

  // Validate parameters
  if (id > 0x7FF) {
    ESP_LOGE(TAG, "Invalid CAN ID: 0x%X (max 0x7FF)", id);
    return CAN_ERR_INVALID_PARAM;
  }

  if (len > 8) {
    ESP_LOGE(TAG, "Invalid length: %d (max 8)", len);
    return CAN_ERR_INVALID_PARAM;
  }

  // Prepare frame
  twai_frame_t tx_frame = {
      .header =
          {
              .id = id,
              .ide = 0, // Standard frame
              .rtr = 0,
              .fdf = 0,
          },
      .buffer = (uint8_t *)data,
      .buffer_len = len,
  };

  // Transmit with timeout
  esp_err_t ret = twai_node_transmit(g_can_ctx.node_handle, &tx_frame,
                                     pdMS_TO_TICKS(TX_TIMEOUT_MS));

  if (ret == ESP_OK) {
    return CAN_OK;
  } else if (ret == ESP_ERR_TIMEOUT) {
    return CAN_ERR_TX_TIMEOUT;
  } else if (ret == ESP_ERR_INVALID_STATE) {
    return CAN_ERR_BUS_OFF;
  }

  return CAN_ERR_TX_FAILED;
}

/**
 * @brief Receive a CAN message (polling, non-blocking)
 * @param msg Pointer to message structure to fill
 * @return can_status_t CAN_OK if message received, CAN_ERR_RX_NO_DATA if no
 * data
 */
can_status_t can_receive(can_message_t *msg) {
  // Validate driver state
  if (!atomic_load(&g_can_ctx.is_initialized)) {
    return CAN_ERR_NOT_INITIALIZED;
  }

  if (!g_can_ctx.rx_queue) {
    return CAN_ERR_NOT_INITIALIZED;
  }

  if (!msg) {
    return CAN_ERR_INVALID_PARAM;
  }

  // Receive from queue (non-blocking)
  rx_queue_item_t item;
  if (xQueueReceive(g_can_ctx.rx_queue, &item, 0) != pdTRUE) {
    return CAN_ERR_RX_NO_DATA;
  }

  // Software whitelist filter (if multiple IDs)
  if (g_can_whitelist_count > 1) {
    bool found = false;
    for (int i = 0; i < g_can_whitelist_count; i++) {
      if (g_can_whitelist[i] == item.msg.id) {
        found = true;
        break;
      }
    }
    if (!found) {
      return CAN_ERR_RX_NO_DATA; // Filter out non-whitelisted IDs
    }
  }

  // Copy message to user
  *msg = item.msg;
  return CAN_OK;
}

/**
 * @brief Check current TWAI bus status
 * @return can_bus_state_t current bus state
 */
can_bus_state_t can_check_bus_status(void) {
  if (!atomic_load(&g_can_ctx.is_initialized) || !g_can_ctx.node_handle) {
    return CAN_BUS_UNKNOWN;
  }

  twai_node_status_t status;
  twai_node_record_t record;
  esp_err_t ret = twai_node_get_info(g_can_ctx.node_handle, &status, &record);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to get node info: %s", esp_err_to_name(ret));
    return CAN_BUS_UNKNOWN;
  }

  // Map TWAI states to CAN bus states
  switch (status.state) {
  case TWAI_ERROR_BUS_OFF:
    return CAN_BUS_BUS_OFF;
  case TWAI_ERROR_PASSIVE:
    return CAN_BUS_ERROR_PASSIVE;
  case TWAI_ERROR_WARNING:
    return CAN_BUS_WARNING;
  case TWAI_ERROR_ACTIVE:
    return CAN_BUS_RUNNING;
  default:
    return CAN_BUS_UNKNOWN;
  }
}

/**
 * @brief Initiate recovery from Bus-Off state
 * @return can_status_t status code
 */
can_status_t can_initiate_recovery(void) {
  if (!atomic_load(&g_can_ctx.is_initialized)) {
    ESP_LOGE(TAG, "Driver not initialized");
    return CAN_ERR_NOT_INITIALIZED;
  }

  if (!g_can_ctx.node_handle) {
    ESP_LOGE(TAG, "Invalid node handle");
    return CAN_ERR_NOT_INITIALIZED;
  }

  // Check if recovery is needed
  twai_node_status_t status;
  esp_err_t ret = twai_node_get_info(g_can_ctx.node_handle, &status, NULL);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get node status: %s", esp_err_to_name(ret));
    return CAN_ERR_BUS_OFF;
  }

  if (status.state != TWAI_ERROR_BUS_OFF) {
    ESP_LOGI(TAG, "Recovery not needed, current state: %d", status.state);
    return CAN_OK;
  }

  // Initiate recovery
  ret = twai_node_recover(g_can_ctx.node_handle);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Bus recovery initiated");
    return CAN_OK;
  }

  ESP_LOGE(TAG, "Recovery failed: %s", esp_err_to_name(ret));
  return CAN_ERR_BUS_OFF;
}

/**
 * @brief Deinitialize the CAN driver
 * @return can_status_t status code
 */
can_status_t can_driver_deinit(void) {
  if (!atomic_load(&g_can_ctx.is_initialized)) {
    ESP_LOGI(TAG, "Driver not initialized");
    return CAN_OK;
  }

  // Disable and delete TWAI node
  if (g_can_ctx.node_handle) {
    esp_err_t ret = twai_node_disable(g_can_ctx.node_handle);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to disable node: %s", esp_err_to_name(ret));
    }

    ret = twai_node_delete(g_can_ctx.node_handle);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to delete node: %s", esp_err_to_name(ret));
    }

    g_can_ctx.node_handle = NULL;
  }

  // Delete RX queue
  if (g_can_ctx.rx_queue) {
    vQueueDelete(g_can_ctx.rx_queue);
    g_can_ctx.rx_queue = NULL;
  }

  atomic_store(&g_can_ctx.is_initialized, false);
  ESP_LOGI(TAG, "CAN driver deinitialized");

  return CAN_OK;
}
