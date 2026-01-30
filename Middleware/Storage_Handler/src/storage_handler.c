/**
 * @file storage_handler.c
 * @brief Storage Handler - Thread-safe wrapper with 100KB batch buffering + 5s
 * timeout flush
 */

#include "storage_handler.h"
#include "SDCard_comm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static const char *TAG = "STORAGE";

/* Configuration - 100KB buffer per design doc */
#define BATCH_BUFFER_SIZE 102400 // 100 KB
#define MAX_PACKET_SIZE 8192     // Max single packet size
#define FLUSH_TIMEOUT_MS 5000    // 5 seconds idle timeout

/* Batch Buffer Structure */
typedef struct {
  uint8_t buffer[BATCH_BUFFER_SIZE];
  uint32_t write_pos;    // Current write position in buffer
  uint32_t packet_count; // Number of packets in buffer
  bool needs_flush;      // Flag indicating buffer needs flushing
} batch_buffer_t;

/* Global State - Allocate from PSRAM for 100KB buffer */
static SemaphoreHandle_t g_storage_mutex = NULL;
static bool g_storage_initialized = false;
static batch_buffer_t *g_batch_buffer = NULL;
static TimerHandle_t g_flush_timer = NULL; // Timer for 5s flush
static int64_t g_last_write_us = 0;        // Track last write timestamp

/* Retry State */
static FILE *g_retry_file = NULL;
static char g_retry_path[64] = {0};
static uint32_t g_retry_offset = 0;

/* Forward Declarations */
static esp_err_t flush_batch_buffer(void);
static void flush_timer_callback(TimerHandle_t xTimer);

/* ========== Private Helper: Timer Callback ========== */

/**
 * @brief Timer callback to check for 5s timeout flush
 */
static void flush_timer_callback(TimerHandle_t xTimer) {
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Check if buffer has data and 5s elapsed since last write
    int64_t now_us = esp_timer_get_time();
    bool timeout_elapsed =
        (now_us - g_last_write_us) >= (FLUSH_TIMEOUT_MS * 1000);

    if (g_batch_buffer->needs_flush && g_batch_buffer->write_pos > 0 &&
        timeout_elapsed) {
      ESP_LOGI(TAG, "Timeout flush: %u bytes after %ums idle",
               g_batch_buffer->write_pos, FLUSH_TIMEOUT_MS);
      flush_batch_buffer();
    }
    xSemaphoreGive(g_storage_mutex);
  }
}

/* ========== Public API ========== */

esp_err_t storage_handler_init(void) {
  if (g_storage_initialized) {
    ESP_LOGW(TAG, "Storage handler already initialized");
    return ESP_OK;
  }

  /* Allocate 100KB buffer from PSRAM if available */
  g_batch_buffer = heap_caps_malloc(sizeof(batch_buffer_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (g_batch_buffer == NULL) {
    /* Fallback to DRAM if PSRAM not available */
    ESP_LOGW(TAG, "PSRAM not available, allocating 100KB from DRAM");
    g_batch_buffer = heap_caps_malloc(sizeof(batch_buffer_t),
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (g_batch_buffer == NULL) {
      ESP_LOGE(TAG, "Failed to allocate 100KB batch buffer");
      return ESP_ERR_NO_MEM;
    }
  }

  /* Create storage mutex for thread-safe SD card access */
  g_storage_mutex = xSemaphoreCreateMutex();
  if (g_storage_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create storage mutex");
    heap_caps_free(g_batch_buffer);
    g_batch_buffer = NULL;
    return ESP_FAIL;
  }

  /* Initialize SD card using sd_card_comm */
  sd_card_config_t sd_config = SD_CARD_CONFIG_DEFAULT();
  esp_err_t ret = sd_card_init(&sd_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
    vSemaphoreDelete(g_storage_mutex);
    heap_caps_free(g_batch_buffer);
    g_storage_mutex = NULL;
    g_batch_buffer = NULL;
    return ret;
  }

  /* Initialize batch buffer */
  memset(g_batch_buffer, 0, sizeof(batch_buffer_t));
  g_last_write_us = esp_timer_get_time(); // Initialize timestamp

  /* Create 5-second flush timer */
  g_flush_timer = xTimerCreate("sd_flush",
                               pdMS_TO_TICKS(1000), // Check every 1s
                               pdTRUE,              // Auto-reload
                               NULL, flush_timer_callback);
  if (g_flush_timer == NULL) {
    ESP_LOGE(TAG, "Failed to create flush timer");
    sd_card_deinit();
    vSemaphoreDelete(g_storage_mutex);
    heap_caps_free(g_batch_buffer);
    g_storage_mutex = NULL;
    g_batch_buffer = NULL;
    return ESP_FAIL;
  }

  if (xTimerStart(g_flush_timer, 0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start flush timer");
    xTimerDelete(g_flush_timer, pdMS_TO_TICKS(100));
    g_flush_timer = NULL;
    sd_card_deinit();
    vSemaphoreDelete(g_storage_mutex);
    heap_caps_free(g_batch_buffer);
    g_storage_mutex = NULL;
    g_batch_buffer = NULL;
    return ESP_FAIL;
  }

  g_storage_initialized = true;
  ESP_LOGI(
      TAG,
      "Storage handler initialized: 100KB batch buffer with %ums flush timer",
      FLUSH_TIMEOUT_MS);
  ESP_LOGI(TAG, "Buffer allocated from: %s",
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > 0 ? "PSRAM" : "DRAM");

  return ESP_OK;
}

esp_err_t storage_handler_deinit(void) {
  if (!g_storage_initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Deinitializing storage handler");

  /* Stop and delete timer */
  if (g_flush_timer != NULL) {
    xTimerStop(g_flush_timer, pdMS_TO_TICKS(100));
    xTimerDelete(g_flush_timer, pdMS_TO_TICKS(100));
    g_flush_timer = NULL;
  }

  /* Take mutex and flush any pending data */
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (g_batch_buffer->write_pos > 0) {
      ESP_LOGI(TAG, "Flushing pending batch buffer: %u bytes",
               g_batch_buffer->write_pos);
      flush_batch_buffer();
    }
    xSemaphoreGive(g_storage_mutex);
  }

  /* Deinitialize SD card */
  sd_card_deinit();

  /* Delete mutex */
  if (g_storage_mutex != NULL) {
    vSemaphoreDelete(g_storage_mutex);
    g_storage_mutex = NULL;
  }

  /* Free buffer */
  if (g_batch_buffer != NULL) {
    heap_caps_free(g_batch_buffer);
    g_batch_buffer = NULL;
  }

  g_storage_initialized = false;
  ESP_LOGI(TAG, "Storage handler deinitialized");
  return ESP_OK;
}

esp_err_t storage_handler_save(const uint8_t *data, uint16_t length) {
  if (!g_storage_initialized) {
    ESP_LOGE(TAG, "Storage handler not initialized");
    return ESP_FAIL;
  }

  if (data == NULL || length == 0) {
    ESP_LOGE(TAG, "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  if (length > MAX_PACKET_SIZE) {
    ESP_LOGE(TAG, "Packet too large: %u > %u", length, MAX_PACKET_SIZE);
    return ESP_ERR_INVALID_SIZE;
  }

  /* Take mutex for thread-safe access */
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire storage mutex (timeout)");
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = ESP_OK;

  /* Calculate space needed: 2 bytes length + data */
  uint32_t packet_size = 2 + length;

  /* Check if packet fits in remaining buffer space */
  if (g_batch_buffer->write_pos + packet_size > BATCH_BUFFER_SIZE) {
    /* Buffer full - flush first */
    ESP_LOGI(
        TAG,
        "Batch buffer full (%u/%u bytes), flushing before adding new packet",
        g_batch_buffer->write_pos, BATCH_BUFFER_SIZE);
    ret = flush_batch_buffer();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to flush batch buffer: %s", esp_err_to_name(ret));
      xSemaphoreGive(g_storage_mutex);
      return ret;
    }

    /* Check if single packet is larger than entire buffer */
    if (packet_size > BATCH_BUFFER_SIZE) {
      /* Packet too large for batch buffer - write directly to SD */
      ESP_LOGW(TAG,
               "Packet size %u exceeds batch buffer, writing directly to SD",
               packet_size);
      ret = sd_card_save(data, length);
      xSemaphoreGive(g_storage_mutex);
      return ret;
    }
  }

  /* Add packet to batch buffer: Format [length(2 bytes)][data] */
  g_batch_buffer->buffer[g_batch_buffer->write_pos++] = (length >> 8) & 0xFF;
  g_batch_buffer->buffer[g_batch_buffer->write_pos++] = length & 0xFF;
  memcpy(&g_batch_buffer->buffer[g_batch_buffer->write_pos], data, length);
  g_batch_buffer->write_pos += length;
  g_batch_buffer->packet_count++;
  g_batch_buffer->needs_flush = true;
  g_last_write_us = esp_timer_get_time(); // Update timestamp on write

  ESP_LOGD(
      TAG,
      "Packet added to batch buffer: %u bytes, %lu packets, %u/%u buffer used",
      length, g_batch_buffer->packet_count, g_batch_buffer->write_pos,
      BATCH_BUFFER_SIZE);

  xSemaphoreGive(g_storage_mutex);
  return ret;
}

esp_err_t storage_handler_flush(void) {
  if (!g_storage_initialized) {
    ESP_LOGE(TAG, "Storage handler not initialized");
    return ESP_FAIL;
  }

  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire storage mutex (timeout)");
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = ESP_OK;
  if (g_batch_buffer->write_pos > 0) {
    ESP_LOGI(TAG, "Manual flush requested: %u bytes, %lu packets",
             g_batch_buffer->write_pos, g_batch_buffer->packet_count);
    ret = flush_batch_buffer();
  } else {
    ESP_LOGD(TAG, "Batch buffer empty, nothing to flush");
  }

  xSemaphoreGive(g_storage_mutex);
  return ret;
}

esp_err_t storage_handler_read_oldest(uint8_t *buffer, uint32_t *length,
                                      uint32_t buffer_size) {
  if (!g_storage_initialized) {
    ESP_LOGE(TAG, "Storage handler not initialized");
    return ESP_FAIL;
  }

  if (buffer == NULL || length == NULL) {
    ESP_LOGE(TAG, "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread-safe SD card access */
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire storage mutex (timeout)");
    return ESP_ERR_TIMEOUT;
  }

  /* Read oldest file from SD card */
  esp_err_t ret = sd_card_read_oldest(buffer, length, buffer_size);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Read oldest data from SD card: %u bytes", *length);
  } else if (ret == ESP_ERR_NOT_FOUND) {
    ESP_LOGD(TAG, "No data files found on SD card");
  } else {
    ESP_LOGE(TAG, "Failed to read oldest data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(g_storage_mutex);
  return ret;
}

esp_err_t storage_handler_delete_oldest(void) {
  if (!g_storage_initialized) {
    ESP_LOGE(TAG, "Storage handler not initialized");
    return ESP_FAIL;
  }

  /* Take mutex for thread-safe SD card access */
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire storage mutex (timeout)");
    return ESP_ERR_TIMEOUT;
  }

  /* Delete oldest file from SD card */
  esp_err_t ret = sd_card_delete_oldest();
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Deleted oldest data from SD card, remaining files: %lu",
             sd_card_get_file_count());
  } else if (ret == ESP_ERR_NOT_FOUND) {
    ESP_LOGD(TAG, "No data files to delete");
  } else {
    ESP_LOGE(TAG, "Failed to delete oldest data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(g_storage_mutex);
  return ret;
}

bool storage_handler_has_data(void) {
  if (!g_storage_initialized) {
    return false;
  }

  /* Check both batch buffer and SD card */
  if (g_batch_buffer->write_pos > 0) {
    return true; // Data in buffer
  }

  /* Check SD card with mutex protection */
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire storage mutex for has_data check");
    return false;
  }

  bool has_data = sd_card_has_data();
  xSemaphoreGive(g_storage_mutex);
  return has_data;
}

uint32_t storage_handler_get_file_count(void) {
  if (!g_storage_initialized) {
    return 0;
  }

  /* Count includes pending buffer packets + SD card files */
  uint32_t buffer_packets = g_batch_buffer->packet_count;

  /* Get SD card file count with mutex protection */
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire storage mutex for file count");
    return buffer_packets;
  }

  uint32_t sd_count = sd_card_get_file_count();
  xSemaphoreGive(g_storage_mutex);

  return buffer_packets + sd_count;
}

/* ========== Private Helper Functions ========== */

/**
 * @brief Flush batch buffer to SD card with 0xFF padding to 100KB
 * @note Caller must hold g_storage_mutex
 * @return ESP_OK on success
 */
static esp_err_t flush_batch_buffer(void) {
  if (g_batch_buffer->write_pos == 0) {
    ESP_LOGD(TAG, "Batch buffer empty, nothing to flush");
    return ESP_OK;
  }

  ESP_LOGI(TAG,
           "Flushing batch buffer: %u bytes, %lu packets (padding to 100KB)",
           g_batch_buffer->write_pos, g_batch_buffer->packet_count);

  /* Pad to 100KB with 0xFF (SD card erase state) */
  size_t padding = BATCH_BUFFER_SIZE - g_batch_buffer->write_pos;
  if (padding > 0) {
    memset(&g_batch_buffer->buffer[g_batch_buffer->write_pos], 0xFF, padding);
    ESP_LOGD(TAG, "Added %u bytes of 0xFF padding", padding);
  }

  /* Write entire 100KB buffer to SD card as a single file */
  esp_err_t ret = sd_card_save(g_batch_buffer->buffer, BATCH_BUFFER_SIZE);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG,
             "Batch buffer flushed successfully (100KB), total SD files: %lu",
             sd_card_get_file_count());
    /* Reset batch buffer */
    g_batch_buffer->write_pos = 0;
    g_batch_buffer->packet_count = 0;
    g_batch_buffer->needs_flush = false;
    g_last_write_us = esp_timer_get_time(); // Reset timestamp after flush
  } else {
    ESP_LOGE(TAG, "Failed to flush batch buffer to SD card: %s",
             esp_err_to_name(ret));
  }

  return ret;
}

/* ========== Stream Read API (Fix for Batch vs Packet Issue) ========== */

esp_err_t storage_handler_prepare_retry(void) {
  if (!g_storage_initialized) {
    return ESP_FAIL;
  }

  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  /* Close any existing file */
  if (g_retry_file) {
    fclose(g_retry_file);
    g_retry_file = NULL;
  }

  /* Find oldest file */
  if (sd_card_get_oldest_file_path(g_retry_path, sizeof(g_retry_path)) !=
      ESP_OK) {
    xSemaphoreGive(g_storage_mutex);
    return ESP_ERR_NOT_FOUND;
  }

  /* Open file for reading */
  g_retry_file = fopen(g_retry_path, "rb");
  if (g_retry_file == NULL) {
    ESP_LOGE(TAG, "Failed to open retry file: %s", g_retry_path);
    xSemaphoreGive(g_storage_mutex);
    return ESP_FAIL;
  }

  g_retry_offset = 0;
  ESP_LOGI(TAG, "Opened retry file: %s", g_retry_path);

  xSemaphoreGive(g_storage_mutex);
  return ESP_OK;
}

esp_err_t storage_handler_get_next_packet(uint8_t *buffer, uint16_t *length,
                                          uint16_t max_len) {
  if (!g_retry_file) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  // Seek to current offset
  fseek(g_retry_file, g_retry_offset, SEEK_SET);

  // Read packet length (2 bytes)
  uint8_t len_bytes[2];
  size_t read = fread(len_bytes, 1, 2, g_retry_file);

  if (read < 2) {
    // End of file or error
    xSemaphoreGive(g_storage_mutex);
    return ESP_ERR_NOT_FOUND;
  }

  // Check for 0xFFFF padding (end of valid data in batch)
  if (len_bytes[0] == 0xFF && len_bytes[1] == 0xFF) {
    xSemaphoreGive(g_storage_mutex);
    return ESP_ERR_NOT_FOUND;
  }

  uint16_t packet_len = (len_bytes[0] << 8) | len_bytes[1];

  if (packet_len == 0 || packet_len > MAX_PACKET_SIZE) {
    ESP_LOGW(TAG, "Invalid packet length at offset %lu: %u", g_retry_offset,
             packet_len);
    xSemaphoreGive(g_storage_mutex);
    return ESP_ERR_INVALID_SIZE;
  }

  if (packet_len > max_len) {
    xSemaphoreGive(g_storage_mutex);
    return ESP_ERR_NO_MEM; // Buffer too small
  }

  // Read payload
  read = fread(buffer, 1, packet_len, g_retry_file);
  if (read < packet_len) {
    ESP_LOGE(TAG, "Incomplete packet at offset %lu", g_retry_offset);
    xSemaphoreGive(g_storage_mutex);
    return ESP_FAIL;
  }

  *length = packet_len;
  g_retry_offset += (2 + packet_len);

  xSemaphoreGive(g_storage_mutex);
  return ESP_OK;
}

void storage_handler_finish_retry(bool success) {
  if (xSemaphoreTake(g_storage_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (g_retry_file) {
      fclose(g_retry_file);
      g_retry_file = NULL;
    }

    if (success && strlen(g_retry_path) > 0) {
      ESP_LOGI(TAG, "Retry successful, deleting file: %s", g_retry_path);
      unlink(g_retry_path);
    } else {
      ESP_LOGW(TAG, "Retry aborted or failed, keeping file: %s", g_retry_path);
    }

    memset(g_retry_path, 0, sizeof(g_retry_path));
    xSemaphoreGive(g_storage_mutex);
  }
}
