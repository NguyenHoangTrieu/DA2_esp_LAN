// BSP/SDCard_Driver/src/sdcard_hal.c

#include "sdcard_hal.h"
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "SDCARD_HAL";

typedef struct {
  sdcard_hal_config_t config;
  sdcard_hal_stats_t stats;

  sdmmc_card_t *card;

  uint8_t *ring_buffer;
  uint32_t write_idx;

  SemaphoreHandle_t mutex;
  TaskHandle_t flush_task;

  volatile bool mounted;
  volatile bool running;
} sdcard_hal_t;

static sdcard_hal_t g_sdcard = {0};

static void sdcard_flush_task(void *arg);
static esp_err_t sdcard_flush_internal(void);

esp_err_t sdcard_hal_init(const sdcard_hal_config_t *config) {
  if (config == NULL)
    return ESP_ERR_INVALID_ARG;

  if (g_sdcard.mounted) {
    ESP_LOGW(TAG, "Already initialized");
    return ESP_OK;
  }

  memcpy(&g_sdcard.config, config, sizeof(sdcard_hal_config_t));

  ESP_LOGI(TAG, "Init SDMMC: CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d",
           config->gpio_clk, config->gpio_cmd, config->gpio_d0, config->gpio_d1,
           config->gpio_d2, config->gpio_d3);

  // SDMMC host config
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40MHz for better performance

  // Slot config
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();

#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
  slot.clk = config->gpio_clk;
  slot.cmd = config->gpio_cmd;
  slot.d0 = config->gpio_d0;
  if (config->bus_width_4) {
    slot.d1 = config->gpio_d1;
    slot.d2 = config->gpio_d2;
    slot.d3 = config->gpio_d3;
    slot.width = 4;
  } else {
    slot.width = 1;
  }
#else
  slot.width = config->bus_width_4 ? 4 : 1;
#endif

  slot.cd = SDMMC_SLOT_NO_CD;
  slot.wp = SDMMC_SLOT_NO_WP;
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  // VFS mount config
  esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
      .format_if_mount_failed = config->format_if_failed,
      .max_files = config->max_open_files,
      .allocation_unit_size = 32 * 1024 // 32KB clusters for better streaming
  };

  // Mount
  esp_err_t ret = esp_vfs_fat_sdmmc_mount(SDCARD_MOUNT_POINT, &host, &slot,
                                          &mount_cfg, &g_sdcard.card);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
    return ret;
  }

  sdmmc_card_print_info(stdout, g_sdcard.card);

  // Allocate ring buffer (DMA-capable for potential optimization)
  g_sdcard.ring_buffer = heap_caps_malloc(SDCARD_RING_SIZE, MALLOC_CAP_8BIT);
  if (g_sdcard.ring_buffer == NULL) {
    esp_vfs_fat_sdcard_unmount(SDCARD_MOUNT_POINT, g_sdcard.card);
    return ESP_ERR_NO_MEM;
  }

  g_sdcard.write_idx = 0;
  g_sdcard.mutex = xSemaphoreCreateMutex();

  if (g_sdcard.mutex == NULL) {
    free(g_sdcard.ring_buffer);
    esp_vfs_fat_sdcard_unmount(SDCARD_MOUNT_POINT, g_sdcard.card);
    return ESP_ERR_NO_MEM;
  }

  g_sdcard.mounted = true;

  ESP_LOGI(TAG, "Initialized: %dKB ring buffer", SDCARD_RING_SIZE / 1024);
  return ESP_OK;
}

esp_err_t sdcard_hal_deinit(void) {
  if (!g_sdcard.mounted)
    return ESP_OK;

  if (g_sdcard.running)
    sdcard_hal_stop();

  // Final flush
  if (g_sdcard.write_idx > 0) {
    ESP_LOGI(TAG, "Final flush: %lu bytes", g_sdcard.write_idx);
    sdcard_flush_internal();
  }

  vSemaphoreDelete(g_sdcard.mutex);
  free(g_sdcard.ring_buffer);

  esp_err_t ret = esp_vfs_fat_sdcard_unmount(SDCARD_MOUNT_POINT, g_sdcard.card);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Unmount failed: %s", esp_err_to_name(ret));
  }

  g_sdcard.mounted = false;
  memset(&g_sdcard, 0, sizeof(sdcard_hal_t));

  ESP_LOGI(TAG, "Deinitialized");
  return ret;
}

esp_err_t sdcard_hal_start(void) {
  if (!g_sdcard.mounted)
    return ESP_ERR_INVALID_STATE;

  if (g_sdcard.running)
    return ESP_ERR_INVALID_STATE;

  g_sdcard.running = true;

  BaseType_t ret = xTaskCreate(sdcard_flush_task, "sd_flush", 3072, NULL, 5,
                               &g_sdcard.flush_task);

  if (ret != pdPASS) {
    g_sdcard.running = false;
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Started flush task");
  return ESP_OK;
}

esp_err_t sdcard_hal_stop(void) {
  if (!g_sdcard.running)
    return ESP_ERR_INVALID_STATE;

  g_sdcard.running = false;

  if (g_sdcard.flush_task) {
    vTaskDelete(g_sdcard.flush_task);
    g_sdcard.flush_task = NULL;
  }

  return ESP_OK;
}

esp_err_t sdcard_hal_append(const uint8_t *data, size_t len) {
  if (!g_sdcard.mounted)
    return ESP_ERR_INVALID_STATE;

  if (data == NULL || len == 0)
    return ESP_ERR_INVALID_ARG;

  if (len > SDCARD_RING_SIZE) {
    ESP_LOGE(TAG, "Data too large: %zu > %d", len, SDCARD_RING_SIZE);
    return ESP_ERR_INVALID_SIZE;
  }

  if (xSemaphoreTake(g_sdcard.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  // Check if buffer would overflow
  if (g_sdcard.write_idx + len > SDCARD_RING_SIZE) {
    // Force flush before appending
    xSemaphoreGive(g_sdcard.mutex);

    esp_err_t ret = sdcard_hal_force_flush();
    if (ret != ESP_OK)
      return ret;

    if (xSemaphoreTake(g_sdcard.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
      return ESP_ERR_TIMEOUT;
  }

  memcpy(&g_sdcard.ring_buffer[g_sdcard.write_idx], data, len);
  g_sdcard.write_idx += len;
  g_sdcard.stats.bytes_buffered = g_sdcard.write_idx;

  xSemaphoreGive(g_sdcard.mutex);

  return ESP_OK;
}

esp_err_t sdcard_hal_force_flush(void) {
  if (!g_sdcard.mounted)
    return ESP_ERR_INVALID_STATE;

  if (g_sdcard.write_idx == 0)
    return ESP_OK;

  return sdcard_flush_internal();
}

static esp_err_t sdcard_flush_internal(void) {
  if (xSemaphoreTake(g_sdcard.mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    ESP_LOGW(TAG, "Flush mutex timeout");
    return ESP_ERR_TIMEOUT;
  }

  if (g_sdcard.write_idx == 0) {
    xSemaphoreGive(g_sdcard.mutex);
    return ESP_OK;
  }

  uint32_t bytes_to_write = g_sdcard.write_idx;

  // Open in append mode
  FILE *f = fopen(SDCARD_DATA_FILE, "ab");
  if (f == NULL) {
    ESP_LOGE(TAG, "fopen failed: %s", strerror(errno));
    g_sdcard.stats.write_errors++;
    xSemaphoreGive(g_sdcard.mutex);
    return ESP_FAIL;
  }

  // Single batch write
  size_t written = fwrite(g_sdcard.ring_buffer, 1, bytes_to_write, f);

  if (written != bytes_to_write) {
    ESP_LOGE(TAG, "Write incomplete: %zu/%lu", written, bytes_to_write);
    fclose(f);
    g_sdcard.stats.write_errors++;
    xSemaphoreGive(g_sdcard.mutex);
    return ESP_FAIL;
  }

  // Sync once per batch (critical for data integrity)
  fflush(f);
  fsync(fileno(f));
  fclose(f);

  // Reset buffer
  g_sdcard.write_idx = 0;
  g_sdcard.stats.bytes_buffered = 0;
  g_sdcard.stats.flush_count++;
  g_sdcard.stats.bytes_written_total += bytes_to_write;

  xSemaphoreGive(g_sdcard.mutex);

  ESP_LOGD(TAG, "Flushed %lu bytes (total: %lu, flushes: %lu)", bytes_to_write,
           g_sdcard.stats.bytes_written_total, g_sdcard.stats.flush_count);

  return ESP_OK;
}

static void sdcard_flush_task(void *arg) {
  uint32_t check_counter = 0;
  const uint32_t check_ticks = 10; // 100ms * 10 = 1 second

  while (g_sdcard.running) {
    check_counter++;

    if (check_counter >= check_ticks) {
      check_counter = 0;

      // Check threshold
      if (g_sdcard.write_idx >= SDCARD_FLUSH_THRESHOLD) {
        ESP_LOGD(TAG, "Auto-flush: %lu/%d bytes", g_sdcard.write_idx,
                 SDCARD_FLUSH_THRESHOLD);
        sdcard_flush_internal();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  vTaskDelete(NULL);
}

esp_err_t sdcard_hal_get_stats(sdcard_hal_stats_t *stats) {
  if (stats == NULL)
    return ESP_ERR_INVALID_ARG;

  if (!g_sdcard.mounted)
    return ESP_ERR_INVALID_STATE;

  if (xSemaphoreTake(g_sdcard.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  memcpy(stats, &g_sdcard.stats, sizeof(sdcard_hal_stats_t));

  // Get card size
  FATFS *fs;
  DWORD free_clusters;

  if (f_getfree("0:", &free_clusters, &fs) == FR_OK) {
    uint64_t total_sectors = (fs->n_fatent - 2) * fs->csize;
    uint64_t free_sectors = free_clusters * fs->csize;

    stats->total_bytes = total_sectors * 512; // FAT sector size
    stats->used_bytes = (total_sectors - free_sectors) * 512;
  }

  xSemaphoreGive(g_sdcard.mutex);

  return ESP_OK;
}

bool sdcard_hal_is_mounted(void) { return g_sdcard.mounted; }

uint32_t sdcard_hal_get_free_space(void) {
  if (!g_sdcard.mounted)
    return 0;

  FATFS *fs;
  DWORD free_clusters;

  if (f_getfree("0:", &free_clusters, &fs) != FR_OK)
    return 0;

  return (uint32_t)((free_clusters * fs->csize * 512) / (1024 * 1024)); // MB
}
