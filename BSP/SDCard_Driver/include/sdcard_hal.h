// BSP/SDCard_Driver/include/sdcard_hal.h

#ifndef SDCARD_HAL_H
#define SDCARD_HAL_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration
#define SDCARD_MOUNT_POINT "/sdcard"
#define SDCARD_DATA_FILE "/sdcard/stream.log"
#define SDCARD_RING_SIZE (100 * 1024)      // 100KB ring buffer
#define SDCARD_FLUSH_THRESHOLD (80 * 1024) // Flush at 80KB

typedef struct {
  int gpio_clk;
  int gpio_cmd;
  int gpio_d0;
  int gpio_d1;
  int gpio_d2;
  int gpio_d3;
  bool bus_width_4;
  bool format_if_failed;
  uint8_t max_open_files;
} sdcard_hal_config_t;

typedef struct {
  uint64_t total_bytes;
  uint64_t used_bytes;
  uint32_t bytes_buffered; // In ring buffer
  uint32_t flush_count;
  uint32_t write_errors;
  uint32_t bytes_written_total;
} sdcard_hal_stats_t;

// Core API
esp_err_t sdcard_hal_init(const sdcard_hal_config_t *config);
esp_err_t sdcard_hal_deinit(void);
esp_err_t sdcard_hal_start(void);
esp_err_t sdcard_hal_stop(void);

// Data API (thread-safe)
esp_err_t sdcard_hal_append(const uint8_t *data, size_t len);
esp_err_t sdcard_hal_force_flush(void);

// Status API
esp_err_t sdcard_hal_get_stats(sdcard_hal_stats_t *stats);
bool sdcard_hal_is_mounted(void);
uint32_t sdcard_hal_get_free_space(void);

#ifdef __cplusplus
}
#endif

#endif // SDCARD_HAL_H
