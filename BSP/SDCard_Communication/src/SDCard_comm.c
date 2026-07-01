/**
 * @file SDCard_comm.c
 * @brief SD Card Communication Handler Implementation - FIFO Queue Storage
 */

#include "SDCard_comm.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "SDCard_COMM";

// ===== Private Variables =====
static bool g_sd_mounted = false;
static sdmmc_card_t *g_card = NULL;
static uint32_t g_file_counter = 0;    // Sequence counter for new files
static uint32_t g_oldest_file_num = 0; // Track oldest file number

// ===== Private Function Declarations =====
static esp_err_t scan_and_update_counters(void);
static esp_err_t find_oldest_file(char *filename, size_t max_len);
static uint32_t extract_file_number(const char *filename);

// ===== Public API Implementation =====

esp_err_t sd_card_init(const sd_card_config_t *config) {
  if (g_sd_mounted) {
    ESP_LOGW(TAG, "SD card already initialized");
    return ESP_OK;
  }

  if (config == NULL) {
    ESP_LOGE(TAG, "Invalid config parameter");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Initializing SD card via SDMMC");

  // Configure SDMMC host
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_DEFAULT;

  // Configure slot
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  // Thêm log trong SDCard_comm.c sau dòng 48:
  ESP_LOGI(TAG, "GPIO pins: CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d",
           config->gpio_clk, config->gpio_cmd, config->gpio_d0, config->gpio_d1,
           config->gpio_d2, config->gpio_d3);
// GPIO configuration (for chips with GPIO matrix)
#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
  slot_config.clk = config->gpio_clk;
  slot_config.cmd = config->gpio_cmd;
  slot_config.d0 = config->gpio_d0;
  if (config->bus_width_4) {
    slot_config.d1 = config->gpio_d1;
    slot_config.d2 = config->gpio_d2;
    slot_config.d3 = config->gpio_d3;
    slot_config.width = 4;
  } else {
    slot_config.width = 1;
  }
#else
  // For chips with dedicated SDMMC pins
  slot_config.width = config->bus_width_4 ? 4 : 1;
#endif
  // Disable CD/WP, use external pull-ups
  slot_config.cd = SDMMC_SLOT_NO_CD;
  slot_config.wp = SDMMC_SLOT_NO_WP;
  vTaskDelay(pdMS_TO_TICKS(1000)); // 1000ms
  // Configure VFS FAT mount
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = config->format_if_failed,
      .max_files = config->max_files,
      .allocation_unit_size = 16 * 1024};
  // Mount filesystem
  esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config,
                                          &mount_config, &g_card);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG,
               "Failed to mount filesystem. Check format_if_failed option.");
    } else {
      ESP_LOGE(TAG,
               "Failed to initialize card (%s). Check SD card connections.",
               esp_err_to_name(ret));
    }
    return ret;
  }

  g_sd_mounted = true;

  // Print card info
  sdmmc_card_print_info(stdout, g_card);

  // Create queue directory if not exists
  struct stat st;
  if (stat(SD_DATA_DIR, &st) != 0) {
    ESP_LOGI(TAG, "Creating queue directory: %s", SD_DATA_DIR);
    if (mkdir(SD_DATA_DIR, 0775) != 0) {
      ESP_LOGE(TAG, "Failed to create queue directory");
      sd_card_deinit();
      return ESP_FAIL;
    }
  }

  // Scan existing files and update counters
  ret = scan_and_update_counters();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to scan existing files");
  }

  ESP_LOGI(TAG, "SD card initialized successfully. Files in queue: %lu",
           sd_card_get_file_count());

  // Test write capability
  ESP_LOGI(TAG, "Testing write capability...");
  FILE *test = fopen(SD_DATA_DIR "/test_write.tmp", "wb");
  if (test == NULL) {
    ESP_LOGE(TAG, "CRITICAL: Cannot create test file: %s (errno=%d)",
             strerror(errno), errno);

    // Try to understand why
    struct stat st;
    if (stat(SD_DATA_DIR, &st) == 0) {
      ESP_LOGI(TAG, "Directory exists, permissions: 0%o", st.st_mode & 0777);
    }

    // Don't fail init, but warn
    ESP_LOGW(TAG, "SD card may be read-only!");
  } else {
    uint8_t test_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    fwrite(test_data, 1, 4, test);
    fclose(test);

    // Verify
    test = fopen(SD_DATA_DIR "/test_write.tmp", "rb");
    if (test) {
      uint8_t verify[4];
      fread(verify, 1, 4, test);
      fclose(test);

      if (memcmp(test_data, verify, 4) == 0) {
        ESP_LOGI(TAG, "✓ Write test PASSED");
      } else {
        ESP_LOGE(TAG, "✗ Write test FAILED (data mismatch)");
      }
    }

    unlink(SD_DATA_DIR "/test_write.tmp");
  }
  return ESP_OK;
}

esp_err_t sd_card_deinit(void) {
  if (!g_sd_mounted) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Unmounting SD card");

  esp_err_t ret = esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, g_card);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    return ret;
  }

  g_sd_mounted = false;
  g_card = NULL;
  g_file_counter = 0;
  g_oldest_file_num = 0;

  ESP_LOGI(TAG, "SD card deinitialized");
  return ESP_OK;
}

esp_err_t sd_card_save(const uint8_t *data, uint32_t length) {
  if (!g_sd_mounted) {
    ESP_LOGE(TAG, "SD card not mounted");
    return ESP_FAIL;
  }

  if (data == NULL || length == 0) {
    ESP_LOGE(TAG, "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  // Check if we've reached max files
  if (sd_card_get_file_count() >= SD_MAX_FILES) {
    ESP_LOGE(TAG, "SD card queue full (%d files)", SD_MAX_FILES);
    return ESP_ERR_NO_MEM;
  }

  // Generate filename with sequence number
  char filepath[SD_MAX_FILENAME_LEN];
  snprintf(filepath, sizeof(filepath), "%s/%s%08lu.dat", SD_DATA_DIR,
           SD_FILE_PREFIX, g_file_counter);

  ESP_LOGI(TAG, "Saving %u bytes to: %s", length, filepath);

  // Reset errno before fopen
  errno = 0;

  // Open file for writing
  FILE *f = fopen(filepath, "wb");
  if (f == NULL) {
    int err = errno; // Capture immediately
    ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
    ESP_LOGE(TAG, "errno=%d: %s", err, strerror(err));

    // Additional debug
    struct stat st;
    if (stat(SD_DATA_DIR, &st) != 0) {
      ESP_LOGE(TAG, "Directory '%s' disappeared! Recreating...", SD_DATA_DIR);
      if (mkdir(SD_DATA_DIR, 0775) == 0) {
        ESP_LOGI(TAG, "Directory recreated, retrying...");
        f = fopen(filepath, "wb");
        if (f) {
          goto file_opened_ok;
        }
      }
    } else {
      ESP_LOGI(TAG, "Directory still exists, mode=0%o", st.st_mode);

      // Try to list directory
      DIR *dir = opendir(SD_DATA_DIR);
      if (dir) {
        ESP_LOGI(TAG, "Directory contents:");
        struct dirent *entry;
        int count = 0;
        while ((entry = readdir(dir)) != NULL) {
          ESP_LOGI(TAG, "  %s", entry->d_name);
          if (++count > 10)
            break; // Limit output
        }
        closedir(dir);
      } else {
        ESP_LOGE(TAG, "Cannot open directory for listing: %s", strerror(errno));
      }
    }

    return ESP_FAIL;
  }

file_opened_ok:
  // Write length header (4 bytes) + data
  uint8_t len_header[4] = {(length >> 24) & 0xFF, (length >> 16) & 0xFF,
                           (length >> 8) & 0xFF, length & 0xFF};
  size_t written = fwrite(len_header, 1, 4, f);
  if (written != 4) {
    ESP_LOGE(TAG, "Failed to write length header");
    fclose(f);
    unlink(filepath);
    return ESP_FAIL;
  }

  // Write actual data
  written = fwrite(data, 1, length, f);

  if (written != length) {
    ESP_LOGE(TAG, "Failed to write complete data (%zu/%u bytes)", written,
             length);
    fclose(f);
    unlink(filepath);
    return ESP_FAIL;
  }

  // CRITICAL: Flush and sync BEFORE closing file!
  fflush(f);        // Flush stdio buffer to kernel
  fsync(fileno(f)); // Sync kernel buffer to storage
  fclose(f);        // Now safe to close

  // Update oldest file number if this is the first file
  if (g_oldest_file_num == 0 || g_file_counter < g_oldest_file_num) {
    g_oldest_file_num = g_file_counter;
  }

  g_file_counter++;

  ESP_LOGI(TAG, "Saved successfully. Total files: %lu",
           sd_card_get_file_count());
  return ESP_OK;
}

esp_err_t sd_card_read_oldest(uint8_t *buffer, uint32_t *length,
                              uint32_t buffer_size) {
  if (!g_sd_mounted) {
    ESP_LOGE(TAG, "SD card not mounted");
    return ESP_FAIL;
  }

  if (buffer == NULL || length == NULL) {
    ESP_LOGE(TAG, "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  // Find oldest file
  char filepath[SD_MAX_FILENAME_LEN];
  esp_err_t ret = find_oldest_file(filepath, sizeof(filepath));
  if (ret != ESP_OK) {
    *length = 0;
    return ESP_ERR_NOT_FOUND;
  }

  ESP_LOGI(TAG, "Reading oldest file: %s", filepath);

  // Open file for reading
  FILE *f = fopen(filepath, "rb");
  if (f == NULL) {
    ESP_LOGE(TAG, "Failed to open file: %s", filepath);
    *length = 0;
    return ESP_FAIL;
  }

  // Read length header (4 bytes)
  uint8_t len_header[4];
  size_t read_bytes = fread(len_header, 1, 4, f);
  if (read_bytes != 4) {
    ESP_LOGE(TAG, "Failed to read length header from %s", filepath);
    fclose(f);

    // AUTO-DELETE CORRUPT FILE
    ESP_LOGW(TAG, "Deleting corrupt file: %s", filepath);
    if (unlink(filepath) == 0) {
      ESP_LOGI(TAG, "Corrupt file deleted successfully");
      scan_and_update_counters();
    }

    *length = 0;
    return ESP_FAIL;
  }

  uint32_t data_length = (len_header[0] << 24) | (len_header[1] << 16) |
                         (len_header[2] << 8) | len_header[3];

  // Check buffer size
  if (data_length > buffer_size) {
    ESP_LOGE(TAG, "Buffer too small (%u needed, %u available)", data_length,
             buffer_size);
    fclose(f);
    *length = 0;
    return ESP_ERR_NO_MEM;
  }

  // Read actual data
  read_bytes = fread(buffer, 1, data_length, f);
  fclose(f);

  if (read_bytes != data_length) {
    ESP_LOGE(TAG, "Failed to read complete data (%zu/%u bytes)", read_bytes,
             data_length);
    *length = 0;
    return ESP_FAIL;
  }

  *length = data_length;
  ESP_LOGI(TAG, "Read %u bytes successfully", data_length);

  return ESP_OK;
}

esp_err_t sd_card_delete_oldest(void) {
  if (!g_sd_mounted) {
    ESP_LOGE(TAG, "SD card not mounted");
    return ESP_FAIL;
  }

  // Find oldest file
  char filepath[SD_MAX_FILENAME_LEN];
  esp_err_t ret = find_oldest_file(filepath, sizeof(filepath));
  if (ret != ESP_OK) {
    return ESP_ERR_NOT_FOUND;
  }

  ESP_LOGI(TAG, "Deleting oldest file: %s", filepath);

  // Delete the file
  if (unlink(filepath) != 0) {
    ESP_LOGE(TAG, "Failed to delete file: %s", filepath);
    return ESP_FAIL;
  }

  // Update oldest file number by scanning directory
  scan_and_update_counters();

  ESP_LOGI(TAG, "Deleted successfully. Remaining files: %lu",
           sd_card_get_file_count());
  return ESP_OK;
}

esp_err_t sd_card_get_oldest_file_path(char *filepath, size_t max_len) {
  if (!g_sd_mounted) {
    return ESP_FAIL;
  }
  return find_oldest_file(filepath, max_len);
}

bool sd_card_has_data(void) { return (sd_card_get_file_count() > 0); }

uint32_t sd_card_get_file_count(void) {
  if (!g_sd_mounted) {
    return 0;
  }

  DIR *dir = opendir(SD_DATA_DIR);
  if (dir == NULL) {
    ESP_LOGW(TAG, "Failed to open queue directory");
    return 0;
  }

  uint32_t count = 0;
  struct dirent *entry;
  size_t prefix_len = strlen(SD_FILE_PREFIX);

  while ((entry = readdir(dir)) != NULL) {
    // Check if filename starts with prefix
    if (strncmp(entry->d_name, SD_FILE_PREFIX, prefix_len) == 0) {
      count++;
    }
  }

  closedir(dir);
  return count;
}

esp_err_t sd_card_get_info(uint64_t *total_bytes, uint64_t *used_bytes) {
  if (!g_sd_mounted) {
    ESP_LOGE(TAG, "SD card not mounted");
    return ESP_FAIL;
  }

  FATFS *fs;
  DWORD free_clusters;

  // Get volume information
  if (f_getfree("0:", &free_clusters, &fs) != FR_OK) {
    ESP_LOGE(TAG, "Failed to get filesystem info");
    return ESP_FAIL;
  }

  uint64_t total_sectors = (fs->n_fatent - 2) * fs->csize;
  uint64_t free_sectors = free_clusters * fs->csize;
  uint64_t sector_size = FF_MAX_SS;

  if (total_bytes != NULL) {
    *total_bytes = total_sectors * sector_size;
  }

  if (used_bytes != NULL) {
    *used_bytes = (total_sectors - free_sectors) * sector_size;
  }

  return ESP_OK;
}

bool sd_card_is_mounted(void) { return g_sd_mounted; }

// ===== Private Helper Functions =====

static esp_err_t scan_and_update_counters(void) {
  DIR *dir = opendir(SD_DATA_DIR);
  if (dir == NULL) {
    ESP_LOGW(TAG, "Failed to open queue directory for scanning");
    return ESP_FAIL;
  }

  struct dirent *entry;
  size_t prefix_len = strlen(SD_FILE_PREFIX);
  uint32_t max_num = 0;
  uint32_t min_num = UINT32_MAX;
  bool found_files = false;

  while ((entry = readdir(dir)) != NULL) {
    // Check if filename starts with prefix
    if (strncmp(entry->d_name, SD_FILE_PREFIX, prefix_len) == 0) {
      uint32_t file_num = extract_file_number(entry->d_name);
      if (file_num != UINT32_MAX) {
        found_files = true;
        if (file_num > max_num) {
          max_num = file_num;
        }
        if (file_num < min_num) {
          min_num = file_num;
        }
      }
    }
  }

  closedir(dir);

  if (found_files) {
    g_file_counter = max_num + 1;
    g_oldest_file_num = min_num;
    ESP_LOGI(TAG, "Scanned files: oldest=%lu, next=%lu", g_oldest_file_num,
             g_file_counter);
  } else {
    g_file_counter = 0;
    g_oldest_file_num = 0;
    ESP_LOGI(TAG, "No existing files found");
  }

  return ESP_OK;
}

static esp_err_t find_oldest_file(char *filename, size_t max_len) {
  DIR *dir = opendir(SD_DATA_DIR);
  if (dir == NULL) {
    ESP_LOGW(TAG, "Failed to open queue directory");
    return ESP_FAIL;
  }

  struct dirent *entry;
  size_t prefix_len = strlen(SD_FILE_PREFIX);
  uint32_t min_num = UINT32_MAX;
  char oldest_name[64] = {0};
  bool found = false;

  while ((entry = readdir(dir)) != NULL) {
    // Check if filename starts with prefix
    if (strncmp(entry->d_name, SD_FILE_PREFIX, prefix_len) == 0) {
      uint32_t file_num = extract_file_number(entry->d_name);
      if (file_num < min_num) {
        min_num = file_num;
        strncpy(oldest_name, entry->d_name, sizeof(oldest_name) - 1);
        found = true;
      }
    }
  }

  closedir(dir);

  if (!found) {
    return ESP_ERR_NOT_FOUND;
  }

  // Build full path
  snprintf(filename, max_len, "%s/%s", SD_DATA_DIR, oldest_name);
  g_oldest_file_num = min_num;

  return ESP_OK;
}

static uint32_t extract_file_number(const char *filename) {
  // Extract number from filename like "pkt_00000123.dat"
  const char *num_start = filename + strlen(SD_FILE_PREFIX);
  char *endptr;
  unsigned long num = strtoul(num_start, &endptr, 10);

  if (endptr == num_start) {
    return UINT32_MAX; // Parse error
  }

  return (uint32_t)num;
}
