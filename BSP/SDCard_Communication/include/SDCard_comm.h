/**
 * @file SDCard_comm.h
 * @brief SD Card Communication Handler - FIFO Queue Storage for Data Backup
 *
 * Provides API for saving/reading data packets to/from SD card
 * using SDMMC interface with FAT filesystem.
 */

#ifndef SDCARD_COMM_H
#define SDCARD_COMM_H

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Configuration =====
#define SD_MOUNT_POINT "/sdcard"
#define SD_DATA_DIR "/sdcard/QUEUE"
#define SD_FILE_PREFIX "PKT_"
#define SD_MAX_FILES 1000
#define SD_MAX_FILENAME_LEN 64

// ===== SD Card Configuration Structure =====
typedef struct {
  int gpio_clk;          // CLK GPIO pin
  int gpio_cmd;          // CMD GPIO pin
  int gpio_d0;           // D0 GPIO pin
  int gpio_d1;           // D1 GPIO pin (-1 if 1-bit mode)
  int gpio_d2;           // D2 GPIO pin (-1 if 1-bit mode)
  int gpio_d3;           // D3 GPIO pin (-1 if 1-bit mode)
  bool format_if_failed; // Format card if mount fails
  uint8_t max_files;     // Max open files
  bool bus_width_4;      // true=4-bit, false=1-bit
} sd_card_config_t;

// ===== Default Configuration Macro =====
#define SD_CARD_CONFIG_DEFAULT()                                               \
  {.gpio_clk = GPIO_NUM_7,                                                     \
   .gpio_cmd = GPIO_NUM_6,                                                     \
   .gpio_d0 = GPIO_NUM_8,                                                      \
   .gpio_d1 = GPIO_NUM_3,                                                      \
   .gpio_d2 = GPIO_NUM_4,                                                      \
   .gpio_d3 = GPIO_NUM_5,                                                      \
   .format_if_failed = false,                                                  \
   .max_files = 10,                                                            \
   .bus_width_4 = false}

// ===== Public API =====

/**
 * @brief Initialize SD card with SDMMC interface
 *
 * @param config Pointer to SD card configuration
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t sd_card_init(const sd_card_config_t *config);

/**
 * @brief Deinitialize and unmount SD card
 *
 * @return ESP_OK on success
 */
esp_err_t sd_card_deinit(void);

/**
 * @brief Save data packet to SD card as a new file
 *
 * Creates a file with sequential naming: pkt_NNNNNNNN.dat
 *
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return ESP_OK on success, ESP_ERR_NO_MEM if full, ESP_FAIL on error
 */
esp_err_t sd_card_save(const uint8_t *data, uint32_t length);

/**
 * @brief Read oldest data packet from SD card
 *
 * Reads the file with lowest sequence number.
 *
 * @param buffer Buffer to store read data
 * @param length Pointer to store actual bytes read
 * @param buffer_size Maximum size of buffer
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no files, ESP_FAIL on error
 */
esp_err_t sd_card_read_oldest(uint8_t *buffer, uint32_t *length,
                              uint32_t buffer_size);

/**
 * @brief Delete oldest data packet from SD card
 *
 * Deletes the file with lowest sequence number.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no files
 */
esp_err_t sd_card_delete_oldest(void);

/**
 * @brief Check if SD card has any data files
 *
 * @return true if files exist, false otherwise
 */
bool sd_card_has_data(void);

/**
 * @brief Get number of data files on SD card
 *
 * @return Number of packet files in queue directory
 */
uint32_t sd_card_get_file_count(void);

/**
 * @brief Get SD card size and usage information
 *
 * @param total_bytes Pointer to store total card size in bytes (can be NULL)
 * @param used_bytes Pointer to store used space in bytes (can be NULL)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t sd_card_get_info(uint64_t *total_bytes, uint64_t *used_bytes);

/**
 * @brief Check if SD card is initialized and mounted
 *
 * @return true if mounted, false otherwise
 */
bool sd_card_is_mounted(void);

#ifdef __cplusplus
}
#endif

#endif // SDCARD_COMM_H
