/**
 * @file storage_handler.h
 * @brief Storage Handler - Thread-safe wrapper with 100KB batch buffering
 */

#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize storage handler with 100KB buffer
 */
esp_err_t storage_handler_init(void);

/**
 * Deinitialize storage handler and flush pending data
 */
esp_err_t storage_handler_deinit(void);

/**
 * Save data to storage (buffered, non-blocking)
 */
esp_err_t storage_handler_save(const uint8_t *data, uint16_t length);

/**
 * Flush batch buffer to SD card immediately
 */
esp_err_t storage_handler_flush(void);

/**
 * Read oldest data from SD card
 */
esp_err_t storage_handler_read_oldest(uint8_t *buffer, uint32_t *length,
                                      uint32_t buffer_size);

/**
 * Delete oldest data file from SD card
 */
esp_err_t storage_handler_delete_oldest(void);

/**
 * @brief Prepare for retry session (open oldest file)
 * @return ESP_OK if file opened, ESP_ERR_NOT_FOUND if no files
 */
esp_err_t storage_handler_prepare_retry(void);

/**
 * @brief Get next packet from current open file (stream read mode)
 * @param buffer Output buffer
 * @param length Output length pointer
 * @param max_len Max buffer size
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND on EOF
 */
esp_err_t storage_handler_get_next_packet(uint8_t *buffer, uint16_t *length,
                                          uint16_t max_len);

/**
 * @brief Finish retry session
 * @param success If true, delete the file. If false, keep for later.
 */
void storage_handler_finish_retry(bool success);

/**
 * Check if storage has data (in buffer or SD card)
 */
bool storage_handler_has_data(void);

/**
 * Get total file count (includes pending buffer packets + SD files)
 */
uint32_t storage_handler_get_file_count(void);

#ifdef __cplusplus
}
#endif

#endif // STORAGE_HANDLER_H