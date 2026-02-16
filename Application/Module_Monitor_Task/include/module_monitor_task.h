/**
 * @file module_monitor_task.h
 * @brief Module Monitor Task - Manages lifecycle of module handlers
 */

#ifndef MODULE_MONITOR_TASK_H
#define MODULE_MONITOR_TASK_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Type Definitions ===== */

/**
 * @brief Module types
 */
typedef enum {
    MODULE_TYPE_NONE = 0,      ///< No module detected
    MODULE_TYPE_BLE = 1,       ///< BLE module
    MODULE_TYPE_ZIGBEE = 2,    ///< Zigbee module
    MODULE_TYPE_LORA = 3,      ///< LoRa module
    MODULE_TYPE_UNKNOWN = 0xFF ///< Unknown module type
} module_type_t;

/**
 * @brief Handler task status
 */
typedef enum {
    HANDLER_STATUS_STOPPED = 0,   ///< Task not running
    HANDLER_STATUS_STARTING = 1,  ///< Task is starting
    HANDLER_STATUS_RUNNING = 2,   ///< Task running normally
    HANDLER_STATUS_STOPPING = 3,  ///< Task is stopping
    HANDLER_STATUS_ERROR = 0xFF   ///< Task encountered error
} handler_status_t;

/**
 * @brief Module information structure (per stack)
 */
typedef struct {
    uint8_t stack_id;                    ///< Stack ID (0 or 1)
    module_type_t module_type;           ///< Detected module type
    bool is_configured;                  ///< Has JSON config loaded
    bool is_running;                     ///< Handler task is running
    handler_status_t handler_status;     ///< Handler task status
    void *config_data;                   ///< Pointer to parsed config
    char *json_config_str;               ///< JSON config string (malloc'd)
    uint16_t json_config_len;            ///< JSON config length
} module_info_t;

/* ===== Public APIs ===== */

/**
 * @brief Start module monitor task
 *
 * - Check NVS for saved JSON config
 * - If no config, enter waiting state
 * - Listen for JSON config from Config Handler
 * - Auto-start appropriate handler tasks
 *
 * @return esp_err_t
 *         - ESP_OK: Monitor task started successfully
 *         - ESP_ERR_NO_MEM: Out of memory
 *         - ESP_FAIL: Failed to create task
 */
esp_err_t module_monitor_task_start(void);

/**
 * @brief Stop module monitor task
 *
 * - Stop all running handler tasks
 * - Cleanup resources
 * - Cleanup queues
 *
 * @return esp_err_t
 *         - ESP_OK: Monitor task stopped successfully
 *         - ESP_ERR_INVALID_STATE: Task not running
 */
esp_err_t module_monitor_task_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_MONITOR_TASK_H */
