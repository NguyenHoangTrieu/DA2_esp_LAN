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

/**
 * @brief Load JSON config and initialize handler
 *
 * Called by Config Handler when receiving BL:JSON command
 *
 * @param stack_id Stack ID (0 or 1)
 * @param json_str JSON config string
 * @param json_len JSON string length
 * @return esp_err_t
 *         - ESP_OK: Config loaded successfully
 *         - ESP_ERR_INVALID_ARG: Invalid arguments
 *         - ESP_ERR_NO_MEM: Out of memory
 *         - ESP_FAIL: JSON parsing or handler initialization failed
 */
esp_err_t module_monitor_load_config(uint8_t stack_id, const char *json_str, uint16_t json_len);

/**
 * @brief Get module type for a stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return module_type_t Module type detected
 */
module_type_t module_monitor_get_stack_type(uint8_t stack_id);

/**
 * @brief Check if stack has valid config
 *
 * @param stack_id Stack ID (0 or 1)
 * @return true if config is loaded and valid
 */
bool module_monitor_is_configured(uint8_t stack_id);

/**
 * @brief Start handler task for a stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return esp_err_t
 *         - ESP_OK: Handler started successfully
 *         - ESP_ERR_INVALID_STATE: Module not configured
 *         - ESP_FAIL: Handler start failed
 */
esp_err_t module_monitor_start_handler(uint8_t stack_id);

/**
 * @brief Stop handler task for a stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return esp_err_t
 *         - ESP_OK: Handler stopped successfully
 *         - ESP_ERR_INVALID_STATE: Handler not running
 */
esp_err_t module_monitor_stop_handler(uint8_t stack_id);

/**
 * @brief Get handler status
 *
 * @param stack_id Stack ID (0 or 1)
 * @return handler_status_t Current handler status
 */
handler_status_t module_monitor_get_handler_status(uint8_t stack_id);

/**
 * @brief Get module info for a stack
 *
 * @param stack_id Stack ID (0 or 1)
 * @return Pointer to module_info_t (do not free)
 */
const module_info_t* module_monitor_get_info(uint8_t stack_id);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_MONITOR_TASK_H */
