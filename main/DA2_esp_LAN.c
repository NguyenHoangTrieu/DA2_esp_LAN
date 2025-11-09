/*
* ESP32 S3 LAN Application
*/

#include "rbg_handler.h"
#include "config_handler.h"
#include "mcu_wan_handler.h"
#include "fota_lan_handler.h"

static const char *TAG = "MAIN APP";

TaskHandle_t main_task_handle = NULL;



// =============================================================================
// Main Application Entry Point
// =============================================================================

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "USB host library example");
    init_led_strip();
    led_on();
    main_task_handle = xTaskGetCurrentTaskHandle();
    // CREATE DEFAULT EVENT LOOP FIRST (if not already created)
    esp_err_t ret = esp_event_loop_create_default();
    if (ret == ESP_ERR_INVALID_STATE) {
        // Event loop already created, this is OK
        ESP_LOGW(TAG, "Default event loop already exists");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create default event loop: %s", esp_err_to_name(ret));
        return;
    }
    config_handler_task_start();
    mcu_wan_handler_start();
    // fota_lan_handler_task_start();
    while (1) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(500));
        led_show_green();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}