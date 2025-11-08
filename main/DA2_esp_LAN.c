/*
* ESP32 S3 LAN Application
*/

#include "rbg_handler.h"
#include "config_handler.h"
#include "mcu_wan_handler.h"

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
    mcu_wan_handler_start();
    config_handler_task_start();

    while (1) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(500));
        led_show_green();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}