/*
 * ESP32 S3 LAN Application
 */

#include "DA2_esp_LAN.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "module_config_controller.h"
#include "bench_counter.h"
#include "bench_throughput.h"
#include "bench_time_sync.h"
#include <esp_pm.h>

static const char *TAG = "MAIN APP";

TaskHandle_t main_task_handle = NULL;

/**
 * @brief Main application entry point
 */
void app_main(void) {
  ESP_LOGI(TAG, "LAN MCU Application Starting... V%s", DA2_CURRENT_VERSION_STR);
  main_task_handle = xTaskGetCurrentTaskHandle();

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_ERROR_CHECK(i2c_dev_support_init());
  ESP_ERROR_CHECK(tca_init());

  ESP_ERROR_CHECK(stack_handler_init());
  ESP_LOGI(TAG, "Stack handler initialized");

  // Initialize module config controller ONCE at system boot (shared across all
  // handlers)
  ESP_ERROR_CHECK(module_config_controller_init());
  ESP_LOGI(TAG, "Module config controller initialized");
  stack_handler_gpio_set_direction(0, STACK_GPIO_PIN_04, true);
  stack_handler_gpio_set_direction(1, STACK_GPIO_PIN_04, true);
  stack_handler_gpio_write(0, STACK_GPIO_PIN_04, true); // ADAPTER POWER ON
  stack_handler_gpio_write(1, STACK_GPIO_PIN_04, true); // ADAPTER POWER ON
  config_init();
  config_handler_task_start();
  ESP_LOGI(TAG, "Config handler started");

  /* Initialize BT controller and Bluedroid early at boot while internal RAM
   * is fresh and unfragmented.  Lazy init after 600+ seconds of uptime
   * causes bt_workqueue allocation failures (internal RAM fragmented).
   * Both BLE Mesh (Native) and GATT Central share this single BT stack. */
  {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t bt_ret = esp_bt_controller_init(&bt_cfg);
    if (bt_ret != ESP_OK) {
      ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(bt_ret));
    } else {
      bt_ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
      if (bt_ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s",
                 esp_err_to_name(bt_ret));
      } else {
        bt_ret = esp_bluedroid_init();
        if (bt_ret != ESP_OK) {
          ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(bt_ret));
        } else {
          bt_ret = esp_bluedroid_enable();
          if (bt_ret != ESP_OK) {
            ESP_LOGE(TAG, "Bluedroid enable failed: %s",
                     esp_err_to_name(bt_ret));
          } else {
            ESP_LOGI(TAG, "BT stack initialized at boot");
          }
        }
      }
    }
  }

  // Start WAN handler FIRST so its queue/mutexes are allocated before
  // module handlers (BLE/LoRa/etc.) consume internal RAM on NVS restore.
  mcu_wan_handler_start();
  ESP_LOGI(TAG, "MCU WAN handler started");

  /* Start inter-MCU throughput benchmark (requires WAN handler uplink queue) */
  if (bench_throughput_start() != ESP_OK) {
    ESP_LOGW(TAG, "MCU throughput benchmark start failed (non-fatal)");
  }
  ESP_LOGI(TAG, "MCU throughput benchmark started");

  /* Inter-MCU µs-level time sync (no-op when BENCH_TIME_SYNC_ENABLE = 0) */
  if (bench_time_sync_init() != ESP_OK) {
    ESP_LOGW(TAG, "Bench time-sync init failed (non-fatal)");
  } else {
    ESP_LOGI(TAG, "Bench time-sync init OK");
  }

  /* Start benchmark counter task after WAN handler so uplink queue exists */
  if (bench_task_start() != ESP_OK) {
    ESP_LOGW(TAG, "Bench counter task start failed (non-fatal)");
  }
  ESP_LOGI(TAG, "Benchmark counter task started");

  // Start Module Monitor Task - Module Base Setting Core
  ESP_ERROR_CHECK(module_monitor_task_start());
  ESP_LOGI(TAG, "Module Monitor Task started (Module Base Setting enabled)");

  /* Restore BLE config from NVS if it was configured before last reboot.
   * This replaces the old eager ble_gatt_handler_init() call — the handler
   * now self-initializes the BT stack so timing no longer matters. */
  config_restore_ble_from_nvs();

#if CONFIG_PM_ENABLE
  esp_pm_config_t pm_config = {
      .max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true};
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
  ESP_LOGI(TAG, "Automatic Light Sleep & Power Management ENABLED");
#endif

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}