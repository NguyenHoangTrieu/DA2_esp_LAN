/*
* ESP32 S3 LAN Application
*/

#include "DA2_esp_LAN.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

static const char *TAG = "MAIN APP";

TaskHandle_t main_task_handle = NULL;

// Transport Selection
#define PPP_USE_UART_TRANSPORT         1

// UART Configuration
#define PPP_UART_PORT                  UART_NUM_0
#define PPP_UART_TX_PIN                GPIO_NUM_43
#define PPP_UART_RX_PIN                GPIO_NUM_44
#define PPP_UART_BAUDRATE              256000
#define PPP_UART_QUEUE_SIZE            40
#define PPP_UART_RX_BUFFER_SIZE        (32*1024)

// // Global DNS Server (8.8.8.8)
// #define PPP_GLOBAL_DNS                 0x08080808

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "LAN MCU Application Starting... V1.0.1");
    main_task_handle = xTaskGetCurrentTaskHandle();

     // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    init_led_strip();
    led_on();
    ESP_ERROR_CHECK(i2c_dev_support_init());
    ESP_ERROR_CHECK(tca_init());
    
    ESP_ERROR_CHECK(stack_handler_init());
    ESP_LOGI(TAG, "Stack handler initialized");
    
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
                ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(bt_ret));
            } else {
                bt_ret = esp_bluedroid_init();
                if (bt_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(bt_ret));
                } else {
                    bt_ret = esp_bluedroid_enable();
                    if (bt_ret != ESP_OK) {
                        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(bt_ret));
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

    // Start Module Monitor Task - Module Base Setting Core
    ESP_ERROR_CHECK(module_monitor_task_start());
    ESP_LOGI(TAG, "Module Monitor Task started (Module Base Setting enabled)");

    /* Restore BLE config from NVS if it was configured before last reboot.
     * This replaces the old eager ble_gatt_handler_init() call — the handler
     * now self-initializes the BT stack so timing no longer matters. */
    config_restore_ble_from_nvs();

    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lan_ppp_connect(void) {
  // Initialize networking
  ESP_ERROR_CHECK(esp_netif_init());
  eppp_config_t config = EPPP_DEFAULT_CLIENT_CONFIG();
  config.transport = EPPP_TRANSPORT_UART;
  config.uart.port = PPP_UART_PORT;
  config.uart.tx_io = PPP_UART_TX_PIN;
  config.uart.rx_io = PPP_UART_RX_PIN;
  config.uart.baud = PPP_UART_BAUDRATE;
  config.uart.rx_buffer_size = PPP_UART_RX_BUFFER_SIZE;
  config.uart.queue_size = PPP_UART_QUEUE_SIZE;

  esp_netif_t *eppp_netif = eppp_connect(&config);

  // Get IP info
  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(eppp_netif, &ip_info) == ESP_OK) {
    ESP_LOGI(TAG, "IP:      " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info.gw));
  }

  // Setup DNS
  esp_netif_dns_info_t dns;
  dns.ip.u_addr.ip4.addr = esp_netif_htonl(PPP_GLOBAL_DNS);
  dns.ip.type = ESP_IPADDR_TYPE_V4;
  ESP_ERROR_CHECK(esp_netif_set_dns_info(eppp_netif, ESP_NETIF_DNS_MAIN, &dns));
  ESP_LOGI(TAG, "DNS:     " IPSTR, IP2STR(&dns.ip.u_addr.ip4));

  vTaskDelay(1000);
}