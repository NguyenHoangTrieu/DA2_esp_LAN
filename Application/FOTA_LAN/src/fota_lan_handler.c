/*
 * Advanced FOTA Handler for LAN MCU (Using eppp_link)
 * 
 * This module manages over-the-air firmware updates for the LAN MCU
 * via PPP connection (eppp_link) to the WAN MCU.
 * Based on: https://github.com/espressif/esp-protocols/tree/master/components/eppp_link/examples/host
 */

#include "fota_lan_handler.h"

static const char *TAG = "fota_lan";

/* eppp_link Client State */
static esp_netif_t *s_eppp_netif = NULL;
static bool s_ppp_connected = false;
static bool s_ota_in_progress = false;

/* Event Group for synchronization */
static EventGroupHandle_t s_event_group = NULL;
#define PPP_CONNECTED_BIT BIT0
#define OTA_TRIGGER_BIT BIT1

/* Function Prototypes */
static void fota_lan_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data);
static esp_err_t fota_lan_connect_ppp(void);
static void fota_lan_ota_event_handler(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data);
static void fota_lan_print_sha256(const uint8_t *image_hash, const char *label);
static void fota_lan_get_sha256_of_partitions(void);
static esp_err_t fota_lan_validate_image_header(esp_app_desc_t *new_app_info);
static void fota_lan_advanced_ota_task(void *pvParameter);
static void fota_lan_control_task(void *pvParameters);

/**
 * @brief Event handler for network events
 */
static void fota_lan_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_PPP_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "PPP Got IP Address");
            ESP_LOGI(TAG, "IP       : " IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Netmask  : " IPSTR, IP2STR(&event->ip_info.netmask));
            ESP_LOGI(TAG, "Gateway  : " IPSTR, IP2STR(&event->ip_info.gw));
            
            s_ppp_connected = true;
            xEventGroupSetBits(s_event_group, PPP_CONNECTED_BIT);
        }
        else if (event_id == IP_EVENT_PPP_LOST_IP) {
            ESP_LOGW(TAG, "PPP Lost IP");
            s_ppp_connected = false;
            xEventGroupClearBits(s_event_group, PPP_CONNECTED_BIT);
        }
    }
}

/**
 * @brief Connect to PPP server using eppp_link
 */
static esp_err_t fota_lan_connect_ppp(void)
{
    ESP_LOGI(TAG, "Connecting to PPP server via eppp_link...");

    // Configure eppp for UART transport (client mode)
    eppp_config_t config = EPPP_DEFAULT_CLIENT_CONFIG();
    
#if CONFIG_EPPP_LINK_DEVICE_UART
    config.transport = EPPP_TRANSPORT_UART;
    config.uart.tx_io = FOTA_LAN_UART_TX_PIN;
    config.uart.rx_io = FOTA_LAN_UART_RX_PIN;
    config.uart.baud = FOTA_LAN_UART_BAUD_RATE;
#else
    #error "UART transport must be enabled in menuconfig"
#endif

    // Connect to PPP server
    s_eppp_netif = eppp_connect(&config);
    
    if (s_eppp_netif == NULL) {
        ESP_LOGE(TAG, "Failed to connect to PPP server");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connected to PPP server successfully");
    return ESP_OK;
}

/**
 * @brief Print SHA256 hash
 */
static void fota_lan_print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[FOTA_LAN_HASH_LEN * 2 + 1];
    hash_print[FOTA_LAN_HASH_LEN * 2] = 0;
    for (int i = 0; i < FOTA_LAN_HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

/**
 * @brief Get SHA256 of partitions
 */
static void fota_lan_get_sha256_of_partitions(void)
{
    uint8_t sha_256[FOTA_LAN_HASH_LEN] = {0};
    esp_partition_t partition;

    // Get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    fota_lan_print_sha256(sha_256, "SHA-256 for bootloader:");

    // Get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    fota_lan_print_sha256(sha_256, "SHA-256 for current firmware:");
}

/**
 * @brief Validate image header
 */
static esp_err_t fota_lan_validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }

#if !FOTA_LAN_SKIP_VERSION_CHECK
    if (memcmp(new_app_info->version, running_app_info.version,
               sizeof(new_app_info->version)) == 0) {
        ESP_LOGW(TAG, "Current running version is the same as new. Update cancelled.");
        return ESP_FAIL;
    }
#endif

#if FOTA_LAN_ENABLE_ANTI_ROLLBACK
    const uint32_t hw_sec_version = esp_efuse_read_secure_version();
    if (new_app_info->secure_version < hw_sec_version) {
        ESP_LOGW(TAG, "New firmware security version is less than eFuse programmed");
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

/**
 * @brief OTA event handler
 */
static void fota_lan_ota_event_handler(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data)
{
    if (event_base == ESP_HTTPS_OTA_EVENT) {
        switch (event_id) {
            case ESP_HTTPS_OTA_START:
                ESP_LOGI(TAG, "OTA started");
                break;
            case ESP_HTTPS_OTA_CONNECTED:
                ESP_LOGI(TAG, "Connected to OTA server");
                break;
            case ESP_HTTPS_OTA_GET_IMG_DESC:
                ESP_LOGI(TAG, "Reading Image Description");
                break;
            case ESP_HTTPS_OTA_WRITE_FLASH:
                ESP_LOGD(TAG, "Writing to flash: %d bytes", *(int *)event_data);
                break;
            case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                ESP_LOGI(TAG, "Boot partition updated");
                break;
            case ESP_HTTPS_OTA_FINISH:
                ESP_LOGI(TAG, "OTA finish");
                break;
            case ESP_HTTPS_OTA_ABORT:
                ESP_LOGI(TAG, "OTA abort");
                break;
            default:
                break;
        }
    }
}

/**
 * @brief Advanced OTA task
 */
static void fota_lan_advanced_ota_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting Advanced OTA for LAN MCU");
    s_ota_in_progress = true;
    esp_err_t err;
    esp_err_t ota_finish_err = ESP_OK;

    // Get the eppp interface name for binding
    char if_name[6];
    esp_netif_get_netif_impl_name(s_eppp_netif, if_name);
    ESP_LOGI(TAG, "Bind interface name is %s", if_name);

    // Configure HTTP client bound to eppp interface
    esp_http_client_config_t config = {
        .url = FOTA_LAN_FIRMWARE_UPGRADE_URL,
#if FOTA_LAN_USE_CERT_BUNDLE
        .crt_bundle_attach = esp_crt_bundle_attach,
#endif
        .timeout_ms = FOTA_LAN_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
        .buffer_size = FOTA_LAN_HTTP_BUFFER_SIZE,
        .buffer_size_tx = FOTA_LAN_HTTP_BUFFER_SIZE_TX,
        .if_name = if_name,  // Bind to eppp interface
#if FOTA_LAN_SKIP_COMMON_NAME_CHECK
        .skip_cert_common_name_check = true,
#endif
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);

    // Register OTA event handler
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID,
                                               &fota_lan_ota_event_handler, NULL));

    // Begin OTA
    esp_https_ota_handle_t https_ota_handle = NULL;
    err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        goto ota_fail;
    }

    // Get and validate image description
    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_get_img_desc failed");
        goto ota_end;
    }

    err = fota_lan_validate_image_header(&app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "image header verification failed");
        goto ota_end;
    }

    // Perform OTA update
    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        ESP_LOGD(TAG, "Image bytes read: %d",
                 esp_https_ota_get_image_len_read(https_ota_handle));
    }

    // Check if complete data was received
    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        ESP_LOGE(TAG, "Complete data was not received");
        goto ota_end;
    }

    // Finish OTA
    ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
        ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    } else {
        if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
        goto ota_fail;
    }

ota_end:
    esp_https_ota_abort(https_ota_handle);

ota_fail:
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
    s_ota_in_progress = false;
    vTaskDelete(NULL);
}

/**
 * @brief Main control task - listens for PPP connection events
 */
static void fota_lan_control_task(void *pvParameters)
{
    while (1) {
        // Wait for OTA trigger - in this case, just wait for connection
        EventBits_t bits = xEventGroupWaitBits(s_event_group, 
                                              OTA_TRIGGER_BIT,
                                              pdTRUE,
                                              pdFALSE,
                                              portMAX_DELAY);

        if (bits & OTA_TRIGGER_BIT) {
            ESP_LOGI(TAG, "OTA trigger received via eppp");

            // Wait for PPP connection with timeout
            bits = xEventGroupWaitBits(s_event_group, 
                                      PPP_CONNECTED_BIT,
                                      pdFALSE,
                                      pdFALSE,
                                      pdMS_TO_TICKS(FOTA_LAN_EPPP_CONNECT_TIMEOUT_MS));

            if (bits & PPP_CONNECTED_BIT) {
                ESP_LOGI(TAG, "PPP connected, starting OTA task");
                xTaskCreate(&fota_lan_advanced_ota_task, "fota_lan_ota_task",
                           FOTA_LAN_TASK_STACK_SIZE, NULL,
                           FOTA_LAN_TASK_PRIORITY, NULL);
            } else {
                ESP_LOGE(TAG, "PPP connection timeout");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Initialize the FOTA LAN handler
 */
void fota_lan_init(void)
{
    ESP_LOGI(TAG, "Initializing FOTA LAN handler with eppp_link");

    // Create event group
    s_event_group = xEventGroupCreate();
    if (!s_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    // Print SHA256 of partitions
    fota_lan_get_sha256_of_partitions();

    // Register event handler for PPP events
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                              &fota_lan_event_handler, NULL));

    // Connect to PPP server via eppp_link
    esp_err_t err = fota_lan_connect_ppp();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to PPP server");
        vEventGroupDelete(s_event_group);
        s_event_group = NULL;
        return;
    }

    // Signal OTA trigger bit to start OTA task
    xEventGroupSetBits(s_event_group, OTA_TRIGGER_BIT);

    // Start control task
    xTaskCreate(fota_lan_control_task, "fota_lan_control",
               4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "FOTA LAN handler initialized successfully with eppp_link");
}

/**
 * @brief Deinitialize the FOTA LAN handler
 */
void fota_lan_deinit(void)
{
    if (s_eppp_netif) {
        eppp_disconnect(s_eppp_netif);
        s_eppp_netif = NULL;
    }

    if (s_event_group) {
        vEventGroupDelete(s_event_group);
        s_event_group = NULL;
    }

    ESP_LOGI(TAG, "FOTA LAN handler deinitialized");
}
