/*
 * Advanced OTA Update Handler for ESP32
 */

#include "fota_lan_handler.h"
#include <net/if.h>

// PPP connection for internet via eppp_link
static esp_netif_t *s_eppp_netif = NULL;

static const char *TAG = "fota_lan_handler";

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static bool ota_task_close = false;

#if CONFIG_ENABLE_OTA_RESUMPTION
#define NVS_NAMESPACE_OTA_RESUMPTION "ota_resumption"
#define NVS_KEY_OTA_WR_LENGTH "nvs_ota_wr_len"
#define NVS_KEY_SAVED_URL "nvs_ota_url"

static esp_err_t ota_res_get_written_len_from_nvs(const nvs_handle_t nvs_handle,
                                                  const char *url,
                                                  uint32_t *nvs_ota_wr_len) {
  esp_err_t err;
  char saved_url[OTA_URL_SIZE] = {0};
  size_t url_len = sizeof(saved_url);

  *nvs_ota_wr_len = 0;

  // Retrieve the saved URL from NVS
  err = nvs_get_str(nvs_handle, NVS_KEY_SAVED_URL, saved_url, &url_len);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGD(TAG, "Saved URL is not initialized yet!");
    return err;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error reading saved URL (%s)", esp_err_to_name(err));
    return err;
  }

  // Compare the current URL with the saved URL
  if (strcmp(url, saved_url) != 0) {
    ESP_LOGD(TAG, "URLs do not match. Restarting OTA from beginning.");
    return ESP_ERR_INVALID_STATE;
  }

  // Fetch the saved write length only if URLs match
  uint16_t saved_wr_len_kb = 0;
  err = nvs_get_u16(nvs_handle, NVS_KEY_OTA_WR_LENGTH, &saved_wr_len_kb);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGD(TAG, "The write length is not initialized yet!");
    *nvs_ota_wr_len = 0;
    return err;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error reading OTA write length (%s)", esp_err_to_name(err));
    return err;
  }

  // Convert the saved value back to bytes
  *nvs_ota_wr_len = saved_wr_len_kb * 1024;

  return ESP_OK;
}

static esp_err_t ota_res_save_cfg_to_nvs(const nvs_handle_t nvs_handle,
                                         int nvs_ota_wr_len, const char *url) {
  // Convert the write length to kilobytes to optimize NVS space
  uint16_t wr_len_kb = nvs_ota_wr_len / 1024;

  // Save the current OTA write length to NVS
  ESP_RETURN_ON_ERROR(nvs_set_u16(nvs_handle, NVS_KEY_OTA_WR_LENGTH, wr_len_kb),
                      TAG, "Failed to set OTA write length");

  // Save the URL only if the OTA write length is non-zero
  if (nvs_ota_wr_len) {
    char saved_url[OTA_URL_SIZE] = {0};
    size_t url_len = sizeof(saved_url);

    esp_err_t err =
        nvs_get_str(nvs_handle, NVS_KEY_SAVED_URL, saved_url, &url_len);
    if (err == ESP_ERR_NVS_NOT_FOUND || strcmp(saved_url, url) != 0) {
      // URL not saved or changed; save it now
      ESP_RETURN_ON_ERROR(nvs_set_str(nvs_handle, NVS_KEY_SAVED_URL, url), TAG,
                          "Failed to set URL in NVS");
    } else if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error reading OTA URL");
      return err;
    }
  }

  ESP_RETURN_ON_ERROR(nvs_commit(nvs_handle), TAG, "Failed to commit NVS");
  ESP_LOGD(TAG, "Saving state in NVS. Total image written: %d KB", wr_len_kb);
  return ESP_OK;
}

static esp_err_t ota_res_cleanup_cfg_from_nvs(nvs_handle_t handle) {
  esp_err_t ret;

  // Erase all keys in the NVS handle and commit changes
  ESP_GOTO_ON_ERROR(nvs_erase_all(handle), err, TAG, "Error in erasing NVS");
  ESP_GOTO_ON_ERROR(nvs_commit(handle), err, TAG, "Error in committing NVS");
  ret = ESP_OK;
err:
  nvs_close(handle);
  return ret;
}
#endif

/* Event handler for catching HTTPS OTA events */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == ESP_HTTPS_OTA_EVENT) {
    switch (event_id) {
    case ESP_HTTPS_OTA_START:
      ESP_LOGI(TAG, "OTA started");
      break;
    case ESP_HTTPS_OTA_CONNECTED:
      ESP_LOGI(TAG, "Connected to server");
      break;
    case ESP_HTTPS_OTA_GET_IMG_DESC:
      ESP_LOGI(TAG, "Reading Image Description");
      break;
    case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
      ESP_LOGI(TAG, "Verifying chip id of new image: %d",
               *(esp_chip_id_t *)event_data);
      break;
    case ESP_HTTPS_OTA_VERIFY_CHIP_REVISION:
      ESP_LOGI(TAG, "Verifying chip revision of new image: %d",
               *(esp_chip_id_t *)event_data);
      break;
    case ESP_HTTPS_OTA_DECRYPT_CB:
      ESP_LOGI(TAG, "Callback to decrypt function");
      break;
    case ESP_HTTPS_OTA_WRITE_FLASH:
      ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
      break;
    case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
      ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d",
               *(esp_partition_subtype_t *)event_data);
      break;
    case ESP_HTTPS_OTA_FINISH:
      ESP_LOGI(TAG, "OTA finish");
      break;
    case ESP_HTTPS_OTA_ABORT:
      ESP_LOGI(TAG, "OTA abort");
      break;
    }
  }
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info) {
  if (new_app_info == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_app_desc_t running_app_info;
  if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
    ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
  }

#if !CONFIG_SKIP_VERSION_CHECK
  if (memcmp(new_app_info->version, running_app_info.version,
             sizeof(new_app_info->version)) == 0) {
    ESP_LOGW(TAG,
             "Current running version is the same as new. Update cancelled.");
    return ESP_FAIL;
  }
#endif

#if CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
  const uint32_t hw_sec_version = esp_efuse_read_secure_version();
  if (new_app_info->secure_version < hw_sec_version) {
    ESP_LOGW(
        TAG,
        "New firmware security version is less than eFuse programmed, %d < %d",
        new_app_info->secure_version, hw_sec_version);
    return ESP_FAIL;
  }
#endif

  return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client) {
  esp_err_t err = ESP_OK;
  return err;
}

static void print_sha256(const uint8_t *image_hash, const char *label) {
  char hash_print[HASH_LEN * 2 + 1];
  hash_print[HASH_LEN * 2] = 0;
  for (int i = 0; i < HASH_LEN; ++i) {
    sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
  }
  ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void) {
  uint8_t sha_256[HASH_LEN] = {0};
  esp_partition_t partition;

  // get sha256 digest for bootloader
  partition.address = ESP_BOOTLOADER_OFFSET;
  partition.size = ESP_PARTITION_TABLE_OFFSET;
  partition.type = ESP_PARTITION_TYPE_APP;
  esp_partition_get_sha256(&partition, sha_256);
  print_sha256(sha_256, "SHA-256 for bootloader:");

  // get sha256 digest for running partition
  esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
  print_sha256(sha_256, "SHA-256 for current firmware:");
}

// Connect to PPP server via eppp_link
static esp_err_t fota_lan_connect_ppp(void)
{
    ESP_LOGI(TAG, "Initializing PPP client connection...");
    
    // Configure eppp client (HOST mode)
    eppp_config_t config = EPPP_DEFAULT_CLIENT_CONFIG();
    
    // Setup UART transport
    config.transport = EPPP_TRANSPORT_UART;
    config.uart.port = FOTA_LAN_UART_PORT;
    config.uart.tx_io = FOTA_LAN_UART_TX_PIN;
    config.uart.rx_io = FOTA_LAN_UART_RX_PIN;
    config.uart.baud = FOTA_LAN_UART_BAUD_RATE;
    config.uart.rx_buffer_size = FOTA_LAN_UART_BUF_SIZE;
    config.uart.queue_size = FOTA_LAN_PPP_UART_QUEUE_SIZE;
    
    // Use simplified blocking API - connects and waits for connection
    ESP_LOGI(TAG, "Connecting to PPP server...");
    s_eppp_netif = eppp_connect(&config);
    
    if (s_eppp_netif == NULL) {
        ESP_LOGE(TAG, "Failed to connect to PPP server");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "PPP client connected successfully");
    
    // Get and log IP info
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(s_eppp_netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "PPP Client IP   : " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "PPP Netmask     : " IPSTR, IP2STR(&ip_info.netmask));
        ESP_LOGI(TAG, "PPP Gateway     : " IPSTR, IP2STR(&ip_info.gw));
    }
    
    // Optional: Set DNS servers
    esp_netif_dns_info_t dns;
    dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(8, 8, 8, 8);  // Google DNS
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(s_eppp_netif, ESP_NETIF_DNS_MAIN, &dns);
    
    return ESP_OK;
}

/**
 * @brief Disconnect from PPP server
 */
static void fota_lan_disconnect_ppp(void)
{
    if (s_eppp_netif) {
        ESP_LOGI(TAG, "Disconnecting PPP client...");
        eppp_deinit(s_eppp_netif);
        s_eppp_netif = NULL;
        ESP_LOGI(TAG, "PPP client disconnected");
    }
}

/**
 * @brief Advanced OTA task using PPP interface
 */
void fota_lan_handler_advanced_ota_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting Advanced OTA - V1.0.0");
    
    esp_err_t err;
    esp_err_t ota_finish_err = ESP_OK;
    
    // Get network interface name for binding
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    esp_netif_get_netif_impl_name(s_eppp_netif, ifr.ifr_name);
    ESP_LOGI(TAG, "Binding HTTP client to interface: %s", ifr.ifr_name);
    
    // Configure HTTP client to use PPP interface
    esp_http_client_config_t config = {
        .url = FOTA_LAN_FIRMWARE_UPGRADE_URL,
#if FOTA_LAN_USE_CERT_BUNDLE
        .crt_bundle_attach = esp_crt_bundle_attach,
#else
        .cert_pem = (char *)server_cert_pem_start,
#endif
        .timeout_ms = FOTA_LAN_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
        .buffer_size = FOTA_LAN_HTTP_BUFFER_SIZE,
        .buffer_size_tx = FOTA_LAN_HTTP_BUFFER_SIZE_TX,
        .if_name = &ifr,  // Bind to PPP interface
#if FOTA_LAN_ENABLE_PARTIAL_HTTP_DOWNLOAD
        .save_client_session = true,
#endif
#if FOTA_LAN_TLS_DYN_BUF_RX_STATIC
        .tls_dyn_buf_strategy = HTTP_TLS_DYN_BUF_RX_STATIC,
#endif
#if FOTA_LAN_SKIP_COMMON_NAME_CHECK
        .skip_cert_common_name_check = true,
#endif
    };
    
#if FOTA_LAN_ENABLE_OTA_RESUMPTION
    nvs_handle_t nvs_ota_resumption_handle;
    err = nvs_open(NVS_NAMESPACE_OTA_RESUMPTION, NVS_READWRITE, &nvs_ota_resumption_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        goto cleanup;
    }
    
    uint32_t ota_wr_len = 0;
    err = ota_res_get_written_len_from_nvs(nvs_ota_resumption_handle, config.url, &ota_wr_len);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Starting OTA from beginning");
    } else {
        ESP_LOGD(TAG, "OTA write length fetched: %d bytes", ota_wr_len);
    }
#endif
    
    // Configure HTTPS OTA
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = _http_client_init_cb,
#if FOTA_LAN_ENABLE_PARTIAL_HTTP_DOWNLOAD
        .partial_http_download = true,
        .max_http_request_size = FOTA_LAN_HTTP_REQUEST_SIZE,
#endif
#if FOTA_LAN_ENABLE_OTA_RESUMPTION
        .ota_resumption = true,
        .ota_image_bytes_written = ota_wr_len,
#endif
    };
    
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    
    esp_https_ota_handle_t https_ota_handle = NULL;
    err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        goto cleanup;
    }
    
    // Get and validate image description
    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_get_img_desc failed");
        goto ota_end;
    }
    
    err = validate_image_header(&app_desc);
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
        
        // Log progress
        const size_t len = esp_https_ota_get_image_len_read(https_ota_handle);
        ESP_LOGD(TAG, "Image bytes read: %zu", len);
        
#if FOTA_LAN_ENABLE_OTA_RESUMPTION
        err = ota_res_save_cfg_to_nvs(nvs_ota_resumption_handle, len, config.url);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save OTA config to NVS");
        }
#endif
    }
    
    // Check if complete data received
    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        ESP_LOGE(TAG, "Complete data was not received");
        goto ota_end;
    }
    
#if FOTA_LAN_ENABLE_OTA_RESUMPTION
    ota_res_cleanup_cfg_from_nvs(nvs_ota_resumption_handle);
#endif
    
    // Finish OTA
    ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
        ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed: 0x%x", ota_finish_err);
    }
    goto cleanup;

ota_end:
    esp_https_ota_abort(https_ota_handle);
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");

cleanup:
    fota_lan_handler_task_stop();
    vTaskDelete(NULL);
}

/**
 * @brief Start FOTA LAN handler
 * 
 * Initializes network stack, connects PPP client, and starts OTA task
 */
void fota_lan_handler_task_start(void)
{
    ota_task_close = false;
    
    ESP_LOGI(TAG, "FOTA LAN Handler Starting");
    get_sha256_of_partitions();
    
    // Initialize TCP/IP stack
    esp_err_t ret = esp_netif_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Network interface already initialized");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        return;
    }
    
    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Default event loop already created");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return;
    }
    
    // Register OTA event handler
    ret = esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, 
                                     &event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register OTA event handler: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "OTA event handler registered");
    
    // Connect to PPP server
    esp_err_t err = fota_lan_connect_ppp();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to PPP server");
        esp_event_handler_unregister(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, event_handler);
        return;
    }
    
    // Create OTA task
    xTaskCreate(fota_lan_handler_advanced_ota_task, 
                "fota_lan_ota", 
                FOTA_LAN_TASK_STACK_SIZE, 
                NULL, 
                FOTA_LAN_TASK_PRIORITY, 
                NULL);
}

/**
 * @brief Stop FOTA LAN handler
 */
void fota_lan_handler_task_stop(void)
{
    ota_task_close = true;
    
    // Disconnect PPP
    fota_lan_disconnect_ppp();
    
    // Unregister event handler
    esp_event_handler_unregister(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, event_handler);
    
    ESP_LOGI(TAG, "FOTA LAN Handler stopped");
}