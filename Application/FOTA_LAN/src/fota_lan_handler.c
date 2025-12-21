/*
 * Advanced OTA Update Handler for ESP32
 * This module manages over-the-air firmware updates using HTTPS.
 * It includes features such as image validation, event handling,
 * and optional OTA resumption using NVS.
 */
#include "fota_lan_handler.h"

#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF
/* The interface name value can refer to if_desc in esp_netif_defaults.h */
#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_ETH
static const char *bind_interface_name = NETIF_DESC_ETH;
#elif FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_STA
static const char *bind_interface_name = NETIF_DESC_STA;
#endif
#endif

static const char *TAG = "lan_advanced_ota";

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static bool ota_task_close = false;

#if FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION
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

#if FOTA_CONFIG_LAN_SKIP_VERSION_CHECK
  if (memcmp(new_app_info->version, running_app_info.version,
             sizeof(new_app_info->version)) == 0) {
    ESP_LOGW(TAG,
             "Current running version is the same as new. Update cancelled.");
    return ESP_FAIL;
  }
#endif

#if FOTA_CONFIG_LAN_BOOTLOADER_APP_ANTI_ROLLBACK
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

void advanced_ota_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting Advanced OTA - V1.0.0");

  esp_err_t err;
  esp_err_t ota_finish_err = ESP_OK;

#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF
  esp_netif_t *netif = get_netif_from_desc(bind_interface_name);
  if (netif == NULL) {
    ESP_LOGE(TAG, "Can't find netif from interface description");
    fota_handler_task_stop();
    vTaskDelete(NULL);
  }

  struct ifreq ifr;
  esp_netif_get_netif_impl_name(netif, ifr.ifr_name);
  ESP_LOGI(TAG, "Bind interface name is %s", ifr.ifr_name);
#endif

  esp_http_client_config_t config = {
      .url = FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL,
#if FOTA_CONFIG_LAN_USE_CERT_BUNDLE
      .crt_bundle_attach = esp_crt_bundle_attach,
#else
      .cert_pem = (char *)server_cert_pem_start,
#endif
      .timeout_ms = FOTA_CONFIG_LAN_OTA_RECV_TIMEOUT,
      .keep_alive_enable = true,
      .buffer_size = 8 * 1024,
      .buffer_size_tx = 8 * 1024,
#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF
      .if_name = &ifr,
#endif
#if FOTA_CONFIG_LAN_ENABLE_PARTIAL_HTTP_DOWNLOAD
      .save_client_session = true,
#endif
#if FOTA_CONFIG_LAN_TLS_DYN_BUF_RX_STATIC
      .tls_dyn_buf_strategy = HTTP_TLS_DYN_BUF_RX_STATIC,
#endif
  };

#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL_FROM_STDIN
  char url_buf[OTA_URL_SIZE];
  if (strcmp(config.url, "FROM_STDIN") == 0) {
    configure_stdin_stdout();
    fgets(url_buf, OTA_URL_SIZE, stdin);
    int len = strlen(url_buf);
    url_buf[len - 1] = '\0';
    config.url = url_buf;
  } else {
    ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
    fota_handler_task_stop();
    vTaskDelete(NULL);
  }
#endif

#if FOTA_CONFIG_LAN_SKIP_COMMON_NAME_CHECK
  config.skip_cert_common_name_check = true;
#endif

#if FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION
  nvs_handle_t nvs_ota_resumption_handle;
  err = nvs_open(NVS_NAMESPACE_OTA_RESUMPTION, NVS_READWRITE,
                 &nvs_ota_resumption_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    fota_handler_task_stop();
    vTaskDelete(NULL);
  }

  uint32_t ota_wr_len = 0;
  err = ota_res_get_written_len_from_nvs(nvs_ota_resumption_handle, config.url,
                                         &ota_wr_len);
  if (err != ESP_OK) {
    ESP_LOGD(TAG, "Starting OTA from beginning");
  } else {
    ESP_LOGD(TAG, "OTA write length fetched successfully: %d bytes",
             ota_wr_len);
  }
#endif

  esp_https_ota_config_t ota_config = {
      .http_config = &config,
      .http_client_init_cb = _http_client_init_cb,
#if FOTA_CONFIG_LAN_ENABLE_PARTIAL_HTTP_DOWNLOAD
      .partial_http_download = true,
      .max_http_request_size = FOTA_CONFIG_LAN_HTTP_REQUEST_SIZE,
#endif
#if FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION
      .ota_resumption = true,
      .ota_image_bytes_written = ota_wr_len,
#endif
  };

  ESP_LOGI(TAG, "Attempting to download update from %s", config.url);

  esp_https_ota_handle_t https_ota_handle = NULL;
  err = esp_https_ota_begin(&ota_config, &https_ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
    fota_lan_handler_task_stop();
    vTaskDelete(NULL);
  }

  esp_app_desc_t app_desc = {};
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

  while (1) {
    err = esp_https_ota_perform(https_ota_handle);
    if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
      break;
    }

    // Monitor OTA progress
    const size_t len = esp_https_ota_get_image_len_read(https_ota_handle);
    ESP_LOGD(TAG, "Image bytes read: %d", len);

#if FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION
    err = ota_res_save_cfg_to_nvs(nvs_ota_resumption_handle, len, config.url);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save OTA config to NVS (%s)",
               esp_err_to_name(err));
    }
#endif
  }

  if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
    ESP_LOGE(TAG, "Complete data was not received.");
  } else {
#if FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION
    err = ota_res_cleanup_cfg_from_nvs(nvs_ota_resumption_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to clean up OTA config from NVS (%s)",
               esp_err_to_name(err));
    }
#endif
    ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
      ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      esp_restart();
    } else {
      if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
        ESP_LOGE(TAG, "Image validation failed, image is corrupted");
      }
      ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
      fota_lan_handler_task_stop();
      vTaskDelete(NULL);
    }
  }

ota_end:
  esp_https_ota_abort(https_ota_handle);
  ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
  fota_lan_handler_task_stop();
  vTaskDelete(NULL);
}

void fota_lan_handler_task_start(void) {
  ota_task_close = false;
  get_sha256_of_partitions();

  // Register event handler for OTA events
  ESP_ERROR_CHECK(esp_event_handler_register(
      ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

#if FOTA_CONFIG_LAN_CONNECT_WIFI
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  xTaskCreate(&advanced_ota_task, "advanced_ota_task", 32 * 1024, NULL, 5,
              NULL);
}

void fota_lan_handler_task_stop(void) { ota_task_close = true; }
