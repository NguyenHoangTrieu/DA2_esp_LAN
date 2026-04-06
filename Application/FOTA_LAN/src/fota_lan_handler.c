/*
 * Advanced OTA Update Handler for ESP32
 * This module manages over-the-air firmware updates using HTTPS.
 * It includes features such as image validation, event handling,
 * and optional OTA resumption using NVS.
 */
#include "fota_lan_handler.h"
#include "esp_heap_caps.h"

static const char *TAG = "lan_advanced_ota";

#if FOTA_CONFIG_LAN_ENABLE_CONNECTIVITY_CHECK
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

/*
 * Performs a full TLS handshake to <host>:<port> and logs:
 *   - DNS resolution time
 *   - TCP connect time (bare socket)
 *   - TLS handshake time (with cert-bundle verification)
 *
 * This distinguishes between a simple TCP reachability problem and a
 * TLS-layer stall (e.g. slow ECDH in software, CPU starvation by BLE tasks).
 */
static void connectivity_check(const char *host, uint16_t port) {
  uint32_t t0;

  /* ---- DNS ---- */
  struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
  struct addrinfo *res  = NULL;
  char port_str[8];
  snprintf(port_str, sizeof(port_str), "%u", port);
  t0 = esp_log_timestamp();
  int dns_ret = getaddrinfo(host, port_str, &hints, &res);
  uint32_t dns_ms = esp_log_timestamp() - t0;
  if (dns_ret != 0 || res == NULL) {
    ESP_LOGE(TAG, "[CHECK] DNS FAIL  %s  err=%d  (%lums)",
             host, dns_ret, (unsigned long)dns_ms);
    return;
  }
  char ip_str[INET_ADDRSTRLEN] = {0};
  inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, ip_str, sizeof(ip_str));
  freeaddrinfo(res);
  ESP_LOGI(TAG, "[CHECK] DNS    %s -> %s  (%lums)", host, ip_str, (unsigned long)dns_ms);

  /* ---- TCP connect (bare socket, 5s timeout) ---- */
  {
    struct addrinfo *r2 = NULL;
    getaddrinfo(host, port_str, &hints, &r2);
    if (r2) {
      int sock = socket(AF_INET, SOCK_STREAM, 0);
      if (sock >= 0) {
        struct timeval tv = {.tv_sec = 5, .tv_usec = 0};
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        t0 = esp_log_timestamp();
        int c = connect(sock, r2->ai_addr, r2->ai_addrlen);
        uint32_t tcp_ms = esp_log_timestamp() - t0;
        if (c == 0)
          ESP_LOGI(TAG, "[CHECK] TCP:443 OK    %s  (%lums)", host, (unsigned long)tcp_ms);
        else
          ESP_LOGE(TAG, "[CHECK] TCP:443 FAIL  %s  errno=%d  (%lums)", host, errno, (unsigned long)tcp_ms);
        close(sock);
      }
      freeaddrinfo(r2);
    }
  }

  /* ---- Full TLS handshake (25s timeout) ---- */
  esp_tls_cfg_t cfg = {
      .crt_bundle_attach = esp_crt_bundle_attach,
      .timeout_ms        = 25000,
      .non_block         = false,
  };
  esp_tls_t *tls = esp_tls_init();
  if (!tls) {
    ESP_LOGE(TAG, "[CHECK] esp_tls_init() OOM for %s", host);
    return;
  }
  t0 = esp_log_timestamp();
  int ret = esp_tls_conn_new_sync(host, (int)strlen(host), (int)port, &cfg, tls);
  int fd = -1;
  esp_tls_get_conn_sockfd(tls, &fd);
  if (fd >= 0) {
    struct timeval tv = { .tv_sec = 30, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ESP_LOGI(TAG, "[OTA-TLS] SO_RCVTIMEO set to 30s on fd=%d", fd);
  }
  uint32_t tls_ms = esp_log_timestamp() - t0;
  if (ret == 1) {
    ESP_LOGI(TAG, "[CHECK] TLS OK    %s  (%lums)", host, (unsigned long)tls_ms);
  } else {
    int esp_err = 0, mbedtls_flags = 0;
    esp_tls_error_handle_t eh = NULL;
    esp_tls_get_error_handle(tls, &eh);
    if (eh) esp_tls_get_and_clear_last_error(eh, &esp_err, &mbedtls_flags);
    ESP_LOGE(TAG, "[CHECK] TLS FAIL  %s  ret=%d esp_err=0x%x mbedtls=0x%x  (%lums)",
             host, ret, esp_err, mbedtls_flags, (unsigned long)tls_ms);
  }
  esp_tls_conn_destroy(tls);
}
#endif /* FOTA_CONFIG_LAN_ENABLE_CONNECTIVITY_CHECK */

#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF
/* The interface name value can refer to if_desc in esp_netif_defaults.h */
#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_ETH
static const char *bind_interface_name = NETIF_DESC_ETH;
#elif FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_STA
static const char *bind_interface_name = NETIF_DESC_STA;
#endif
#endif

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
  ESP_LOGI(TAG, "[OTA] HTTP client init cb: new TLS hop starting (t=%lums)",
           (unsigned long)esp_log_timestamp());
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

/*
 * FIX 1: Synchronous BLE disable with status polling.
 *
 * Original code used vTaskDelay(500ms) which is a blind wait — BLE controller
 * disable is asynchronous and 500ms may not be enough. If BLE interrupt fires
 * during esp_tls_conn_read(), it stalls UART DMA for 1-3ms per interrupt,
 * causing PPP/LwIP to drop TCP segments, which stalls TLS record reassembly
 * and ultimately triggers MBEDTLS_ERR_SSL_TIMEOUT (-26880 / 0x6900).
 *
 * This helper polls esp_bt_controller_get_status() until IDLE or timeout.
 */
#include "esp_bt.h"
#include "esp_bt_main.h"

static void ble_disable_sync(void) {
  ESP_LOGW(TAG, "[OTA-TLS] Temporarily disabling BLE to secure UART DMA...");

  esp_bluedroid_disable();
  /* Poll until bluedroid is fully stopped (max 2s) */
  for (int i = 0; i < 20; i++) {
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED ||
        esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
      /* ENABLED means disable() call hasn't taken effect yet, keep polling */
      if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) break;
    } else {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  esp_bt_controller_disable();
  /* Poll until BT controller reaches IDLE state (max 2s) */
  for (int i = 0; i < 20; i++) {
    esp_bt_controller_status_t st = esp_bt_controller_get_status();
    if (st == ESP_BT_CONTROLLER_STATUS_IDLE) {
      ESP_LOGI(TAG, "[OTA-TLS] BT controller idle after %dms", (i + 1) * 100);
      break;
    }
    if (i == 19) {
      ESP_LOGW(TAG, "[OTA-TLS] BT controller not idle after 2s (status=%d), proceeding anyway", st);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/*
 * Manual OTA download using esp_tls directly.
 *
 * esp_https_ota / esp_http_client TLS to raw.githubusercontent.com fails
 * deterministically at 21s with -0x004C (RST) after cert validation.
 * However, esp_tls_conn_new_sync() to the SAME host completes in ~700ms
 * every time.  This function bypasses the broken esp_http_client TLS path
 * and uses only esp_tls_conn_new_sync() + raw HTTP/1.1 GET.
 */
#include "esp_tls.h"

#define OTA_HOST "raw.githubusercontent.com"
#define OTA_PATH "/NguyenHoangTrieu/DATN_config_app/main/dist/bin/DA2_esp_LAN.bin"
#define OTA_DL_BUF_SIZE 4096

/*
 * FIX 3: Block-read HTTP response headers with sliding-window CRLFCRLF detector.
 *
 * Original code read 1 byte per esp_tls_conn_read() call. With TLS record
 * overhead, this generates up to 1024 round-trips just to read the header,
 * consuming most of the 30s timeout window before any body data arrives.
 *
 * This implementation reads up to HDR_CHUNK_SIZE bytes per call and detects
 * the end-of-headers marker (\r\n\r\n) using a 4-byte sliding window.
 * Reduces TLS record calls from ~1024 to ~3-5 for a typical HTTP response header.
 */
#define HDR_BUF_SIZE   2048
#define HDR_CHUNK_SIZE 256

static esp_err_t read_http_headers(esp_tls_t *tls, char *hdr_out, int hdr_out_size,
                                   int *hdr_len_out) {
  int hdr_len = 0;
  /* Sliding window: track last 3 bytes to detect \r\n\r\n across chunk boundaries */
  uint8_t tail[3] = {0};
  bool header_done = false;

  while (hdr_len < hdr_out_size - 1) {
    int want = hdr_out_size - 1 - hdr_len;
    if (want > HDR_CHUNK_SIZE) want = HDR_CHUNK_SIZE;

    int rd = esp_tls_conn_read(tls, (unsigned char *)hdr_out + hdr_len, want);
    if (rd <= 0) {
        int esp_err_code = 0, mbedtls_flags = 0;
        esp_tls_error_handle_t eh = NULL;
        esp_tls_get_error_handle(tls, &eh);
        if (eh) {
            esp_tls_get_and_clear_last_error(eh, &esp_err_code, &mbedtls_flags);
        }
        ESP_LOGE(TAG, "[OTA-TLS] header read failed: rd=%d, esp_err=0x%x, mbedtls=0x%x, errno=%d (%s)",
                rd, esp_err_code, mbedtls_flags, errno, strerror(errno));
        goto cleanup;
    }

    /* Scan newly read chunk for \r\n\r\n */
    for (int i = 0; i < rd; i++) {
      uint8_t b = (uint8_t)hdr_out[hdr_len + i];
      /* Check the 4-byte window: tail[0] tail[1] tail[2] b */
      if (hdr_len + i >= 3) {
        /* All 4 bytes available in hdr_out */
        if (hdr_out[hdr_len + i - 3] == '\r' &&
            hdr_out[hdr_len + i - 2] == '\n' &&
            hdr_out[hdr_len + i - 1] == '\r' &&
            b == '\n') {
          hdr_len += (i + 1);
          hdr_out[hdr_len] = '\0';
          header_done = true;
          break;
        }
      } else {
        /* Straddles the boundary: use tail[] for the first bytes */
        int pos = hdr_len + i; /* absolute position in hdr_out */
        uint8_t w[4];
        /* Build a 4-byte window from tail + current chunk */
        for (int j = 0; j < 4; j++) {
          int abs = pos - 3 + j;
          if (abs < 0) {
            /* Before start of hdr_out — use tail ring */
            w[j] = tail[3 + abs]; /* tail has indices 0,1,2 for positions -3,-2,-1 */
          } else if (abs < hdr_len) {
            w[j] = (uint8_t)hdr_out[abs];
          } else {
            w[j] = (uint8_t)hdr_out[hdr_len + (abs - hdr_len)];
          }
        }
        if (w[0] == '\r' && w[1] == '\n' && w[2] == '\r' && w[3] == '\n') {
          hdr_len += (i + 1);
          hdr_out[hdr_len] = '\0';
          header_done = true;
          break;
        }
      }
    }

    if (!header_done) {
      /* Update tail with last 3 bytes of hdr_out so far */
      hdr_len += rd;
      int tstart = hdr_len - 3;
      if (tstart < 0) tstart = 0;
      for (int j = 0; j < 3; j++) {
        int src = tstart + j;
        tail[j] = (src < hdr_len) ? (uint8_t)hdr_out[src] : 0;
      }
    } else {
      break;
    }
  }

  if (!header_done) {
    ESP_LOGE(TAG, "[OTA-TLS] Header buffer overflow (%d bytes, no CRLFCRLF found)", hdr_len);
    return ESP_FAIL;
  }

  *hdr_len_out = hdr_len;
  return ESP_OK;
}

static esp_err_t manual_ota_download(void) {
  esp_err_t ret = ESP_FAIL;
  esp_tls_t *tls = NULL;
  uint8_t *buf = NULL;
  esp_ota_handle_t ota_handle = 0;
  bool ota_started = false;

  tls = esp_tls_init();
  if (!tls) {
    ESP_LOGE(TAG, "[OTA-TLS] esp_tls_init failed");
    return ESP_ERR_NO_MEM;
  }

  /*
   * FIX 1: Synchronous BLE disable — poll until controller reaches IDLE.
   * Previous: vTaskDelay(500ms) was a blind wait that could leave BLE interrupts
   * active during TLS read, causing UART DMA overruns -> PPP segment loss ->
   * MBEDTLS_ERR_SSL_TIMEOUT.
   */
  ble_disable_sync();

  /* ALPN "http/1.1" is required: Fastly CDN uses ALPN to route TLS
   * connections to the HTTP handler. Without it, TLS connects fine but
   * the CDN silently drops all HTTP requests (30s read timeout). */
  static const char *alpn[] = {"http/1.1", NULL};

  esp_tls_cfg_t tls_cfg = {
      .crt_bundle_attach = esp_crt_bundle_attach,
      /*
       * FIX 2: Increased from 30000ms to 60000ms.
       * Fastly CDN has a ~30s idle timeout. The previous value raced exactly at
       * that limit — any scheduling jitter caused deterministic timeout before
       * the first response byte arrived.
       */
      .timeout_ms        = 60000,
      .non_block         = false,
      .alpn_protos       = alpn,
  };

  uint32_t t0 = esp_log_timestamp();
  ESP_LOGI(TAG, "[OTA-TLS] connect to %s:443", OTA_HOST);
  int r = esp_tls_conn_new_sync(OTA_HOST, strlen(OTA_HOST), 443, &tls_cfg, tls);
  uint32_t tls_ms = esp_log_timestamp() - t0;
  ESP_LOGI(TAG, "[OTA-TLS] connect ret=%d (%lums)", r, (unsigned long)tls_ms);
  if (r != 1) {
    ESP_LOGE(TAG, "[OTA-TLS] TLS connect failed");
    goto cleanup;
  }

  /* ---- Send HTTP/1.1 GET (Fastly CDN drops HTTP/1.0 silently) ---- */
  {
    char req[512];
    int req_len = snprintf(req, sizeof(req),
        "GET " OTA_PATH " HTTP/1.0\r\n"
        "Host: " OTA_HOST "\r\n"
        "Accept: */*\r\n"
        "User-Agent: ESP32-OTA/1.0\r\n"
        "\r\n");
    int written = 0;
    while (written < req_len) {
        int w = esp_tls_conn_write(tls, req + written, req_len - written);
        if (w < 0) {
            ESP_LOGE(TAG, "[OTA-TLS] write failed: %d, errno=%d", w, errno);
            goto cleanup;
        }
        if (w == 0) {
            ESP_LOGE(TAG, "[OTA-TLS] write returned 0 (connection closed by peer)");
            goto cleanup;
        }
        written += w;
    }
    ESP_LOGI(TAG, "[OTA-TLS] Request first line: %.50s", req);

  /* ---- Read HTTP response headers (FIX 3: block read, not byte-by-byte) ---- */
  int content_length = -1;
  {
    char *hdr = malloc(HDR_BUF_SIZE);
    if (!hdr) {
      ESP_LOGE(TAG, "[OTA-TLS] malloc(%d) for header buffer failed", HDR_BUF_SIZE);
      ret = ESP_ERR_NO_MEM;
      goto cleanup;
    }

    int hdr_len = 0;
    esp_err_t hdr_err = read_http_headers(tls, hdr, HDR_BUF_SIZE, &hdr_len);
    if (hdr_err != ESP_OK) {
      free(hdr);
      goto cleanup;
    }

    /* Check status */
    if (strstr(hdr, " 200") == NULL) {
      ESP_LOGE(TAG, "[OTA-TLS] HTTP response not 200:\n%.120s", hdr);
      free(hdr);
      goto cleanup;
    }
    /* Parse Content-Length */
    const char *cl = strstr(hdr, "Content-Length:");
    if (!cl) cl = strstr(hdr, "content-length:");
    if (cl) {
      content_length = atoi(cl + 15);
    }
    ESP_LOGI(TAG, "[OTA-TLS] HTTP 200 OK, Content-Length=%d", content_length);
    free(hdr);
  }

  if (content_length <= 0) {
    ESP_LOGE(TAG, "[OTA-TLS] Invalid Content-Length");
    goto cleanup;
  }

  /* ---- Begin OTA flash write ---- */
  {
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
      ESP_LOGE(TAG, "[OTA-TLS] No OTA partition available");
      goto cleanup;
    }
    ESP_LOGI(TAG, "[OTA-TLS] Writing to <%s> at offset 0x%lx",
             update->label, (unsigned long)update->address);

    ret = esp_ota_begin(update, (size_t)content_length, &ota_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "[OTA-TLS] esp_ota_begin failed: %s", esp_err_to_name(ret));
      goto cleanup;
    }
    ota_started = true;

    /* ---- Download body → flash ---- */
    buf = malloc(OTA_DL_BUF_SIZE);
    if (!buf) {
      ESP_LOGE(TAG, "[OTA-TLS] malloc(%d) failed", OTA_DL_BUF_SIZE);
      ret = ESP_ERR_NO_MEM;
      goto cleanup;
    }

    int total = 0;
    t0 = esp_log_timestamp();
    while (total < content_length) {
      int want = content_length - total;
      if (want > OTA_DL_BUF_SIZE) want = OTA_DL_BUF_SIZE;

      int rd = esp_tls_conn_read(tls, buf, want);
      if (rd < 0) {
        ESP_LOGE(TAG, "[OTA-TLS] read error %d at %d/%d", rd, total, content_length);
        ret = ESP_FAIL;
        goto cleanup;
      }
      if (rd == 0) {
        ESP_LOGE(TAG, "[OTA-TLS] unexpected EOF at %d/%d", total, content_length);
        ret = ESP_FAIL;
        goto cleanup;
      }

      ret = esp_ota_write(ota_handle, buf, rd);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[OTA-TLS] esp_ota_write failed: %s", esp_err_to_name(ret));
        goto cleanup;
      }

      total += rd;
      /* Log every ~64KB */
      if ((total / (64 * 1024)) != ((total - rd) / (64 * 1024))) {
        ESP_LOGI(TAG, "[OTA-TLS] %d/%d (%d%%)",
                 total, content_length, total * 100 / content_length);
      }
    }

    uint32_t dl_ms = esp_log_timestamp() - t0;
    ESP_LOGI(TAG, "[OTA-TLS] Download complete: %d bytes in %lums (%ld B/s)",
             total, (unsigned long)dl_ms,
             dl_ms > 0 ? (long)(total * 1000L / dl_ms) : 0);

    ret = esp_ota_end(ota_handle);
    ota_started = false; /* esp_ota_end was called, don't abort */
    if (ret != ESP_OK) {
      if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
        ESP_LOGE(TAG, "[OTA-TLS] Image validation failed (corrupted)");
      } else {
        ESP_LOGE(TAG, "[OTA-TLS] esp_ota_end failed: %s", esp_err_to_name(ret));
      }
      goto cleanup;
    }

    ret = esp_ota_set_boot_partition(update);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "[OTA-TLS] set_boot_partition failed: %s", esp_err_to_name(ret));
      goto cleanup;
    }

    ESP_LOGI(TAG, "[OTA-TLS] OTA successful! Rebooting in 1s...");
    ret = ESP_OK;
  }

cleanup:
  free(buf);
  if (ota_started) {
    esp_ota_abort(ota_handle);
  }
  if (tls) {
    esp_tls_conn_destroy(tls);
  }
  return ret;
}
}

void advanced_ota_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting Advanced OTA (direct TLS) - V2.0.0");

  const int max_retries = 5;
  esp_err_t err = ESP_FAIL;

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    uint32_t t0 = esp_log_timestamp();
    ESP_LOGI(TAG, "OTA attempt %d/%d (t=%lums)",
             attempt, max_retries, (unsigned long)t0);

    err = manual_ota_download();

    uint32_t elapsed = esp_log_timestamp() - t0;
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "OTA attempt %d succeeded after %lums",
               attempt, (unsigned long)elapsed);
      vTaskDelay(pdMS_TO_TICKS(1000));
      esp_restart();
    }

    ESP_LOGE(TAG, "OTA attempt %d failed after %lums: %s (0x%x)",
             attempt, (unsigned long)elapsed, esp_err_to_name(err), err);

    if (attempt < max_retries) {
      uint32_t delay_ms = 3000 * attempt;
      ESP_LOGW(TAG, "Retrying in %lums ...", (unsigned long)delay_ms);
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }

  ESP_LOGE(TAG, "OTA failed after %d attempts, rebooting", max_retries);
  esp_restart();
  vTaskDelete(NULL);
}

void fota_lan_handler_task_start(void) {
  ota_task_close = false;
  get_sha256_of_partitions();

  size_t internal_free    = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "Heap before OTA task: total=%d, internal=%d, internal_largest=%d",
           esp_get_free_heap_size(), internal_free, internal_largest);

  /* Stack MUST be in internal RAM for mbedTLS performance.
   * 12KB is sufficient; fallback to PSRAM if internal RAM is fragmented. */
  /* Priority 5: P-256 TLS handshake completes in ~760ms even at low priority.
   * Higher priority starved PPP/LwIP tasks → UART_FIFO_OVF → TCP data corruption.
   * Do NOT raise this above the LwIP task priority (CONFIG_LWIP_TCPIP_TASK_PRIO). */
  const UBaseType_t ota_prio = 5;
  BaseType_t ret = xTaskCreate(&advanced_ota_task, "advanced_ota_task", 12 * 1024, NULL, ota_prio, NULL);
  if (ret != pdPASS) {
    ESP_LOGW(TAG, "Internal RAM stack failed (largest=%d), retrying in PSRAM", internal_largest);
    ret = xTaskCreateWithCaps(&advanced_ota_task, "advanced_ota_task",
                              32 * 1024, NULL, ota_prio, NULL,
                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ret != pdPASS) {
      ESP_LOGE(TAG, "Failed to create OTA task (internal AND PSRAM)");
      return;
    }
  }
  ESP_LOGI(TAG, "OTA task created successfully");
}

void fota_lan_handler_task_stop(void) { ota_task_close = true; }
