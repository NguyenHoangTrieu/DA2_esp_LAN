/*
 * Advanced OTA Update Handler for ESP32
 * Direct TLS + raw HTTP/1.0 GET ΓÇö bypasses esp_http_client broken path.
 */
#include "fota_lan_handler.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_tls.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <string.h>


static const char *TAG = "lan_advanced_ota";

/* ------------------------------------------------------------------ */
/*  OTA target                                                          */
/* ------------------------------------------------------------------ */
/* OTA target ΓÇö raw.githubusercontent.com uses Fastly CDN.
 * If Fastly rate-limits the IP (silent hold, no HTTP response),
 * we fall back to objects.githubusercontent.com which uses a
 * different Fastly PoP / backend pool and may not be blocked. */
#define OTA_HOST_PRIMARY  "raw.githubusercontent.com"
#define OTA_HOST_FALLBACK "objects.githubusercontent.com"
#define OTA_PATH \
  "/NguyenHoangTrieu/DATN_config_app/main/dist/bin/DA2_esp_LAN.bin"
/* objects.githubusercontent.com uses the same path format as raw for
 * content served via GitHub's blob CDN (same content, different PoP). */
#define OTA_PATH_FALLBACK OTA_PATH
#define OTA_HOST OTA_HOST_PRIMARY   /* default ΓÇö overridden per attempt */
#define OTA_DL_BUF_SIZE 4096
#define HDR_BUF_SIZE 2048
#define HDR_CHUNK_SIZE 256


/* ------------------------------------------------------------------ */
/*  Connectivity check (diagnostic only)                               */
/* ------------------------------------------------------------------ */
#if FOTA_CONFIG_LAN_ENABLE_CONNECTIVITY_CHECK
static void connectivity_check(const char *host, uint16_t port) {
  uint32_t t0;
  char port_str[8];
  snprintf(port_str, sizeof(port_str), "%u", port);

  struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
  struct addrinfo *res = NULL;
  t0 = esp_log_timestamp();
  int dns_ret = getaddrinfo(host, port_str, &hints, &res);
  uint32_t dns_ms = esp_log_timestamp() - t0;
  if (dns_ret != 0 || res == NULL) {
    ESP_LOGE(TAG, "[CHECK] DNS FAIL %s err=%d (%lums)", host, dns_ret,
             (unsigned long)dns_ms);
    return;
  }
  char ip_str[INET_ADDRSTRLEN] = {0};
  inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, ip_str,
              sizeof(ip_str));
  freeaddrinfo(res);
  ESP_LOGI(TAG, "[CHECK] DNS %s -> %s (%lums)", host, ip_str,
           (unsigned long)dns_ms);

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
          ESP_LOGI(TAG, "[CHECK] TCP:443 OK %s (%lums)", host,
                   (unsigned long)tcp_ms);
        else
          ESP_LOGE(TAG, "[CHECK] TCP:443 FAIL %s errno=%d (%lums)", host, errno,
                   (unsigned long)tcp_ms);
        close(sock);
      }
      freeaddrinfo(r2);
    }
  }

  esp_tls_cfg_t cfg = {
      .crt_bundle_attach = esp_crt_bundle_attach,
      .timeout_ms = 25000,
      .non_block = false,
  };
  esp_tls_t *tls = esp_tls_init();
  if (!tls) {
    ESP_LOGE(TAG, "[CHECK] esp_tls_init OOM");
    return;
  }
  t0 = esp_log_timestamp();
  int ret =
      esp_tls_conn_new_sync(host, (int)strlen(host), (int)port, &cfg, tls);
  uint32_t tls_ms = esp_log_timestamp() - t0;
  if (ret == 1) {
    ESP_LOGI(TAG, "[CHECK] TLS OK %s (%lums)", host, (unsigned long)tls_ms);
  } else {
    int ec = 0, mf = 0;
    esp_tls_error_handle_t eh = NULL;
    esp_tls_get_error_handle(tls, &eh);
    if (eh)
      esp_tls_get_and_clear_last_error(eh, &ec, &mf);
    ESP_LOGE(TAG, "[CHECK] TLS FAIL %s ret=%d esp=0x%x mbed=0x%x (%lums)", host,
             ret, ec, mf, (unsigned long)tls_ms);
  }
  esp_tls_conn_destroy(tls);
}
#endif /* FOTA_CONFIG_LAN_ENABLE_CONNECTIVITY_CHECK */

/* ------------------------------------------------------------------ */
/*  Binding interface (optional)                                        */
/* ------------------------------------------------------------------ */
#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF
#if FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_ETH
static const char *bind_interface_name = NETIF_DESC_ETH;
#elif FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_STA
static const char *bind_interface_name = NETIF_DESC_STA;
#endif
#endif

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static bool ota_task_close = false;

/* ------------------------------------------------------------------ */
/*  NVS resumption helpers                                              */
/* ------------------------------------------------------------------ */
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

  err = nvs_get_str(nvs_handle, NVS_KEY_SAVED_URL, saved_url, &url_len);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGD(TAG, "Saved URL not init");
    return err;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error reading saved URL (%s)", esp_err_to_name(err));
    return err;
  }

  if (strcmp(url, saved_url) != 0) {
    ESP_LOGD(TAG, "URLs do not match. Restarting OTA from beginning.");
    return ESP_ERR_INVALID_STATE;
  }

  uint16_t saved_wr_len_kb = 0;
  err = nvs_get_u16(nvs_handle, NVS_KEY_OTA_WR_LENGTH, &saved_wr_len_kb);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    *nvs_ota_wr_len = 0;
    return err;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error reading OTA write length (%s)", esp_err_to_name(err));
    return err;
  }

  *nvs_ota_wr_len = saved_wr_len_kb * 1024;
  return ESP_OK;
}

static esp_err_t ota_res_save_cfg_to_nvs(const nvs_handle_t nvs_handle,
                                         int nvs_ota_wr_len, const char *url) {
  uint16_t wr_len_kb = nvs_ota_wr_len / 1024;
  ESP_RETURN_ON_ERROR(nvs_set_u16(nvs_handle, NVS_KEY_OTA_WR_LENGTH, wr_len_kb),
                      TAG, "Failed to set OTA write length");
  if (nvs_ota_wr_len) {
    char saved_url[OTA_URL_SIZE] = {0};
    size_t url_len = sizeof(saved_url);
    esp_err_t err =
        nvs_get_str(nvs_handle, NVS_KEY_SAVED_URL, saved_url, &url_len);
    if (err == ESP_ERR_NVS_NOT_FOUND || strcmp(saved_url, url) != 0) {
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
  ESP_GOTO_ON_ERROR(nvs_erase_all(handle), err, TAG, "Error in erasing NVS");
  ESP_GOTO_ON_ERROR(nvs_commit(handle), err, TAG, "Error in committing NVS");
  ret = ESP_OK;
err:
  nvs_close(handle);
  return ret;
}
#endif /* FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION */

/* ------------------------------------------------------------------ */
/*  HTTPS OTA event handler                                             */
/* ------------------------------------------------------------------ */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base != ESP_HTTPS_OTA_EVENT)
    return;
  switch (event_id) {
  case ESP_HTTPS_OTA_START:
    ESP_LOGI(TAG, "OTA started");
    break;
  case ESP_HTTPS_OTA_CONNECTED:
    ESP_LOGI(TAG, "Connected to server");
    break;
  case ESP_HTTPS_OTA_GET_IMG_DESC:
    ESP_LOGI(TAG, "Reading Image Desc");
    break;
  case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
    ESP_LOGI(TAG, "Verifying chip id: %d", *(esp_chip_id_t *)event_data);
    break;
  case ESP_HTTPS_OTA_VERIFY_CHIP_REVISION:
    ESP_LOGI(TAG, "Verifying chip rev: %d", *(esp_chip_id_t *)event_data);
    break;
  case ESP_HTTPS_OTA_DECRYPT_CB:
    ESP_LOGI(TAG, "Decrypt cb");
    break;
  case ESP_HTTPS_OTA_WRITE_FLASH:
    ESP_LOGD(TAG, "Flash write: %d", *(int *)event_data);
    break;
  case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
    ESP_LOGI(TAG, "Boot partition updated. Next: %d",
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

/* ------------------------------------------------------------------ */
/*  Image header validation                                             */
/* ------------------------------------------------------------------ */
static esp_err_t validate_image_header(esp_app_desc_t *new_app_info) {
  if (new_app_info == NULL)
    return ESP_ERR_INVALID_ARG;

  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_app_desc_t running_app_info;
  if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
    ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);

#if FOTA_CONFIG_LAN_SKIP_VERSION_CHECK
  if (memcmp(new_app_info->version, running_app_info.version,
             sizeof(new_app_info->version)) == 0) {
    ESP_LOGW(TAG, "Same version, update cancelled.");
    return ESP_FAIL;
  }
#endif
#if FOTA_CONFIG_LAN_BOOTLOADER_APP_ANTI_ROLLBACK
  const uint32_t hw_sec_version = esp_efuse_read_secure_version();
  if (new_app_info->secure_version < hw_sec_version) {
    ESP_LOGW(TAG, "New secure_version %d < eFuse %d",
             new_app_info->secure_version, hw_sec_version);
    return ESP_FAIL;
  }
#endif
  return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client) {
  ESP_LOGI(TAG, "[OTA] HTTP client init cb (t=%lums)",
           (unsigned long)esp_log_timestamp());
  return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  SHA-256 helpers                                                     */
/* ------------------------------------------------------------------ */
static void print_sha256(const uint8_t *image_hash, const char *label) {
  char hash_print[HASH_LEN * 2 + 1];
  hash_print[HASH_LEN * 2] = 0;
  for (int i = 0; i < HASH_LEN; ++i)
    sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
  ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void) {
  uint8_t sha_256[HASH_LEN] = {0};
  esp_partition_t partition;

  partition.address = ESP_BOOTLOADER_OFFSET;
  partition.size = ESP_PARTITION_TABLE_OFFSET;
  partition.type = ESP_PARTITION_TYPE_APP;
  esp_partition_get_sha256(&partition, sha_256);
  print_sha256(sha_256, "SHA-256 for bootloader:");

  esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
  print_sha256(sha_256, "SHA-256 for current firmware:");
}

/* ------------------------------------------------------------------ */
/*  BLE disable (synchronous poll)                                      */
/* ------------------------------------------------------------------ */
static void ble_disable_sync(void) {
  ESP_LOGW(TAG, "[OTA-TLS] Disabling BLE...");
  esp_bluedroid_disable();
  for (int i = 0; i < 20; i++) {
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED)
      break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  esp_bt_controller_disable();
  for (int i = 0; i < 20; i++) {
    esp_bt_controller_status_t st = esp_bt_controller_get_status();
    if (st == ESP_BT_CONTROLLER_STATUS_IDLE) {
      ESP_LOGI(TAG, "[OTA-TLS] BT idle after %dms", (i + 1) * 100);
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGW(TAG, "[OTA-TLS] BT not idle after 2s, proceeding anyway");
}

/* ------------------------------------------------------------------ */
/*  Internet connectivity pre-check (no TLS ΓÇö avoids CDN rate limit)   */
/* ------------------------------------------------------------------ */
/*
 * internet_reachable: TCP connect to 1.1.1.1:80 (Cloudflare HTTP).
 *
 * Purpose: verify PPP routing works BEFORE spending 60s on a TLS
 * attempt to Fastly CDN.  Using plain TCP (not TLS) so we do not
 * trigger Fastly's per-IP TLS rate limiter, which causes subsequent
 * OTA TLS handshakes to stall.
 *
 * If TCP connect to 1.1.1.1:80 fails the PPP link or its NAT table
 * is not forwarding return traffic yet ΓÇö wait and try again.
 * If it succeeds, internet routing is confirmed working, proceed with
 * the HTTPS OTA download.
 */
static bool internet_reachable(void) {
  /* 1.1.1.1 = Cloudflare public DNS / HTTP server, always reachable */
  struct sockaddr_in addr = {
      .sin_family = AF_INET,
      .sin_port   = htons(80),
  };
  addr.sin_addr.s_addr = inet_addr("1.1.1.1");

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ESP_LOGE(TAG, "[OTA-CHECK] socket() failed: errno=%d", errno);
    return false;
  }

  /* 10s connect timeout */
  struct timeval tv = {.tv_sec = 10, .tv_usec = 0};
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint32_t t0 = esp_log_timestamp();
  int r = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  uint32_t ms = esp_log_timestamp() - t0;
  close(sock);

  if (r == 0) {
    ESP_LOGI(TAG, "[OTA-CHECK] 1.1.1.1:80 reachable (%lums)", (unsigned long)ms);
    return true;
  }
  ESP_LOGW(TAG, "[OTA-CHECK] 1.1.1.1:80 unreachable (%lums) errno=%d(%s)",
           (unsigned long)ms, errno, strerror(errno));
  return false;
}

/*
 * apply_socket_opts: set TCP_NODELAY + SO_SNDTIMEO + SO_RCVTIMEO.
 *
 * SO_RCVTIMEO=30s: generous per-read timeout for early failure detection.
 *   - Normal PPP responses arrive within 1-5s, well within 30s.
 *   - If server/link goes silent, detected in 30s instead of waiting
 *     the full mbedTLS timeout_ms (120s).
 *   - When SO_RCVTIMEO fires with no data, lwIP returns EAGAIN;
 *     esp_tls maps this to ESP_TLS_ERR_SSL_WANT_READ, which the
 *     caller retries (see read_http_headers / body loop).
 */
static esp_err_t apply_socket_opts(esp_tls_t *tls) {
  int fd = -1;
  esp_tls_get_conn_sockfd(tls, &fd);
  if (fd < 0) {
    ESP_LOGE(TAG, "[OTA-TLS] esp_tls_get_conn_sockfd failed");
    return ESP_FAIL;
  }

  /* Disable Nagle ΓÇö force immediate flush of the HTTP GET request */
  int flag = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  /* Send timeout ΓÇö prevents infinite block if PPP link drops mid-write */
  struct timeval stv = {.tv_sec = 15, .tv_usec = 0};
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

  /* Recv timeout ΓÇö per-read guard; callers retry on WANT_READ */
  struct timeval rtv = {.tv_sec = 30, .tv_usec = 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));

  ESP_LOGI(
      TAG,
      "[OTA-TLS] TCP_NODELAY + SO_SNDTIMEO(15s) + SO_RCVTIMEO(30s) on fd=%d",
      fd);
  return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Block-read HTTP headers with sliding-window CRLFCRLF detector      */
/* ------------------------------------------------------------------ */
/* deadline_ms: absolute timestamp (esp_log_timestamp) after which we abort.
 * Needed because SO_RCVTIMEO is not reliable on lwIP PPP ΓÇö when the PPP
 * link is silently dead, lwIP never fires EAGAIN; we'd spin WANT_READ
 * forever until TCP RTO (~388s) finally aborts. The deadline short-circuits
 * that wait well before LCP Echo (15s) even fires. */
static esp_err_t read_http_headers(esp_tls_t *tls, char *hdr_out,
                                   int hdr_out_size, int *hdr_len_out,
                                   uint32_t deadline_ms) {
  int hdr_len = 0;
  bool done = false;
  uint8_t tail[3] = {0};

  while (hdr_len < hdr_out_size - 1) {
    int want = hdr_out_size - 1 - hdr_len;
    if (want > HDR_CHUNK_SIZE)
      want = HDR_CHUNK_SIZE;

    int rd = esp_tls_conn_read(tls, (unsigned char *)hdr_out + hdr_len, want);
    if (rd == ESP_TLS_ERR_SSL_WANT_READ || rd == ESP_TLS_ERR_SSL_WANT_WRITE) {
      /* SO_RCVTIMEO fired with no data yet ΓÇö check deadline then retry */
      if (esp_log_timestamp() >= deadline_ms) {
        hdr_out[hdr_len] = '\0';
        ESP_LOGE(TAG,
                 "[OTA-TLS] header read deadline exceeded (%lums), "
                 "read_so_far=%d bytes:[%.80s]",
                 (unsigned long)deadline_ms, hdr_len, hdr_out);
        return ESP_FAIL;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (rd <= 0) {
      int ec = 0, mf = 0;
      esp_tls_error_handle_t eh = NULL;
      esp_tls_get_error_handle(tls, &eh);
      if (eh)
        esp_tls_get_and_clear_last_error(eh, &ec, &mf);
      hdr_out[hdr_len] = '\0';
      ESP_LOGE(
          TAG,
          "[OTA-TLS] header read failed: rd=%d esp=0x%x mbed=0x%x errno=%d(%s) "
          "read_so_far=%d bytes:[%.80s]",
          rd, ec, mf, errno, strerror(errno), hdr_len, hdr_out);
      return ESP_FAIL;
    }

    for (int i = 0; i < rd && !done; i++) {
      int abs_pos = hdr_len + i;
      uint8_t w[4];
      for (int j = 0; j < 4; j++) {
        int p = abs_pos - 3 + j;
        if (p < 0)
          w[j] = tail[3 + p];
        else if (p < hdr_len)
          w[j] = (uint8_t)hdr_out[p];
        else
          w[j] = (uint8_t)hdr_out[hdr_len + (p - hdr_len)];
      }
      if (w[0] == '\r' && w[1] == '\n' && w[2] == '\r' && w[3] == '\n') {
        hdr_len += (i + 1);
        hdr_out[hdr_len] = '\0';
        done = true;
      }
    }

    if (!done) {
      hdr_len += rd;
      int ts = hdr_len - 3;
      if (ts < 0)
        ts = 0;
      for (int j = 0; j < 3; j++) {
        int s = ts + j;
        tail[j] = (s < hdr_len) ? (uint8_t)hdr_out[s] : 0;
      }
    } else {
      break;
    }
  }

  if (!done) {
    ESP_LOGE(TAG, "[OTA-TLS] header buffer overflow (%d bytes, no CRLFCRLF)",
             hdr_len);
    return ESP_FAIL;
  }

  *hdr_len_out = hdr_len;
  return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Manual OTA download over raw TLS                                   */
/* ------------------------------------------------------------------ */
static esp_err_t manual_ota_download(const char *host, const char *path) {
  esp_err_t ret = ESP_FAIL;
  esp_tls_t *tls = NULL;
  uint8_t *buf = NULL;
  esp_ota_handle_t ota_handle = 0;
  bool ota_started = false;

  tls = esp_tls_init();
  if (!tls) {
    ESP_LOGE(TAG, "[OTA-TLS] esp_tls_init OOM");
    return ESP_ERR_NO_MEM;
  }

  ble_disable_sync();

  /*
   * ALPN "http/1.1" required: Fastly CDN uses ALPN to route TLS connections.
   * timeout_ms=120000: covers the full PPP round-trip budget.
   *   - TLS handshake:     ~800ms
   *   - Server processing: ~1-3s
   *   - Firmware download: varies, but mbedTLS resets this timer per read
   *     (it is an inactivity timeout, not a total transfer timeout).
   */
  static const char *alpn[] = {"http/1.1", NULL};
  esp_tls_cfg_t tls_cfg = {
      .crt_bundle_attach = esp_crt_bundle_attach,
      .timeout_ms = 120000,
      .non_block = false,
      .alpn_protos = alpn,
  };

  uint32_t t0 = esp_log_timestamp();
  ESP_LOGI(TAG, "[OTA-TLS] connect to %s:443", host);
  int r = esp_tls_conn_new_sync(host, strlen(host), 443, &tls_cfg, tls);
  ESP_LOGI(TAG, "[OTA-TLS] connect ret=%d (%lums)", r,
           (unsigned long)(esp_log_timestamp() - t0));
  if (r != 1) {
    int ec = 0, mf = 0;
    esp_tls_error_handle_t eh = NULL;
    esp_tls_get_error_handle(tls, &eh);
    if (eh)
      esp_tls_get_and_clear_last_error(eh, &ec, &mf);
    ESP_LOGE(TAG, "[OTA-TLS] TLS connect failed esp=0x%x mbed=0x%x", ec, mf);
    goto cleanup;
  }

  if (apply_socket_opts(tls) != ESP_OK)
    goto cleanup;

  /* ---- HTTP/1.1 GET + Connection: close ----
   * curl/8.11.0 User-Agent: GitHub/Fastly whitelist curl by default.
   *   ESP32-OTA/x.x triggered Fastly's bot detection ΓåÆ silent hold
   *   (server keeps connection open but never sends HTTP response).
   * Accept-Encoding: identity: disables compression (simpler for ESP32)
   *   and changes the CDN's cache key, which may route to a different
   *   backend that is not holding the connection.
   * Cache-Control: no-cache: prevents Fastly from serving a stale
   *   "hold" response from its cache.
   */
  {
    char req[512];
    int req_len = snprintf(req, sizeof(req),
                           "GET %s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Connection: close\r\n"
                           "Accept: */*\r\n"
                           "Accept-Encoding: identity\r\n"
                           "Cache-Control: no-cache\r\n"
                           "User-Agent: curl/8.11.0\r\n"
                           "\r\n",
                           path, host);

    int written = 0;
    uint32_t tw = esp_log_timestamp();
    while (written < req_len) {
      int w = esp_tls_conn_write(tls, req + written, req_len - written);
      if (w < 0) {
        ESP_LOGE(TAG, "[OTA-TLS] write failed: %d errno=%d", w, errno);
        goto cleanup;
      }
      if (w == 0) {
        ESP_LOGE(TAG, "[OTA-TLS] write=0, peer closed");
        goto cleanup;
      }
      written += w;
    }
    ESP_LOGI(TAG, "[OTA-TLS] HTTP/1.1 GET sent (%d bytes) in %lums", req_len,
             (unsigned long)(esp_log_timestamp() - tw));
  }

  /* ---- HTTP response headers ---- */
  int content_length = -1;
  {
    char *hdr = malloc(HDR_BUF_SIZE);
    if (!hdr) {
      ret = ESP_ERR_NO_MEM;
      goto cleanup;
    }

    int hdr_len = 0;
    /* 60s hard deadline for receiving the HTTP response header.
     * Normal CDN response arrives in < 5s over PPP. If LCP Echo
     * has already torn down the link (15s), this 60s is a last resort. */
    uint32_t hdr_deadline_ms = esp_log_timestamp() + 60000;
    if (read_http_headers(tls, hdr, HDR_BUF_SIZE, &hdr_len,
                          hdr_deadline_ms) != ESP_OK) {
      free(hdr);
      goto cleanup;
    }

    ESP_LOGI(TAG, "[OTA-TLS] Response header (%d bytes):\n%.300s", hdr_len,
             hdr);

    if (strstr(hdr, " 200") == NULL) {
      ESP_LOGE(TAG, "[OTA-TLS] HTTP not 200");
      free(hdr);
      goto cleanup;
    }

    const char *cl = strstr(hdr, "Content-Length:");
    if (!cl)
      cl = strstr(hdr, "content-length:");
    if (cl)
      content_length = atoi(cl + 15);
    ESP_LOGI(TAG, "[OTA-TLS] HTTP 200 OK, Content-Length=%d", content_length);
    free(hdr);
  }

  /*
   * HTTP/1.1 + Connection: close: server closes TCP after sending body.
   * If Content-Length is missing, read until EOF (rd==0).
   * If Content-Length is present, use it for progress logging and ota_begin
   * size hint.
   */
  {
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
      ESP_LOGE(TAG, "[OTA-TLS] No OTA partition");
      goto cleanup;
    }
    ESP_LOGI(TAG, "[OTA-TLS] Writing to <%s> @ 0x%lx", update->label,
             (unsigned long)update->address);

    size_t ota_size =
        (content_length > 0) ? (size_t)content_length : OTA_SIZE_UNKNOWN;
    ret = esp_ota_begin(update, ota_size, &ota_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "[OTA-TLS] esp_ota_begin: %s", esp_err_to_name(ret));
      goto cleanup;
    }
    ota_started = true;

    buf = malloc(OTA_DL_BUF_SIZE);
    if (!buf) {
      ret = ESP_ERR_NO_MEM;
      goto cleanup;
    }

    int total = 0;
    t0 = esp_log_timestamp();
    /* Per-chunk deadline: reset on every successful read chunk.
     * 60s without a new chunk ΓåÆ link is dead, abort. */
    uint32_t body_deadline_ms = esp_log_timestamp() + 60000;
    while (1) {
      int rd = esp_tls_conn_read(tls, buf, OTA_DL_BUF_SIZE);
      if (rd == ESP_TLS_ERR_SSL_WANT_READ || rd == ESP_TLS_ERR_SSL_WANT_WRITE) {
        /* SO_RCVTIMEO fired with no data yet ΓÇö check deadline then retry */
        if (esp_log_timestamp() >= body_deadline_ms) {
          ESP_LOGE(TAG,
                   "[OTA-TLS] body read deadline exceeded (60s, no new chunk), "
                   "total=%d bytes", total);
          ret = ESP_FAIL;
          goto cleanup;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      if (rd < 0) {
        int ec = 0, mf = 0;
        esp_tls_error_handle_t eh = NULL;
        esp_tls_get_error_handle(tls, &eh);
        if (eh)
          esp_tls_get_and_clear_last_error(eh, &ec, &mf);
        ESP_LOGE(TAG,
                 "[OTA-TLS] body read error %d at %d bytes esp=0x%x mbed=0x%x "
                 "errno=%d(%s)",
                 rd, total, ec, mf, errno, strerror(errno));
        ret = ESP_FAIL;
        goto cleanup;
      }
      if (rd == 0) {
        /* EOF ΓÇö server closed connection after body (Connection: close) */
        ESP_LOGI(TAG, "[OTA-TLS] EOF at %d bytes (Content-Length=%d)", total,
                 content_length);
        break;
      }

      ret = esp_ota_write(ota_handle, buf, rd);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[OTA-TLS] esp_ota_write: %s", esp_err_to_name(ret));
        goto cleanup;
      }

      total += rd;
      /* Reset per-chunk deadline ΓÇö data is flowing normally */
      body_deadline_ms = esp_log_timestamp() + 60000;
      if ((total / (64 * 1024)) != ((total - rd) / (64 * 1024))) {
        int pct = (content_length > 0) ? (total * 100 / content_length) : -1;
        ESP_LOGI(TAG, "[OTA-TLS] %d/%d bytes (%d%%)", total, content_length,
                 pct);
      }
    }

    uint32_t dl_ms = esp_log_timestamp() - t0;
    ESP_LOGI(TAG, "[OTA-TLS] Download done: %d bytes in %lums (%ld B/s)", total,
             (unsigned long)dl_ms,
             dl_ms > 0 ? (long)(total * 1000L / dl_ms) : 0);

    if (total == 0) {
      ESP_LOGE(TAG, "[OTA-TLS] Zero bytes received");
      ret = ESP_FAIL;
      goto cleanup;
    }

    ret = esp_ota_end(ota_handle);
    ota_started = false;
    if (ret != ESP_OK) {
      if (ret == ESP_ERR_OTA_VALIDATE_FAILED)
        ESP_LOGE(TAG, "[OTA-TLS] Image validation failed (corrupted)");
      else
        ESP_LOGE(TAG, "[OTA-TLS] esp_ota_end: %s", esp_err_to_name(ret));
      goto cleanup;
    }

    ret = esp_ota_set_boot_partition(update);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "[OTA-TLS] set_boot_partition: %s", esp_err_to_name(ret));
      goto cleanup;
    }

    ESP_LOGI(TAG, "[OTA-TLS] OTA successful! Rebooting...");
    ret = ESP_OK;
  }

cleanup:
  free(buf);
  if (ota_started)
    esp_ota_abort(ota_handle);
  if (tls)
    esp_tls_conn_destroy(tls);
  return ret;
}

/* ------------------------------------------------------------------ */
/*  OTA task                                                            */
/* ------------------------------------------------------------------ */
void advanced_ota_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting Advanced OTA (direct TLS) - V2.0.0");

  /* Increase retries: with exponential backoff each attempt is more spread
   * out, giving Fastly CDN time to recover from rate limiting. */
  const int max_retries = 8;
  esp_err_t err = ESP_FAIL;

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    uint32_t t0 = esp_log_timestamp();
    ESP_LOGI(TAG, "OTA attempt %d/%d (t=%lums)", attempt, max_retries,
             (unsigned long)t0);

    /* ---- Connectivity pre-check ----
     * Verify PPP ΓåÆ internet routing is working before spending 60s on a
     * TLS attempt that Fastly might silently ignore.  Uses plain TCP (no
     * TLS) so it does NOT trigger Fastly's per-IP TLS rate limiter. */
    if (!internet_reachable()) {
      ESP_LOGW(TAG, "[OTA] Internet routing not ready, waiting 30s...");
      vTaskDelay(pdMS_TO_TICKS(30000));
      /* Count as a failed attempt so we eventually reboot instead of
       * looping forever if the PPP link itself is broken. */
      ESP_LOGE(TAG, "OTA attempt %d skipped (no internet)", attempt);
      if (attempt < max_retries) {
        uint32_t delay_ms = 30000;
        ESP_LOGW(TAG, "Retrying in %lums ...", (unsigned long)delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }
      continue;
    }

    /* Alternate between CDN hosts on each attempt.
     * Primary:  raw.githubusercontent.com  (Fastly, sometimes rate-limits)
     * Fallback: objects.githubusercontent.com (different Fastly PoP/pool)
     * Odd attempts ΓåÆ primary, even attempts ΓåÆ fallback. */
    const char *host = (attempt % 2 == 1) ? OTA_HOST_PRIMARY : OTA_HOST_FALLBACK;
    const char *path = OTA_PATH;
    ESP_LOGI(TAG, "[OTA] Using host: %s", host);

    err = manual_ota_download(host, path);


    uint32_t elapsed = esp_log_timestamp() - t0;
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "OTA attempt %d succeeded after %lums", attempt,
               (unsigned long)elapsed);
      vTaskDelay(pdMS_TO_TICKS(1000));
      esp_restart();
    }

    ESP_LOGE(TAG, "OTA attempt %d failed after %lums: %s (0x%x)", attempt,
             (unsigned long)elapsed, esp_err_to_name(err), err);

    if (attempt < max_retries) {
      /* Exponential backoff: 10s, 20s, 40s, 80s (capped), 80s, ...
       * Spreading retries reduces the TLS connection rate seen by the
       * Fastly CDN, avoiding the per-IP rate limiter that causes the
       * server to silently drop our GET requests. */
      uint32_t delay_ms;
      if (attempt == 1)      delay_ms = 10000;
      else if (attempt == 2) delay_ms = 20000;
      else if (attempt == 3) delay_ms = 40000;
      else                   delay_ms = 80000;
      ESP_LOGW(TAG, "Retrying in %lums (exp backoff attempt %d)...",
               (unsigned long)delay_ms, attempt);
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }

  ESP_LOGE(TAG, "OTA failed after %d attempts, rebooting", max_retries);
  esp_restart();
  vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */
void fota_lan_handler_task_start(void) {
  ota_task_close = false;
  get_sha256_of_partitions();

  size_t internal_free =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t internal_largest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  ESP_LOGI(TAG,
           "Heap before OTA task: total=%d, internal=%d, internal_largest=%d",
           esp_get_free_heap_size(), internal_free, internal_largest);

  const UBaseType_t ota_prio = 8; /* above WAN_DL(7) and WAN_UL(5) */
  BaseType_t ret = xTaskCreate(&advanced_ota_task, "advanced_ota_task",
                               12 * 1024, NULL, ota_prio, NULL);
  if (ret != pdPASS) {
    ESP_LOGW(TAG, "Internal RAM stack failed (largest=%d), retrying in PSRAM",
             internal_largest);
    ret = xTaskCreateWithCaps(&advanced_ota_task, "advanced_ota_task",
                              32 * 1024, NULL, ota_prio, NULL,
                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ret != pdPASS) {
      ESP_LOGE(TAG, "Failed to create OTA task");
      return;
    }
  }
  ESP_LOGI(TAG, "OTA task created successfully");
}

void fota_lan_handler_task_stop(void) { ota_task_close = true; }
