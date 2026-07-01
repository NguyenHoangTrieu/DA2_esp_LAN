/*
 * Advanced OTA Update Handler for ESP32
 * Downloads firmware from ThingsBoard via the WAN MCU's FOTA WiFi AP.
 *
 * Flow:
 *   1. BLE disabled (frees RF resources for WiFi).
 *   2. Connect WiFi STA to WAN MCU's "DA2-FOTA" AP.
 *   3. WAN MCU NATs traffic to internet → download from ThingsBoard.
 *   4. Flash firmware from PSRAM buffer (no PPP disruption possible).
 *   5. WiFi disconnected, device reboots into new firmware.
 */
#include "fota_lan_handler.h"
#include "config_handler.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <fcntl.h>
#include <sys/param.h>


static const char *TAG = "lan_advanced_ota";

/* Download buffer for esp_http_client streaming */
#define OTA_DL_BUF_SIZE 4096
#define OTA_MAIN_TASK_STACK_SIZE  (20 * 1024)
#define OTA_FLASH_TASK_STACK_SIZE (12 * 1024)

/* ------------------------------------------------------------------ */
/*  FOTA WiFi AP connect / disconnect                                   */
/* ------------------------------------------------------------------ */
static EventGroupHandle_t s_wifi_eg     = NULL;
static esp_netif_t       *s_wifi_netif  = NULL;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static void fota_wifi_event_handler(void *arg, esp_event_base_t base,
                                    int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "[WiFi] Disconnected from FOTA AP");
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "[WiFi] FOTA AP connected, IP=" IPSTR,
                 IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* fota_wifi_start() — MUST be called from a task whose stack is in
 * internal RAM (not PSRAM).  esp_wifi_init() reads NVS calibration data
 * via spi_flash, which calls spi_flash_disable_interrupts_caches_and_other_cpu().
 * That function asserts if the current task stack is in PSRAM because
 * the stack becomes inaccessible when the flash cache is disabled.
 *
 * Call this from fota_lan_handler_task_start() (runs on the config_handler
 * task stack, which is always in internal RAM).  The OTA task then only
 * calls fota_wifi_wait_connected() which is a plain event-group wait. */
static esp_err_t fota_wifi_start(void)
{
    s_wifi_eg = xEventGroupCreate();
    if (!s_wifi_eg) return ESP_ERR_NO_MEM;

    /* Initialise the network stack (must be called once before any netif).
     * Returns ESP_ERR_INVALID_STATE if already initialised — ignore that. */
    esp_err_t ni = esp_netif_init();
    if (ni != ESP_OK && ni != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "[WiFi] esp_netif_init failed: %s", esp_err_to_name(ni));
        return ni;
    }

    /* netif must be created before esp_wifi_init — esp_wifi_init reads NVS
     * (flash) so this entire function must run on an internal-RAM stack. */
    s_wifi_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                fota_wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                fota_wifi_event_handler, NULL);

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = FOTA_CONFIG_LAN_WIFI_AP_SSID,
            .password = FOTA_CONFIG_LAN_WIFI_AP_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        .listen_interval = 1,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    /* esp_wifi_start() fires WIFI_EVENT_STA_START in the esp_event_loop
     * task (internal RAM stack) → our handler calls esp_wifi_connect(). */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* OTA is throughput-sensitive; keep the STA awake while pulling the image
     * through the WAN MCU's SoftAP instead of using the default modem sleep. */
    esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_ret != ESP_OK) {
      ESP_LOGW(TAG, "[WiFi] Failed to disable power save for OTA: %s",
           esp_err_to_name(ps_ret));
    } else {
      ESP_LOGI(TAG, "[WiFi] WiFi power save disabled for OTA download");
    }

    ESP_LOGI(TAG, "[WiFi] WiFi started, FOTA AP association in progress...");
    return ESP_OK;
}

/* fota_wifi_wait_connected() — waits for the IP event set by the event
 * handler.  Safe to call from a PSRAM-stacked task (no flash ops here). */
static esp_err_t fota_wifi_wait_connected(void)
{
    if (!s_wifi_eg) {
        ESP_LOGE(TAG, "[WiFi] Event group not created — fota_wifi_start() not called");
        return ESP_FAIL;
    }
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(FOTA_CONFIG_LAN_WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "[WiFi] Connected to FOTA AP \"%s\"",
                 FOTA_CONFIG_LAN_WIFI_AP_SSID);
        /* Override DNS with a public server — the FOTA AP (192.168.4.1) has
         * no DNS proxy; NAPT will forward UDP/53 to 8.8.8.8 over the WAN
         * MCU's internet connection.  Must be set AFTER IP is assigned. */
        ip_addr_t dns_primary   = IPADDR4_INIT_BYTES(8, 8, 8, 8);
        ip_addr_t dns_secondary = IPADDR4_INIT_BYTES(8, 8, 4, 4);
        dns_setserver(0, &dns_primary);
        dns_setserver(1, &dns_secondary);
        ESP_LOGI(TAG, "[WiFi] DNS overridden to 8.8.8.8 / 8.8.4.4");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "[WiFi] Failed to connect to FOTA AP (timeout or rejected)");
    return ESP_FAIL;
}

static void fota_wifi_disconnect(void)
{
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, fota_wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT,   IP_EVENT_STA_GOT_IP, fota_wifi_event_handler);
    esp_wifi_stop();
    esp_wifi_deinit();
    if (s_wifi_netif) {
        esp_netif_destroy(s_wifi_netif);
        s_wifi_netif = NULL;
    }
    if (s_wifi_eg) {
        vEventGroupDelete(s_wifi_eg);
        s_wifi_eg = NULL;
    }
    ESP_LOGI(TAG, "[WiFi] Disconnected from FOTA AP");
}


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

/* -------------------------------------------------------------------
 * Runtime firmware download URL
 * Default = FOTA_CONFIG_LAN_FIRMWARE_URL from fota_lan_config.h.
 * Overridden at runtime by fota_lan_handler_set_url() when the WAN MCU
 * sends "CFML:CFFW:<url>" via web config or Python app.
 * ------------------------------------------------------------------- */
static char s_fota_url[FOTA_CONFIG_LAN_FIRMWARE_URL_MAX_LEN] =
    FOTA_CONFIG_LAN_FIRMWARE_URL;

void fota_lan_handler_set_url(const char *url)
{
    if (!url || url[0] == '\0') return;
    strncpy(s_fota_url, url, sizeof(s_fota_url) - 1);
    s_fota_url[sizeof(s_fota_url) - 1] = '\0';
    ESP_LOGI("lan_advanced_ota", "[OTA] Firmware URL updated: %s", s_fota_url);
    config_save_fota_lan_url_to_nvs();
}

const char *fota_lan_handler_get_url(void)
{
    return s_fota_url;
}

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
/*  Server connectivity pre-check: parse host:port from runtime URL    */
/* ------------------------------------------------------------------ */
static bool internet_reachable(void) {
  const char *url = fota_lan_handler_get_url();

  /* ── Parse host and port from the URL ─────────────────────────── */
  const char *after_scheme = strstr(url, "://");
  if (!after_scheme) {
    ESP_LOGW(TAG, "[OTA-CHECK] Malformed URL: %s", url);
    return false;
  }
  after_scheme += 3;

  char host[128] = {0};
  char port_str[8] = "80";
  const char *slash = strchr(after_scheme, '/');
  const char *colon = strchr(after_scheme, ':');
  if (colon && (!slash || colon < slash)) {
    int hlen = (int)(colon - after_scheme);
    if (hlen >= (int)sizeof(host)) hlen = (int)sizeof(host) - 1;
    memcpy(host, after_scheme, hlen);
    int plen = slash ? (int)(slash - colon - 1) : (int)strlen(colon + 1);
    if (plen > 0 && plen < (int)sizeof(port_str))
      memcpy(port_str, colon + 1, plen);
  } else {
    int hlen = slash ? (int)(slash - after_scheme) : (int)strlen(after_scheme);
    if (hlen >= (int)sizeof(host)) hlen = (int)sizeof(host) - 1;
    memcpy(host, after_scheme, hlen);
    if (strncmp(url, "https", 5) == 0) strcpy(port_str, "443");
  }

  int port_num = atoi(port_str);
  ESP_LOGI(TAG, "[OTA-CHECK] Starting connectivity check to %s:%d", host, port_num);

  struct sockaddr_in server_addr = {0};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons((uint16_t)port_num);

  if (inet_aton(host, &server_addr.sin_addr) == 1) {
    ESP_LOGI(TAG, "[OTA-CHECK] Host is IP address, skipping DNS");
  } else {
    ESP_LOGI(TAG, "[OTA-CHECK] Resolving hostname %s via DNS...", host);
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *res = NULL;
    int gai_err = getaddrinfo(host, port_str, &hints, &res);
    if (gai_err != 0 || !res) {
      ESP_LOGW(TAG, "[OTA-CHECK] DNS failed: %d", gai_err);
      return false;
    }
    struct sockaddr_in *addr_in = (struct sockaddr_in *)res->ai_addr;
    server_addr.sin_addr = addr_in->sin_addr;
    freeaddrinfo(res);
  }

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ESP_LOGE(TAG, "[OTA-CHECK] socket() failed: errno=%d", errno);
    return false;
  }
  ESP_LOGI(TAG, "[OTA-CHECK] socket created (fd=%d), setting timeouts...", sock);

  int timeout_ms = FOTA_CONFIG_LAN_CONNECTIVITY_CHECK_TIMEOUT_MS;
  ESP_LOGI(TAG, "[OTA-CHECK] timeout=%dms, attempting connect...", timeout_ms);

  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  uint32_t t0 = esp_log_timestamp();
  int r = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (r < 0 && errno == EINPROGRESS) {
    struct timeval tv = {.tv_sec  = timeout_ms / 1000,
                         .tv_usec = (timeout_ms % 1000) * 1000};
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    int sel = select(sock + 1, NULL, &wfds, NULL, &tv);
    if (sel > 0) {
      int so_err = 0;
      socklen_t slen = sizeof(so_err);
      getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_err, &slen);
      r = (so_err == 0) ? 0 : -1;
      if (so_err) errno = so_err;
    } else {
      r = -1;
      errno = (sel == 0) ? ETIMEDOUT : errno;
    }
  }
  uint32_t ms = esp_log_timestamp() - t0;
  ESP_LOGI(TAG, "[OTA-CHECK] connect() returned %d after %lums (errno=%d)",
           r, (unsigned long)ms, (r < 0) ? errno : 0);
  close(sock);

  if (r == 0) {
    ESP_LOGI(TAG, "[OTA-CHECK] %s:%d reachable (%lums)", host, port_num, (unsigned long)ms);
    return true;
  }
  ESP_LOGW(TAG, "[OTA-CHECK] %s:%d unreachable (%lums) errno=%d",
           host, port_num, (unsigned long)ms, errno);
  return false;
}


/* ------------------------------------------------------------------ */
/*  OTA download via esp_http_client (ThingsBoard) — PSRAM-first        */
/*                                                                      */
/*  PHASE 1 — DOWNLOAD TO PSRAM:                                        */
/*    Open HTTP and stream all bytes to a PSRAM buffer.  Zero flash     */
/*    operations in this phase → eppp_link/LwIP run uninterrupted at   */
/*    full TCP bandwidth.                                               */
/*  PHASE 2 — FLASH FROM PSRAM:                                         */
/*    Close HTTP first, then call esp_ota_begin()+write().  Flash erase */
/*    (~27s) disables the cache and starves the eppp_link task, but     */
/*    the download is already complete so PPP disruption is harmless.   */
/*                                                                      */
/*  Previous attempts at streaming download+flash simultaneously all   */
/*  failed because esp_ota_begin/write disables the instruction cache   */
/*  during sector erases, preventing eppp_link from draining the UART, */
/*  which causes UART FIFO overflow and TCP stall within seconds.       */
/*  The PSRAM approach was tried before but failed due to a separate    */
/*  bug (eppp_link priority 19 > LwIP 18 starved the tcpip mailbox,    */
/*  causing socket() to block indefinitely).  That is now fixed.       */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/*  Event handler to capture redirect Location header                   */
/* ------------------------------------------------------------------ */
static char s_lan_location_header[2048];

typedef struct {
  TaskHandle_t waiter;
  uint8_t *fw_buf;
  size_t total;
  esp_err_t result;
} ota_flash_task_args_t;

static esp_err_t lan_ota_http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_HEADER) {
        if (strcasecmp(evt->header_key, "Location") == 0) {
            strlcpy(s_lan_location_header, evt->header_value,
                    sizeof(s_lan_location_header));
        }
    }
    return ESP_OK;
}

static esp_err_t flash_from_psram_buffer(uint8_t *fw_buf, size_t total)
{
  const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
  if (!update) {
    ESP_LOGE(TAG, "[OTA] No OTA partition available");
    return ESP_FAIL;
  }

  esp_ota_handle_t ota_handle = 0;
  ESP_LOGI(TAG, "[OTA] HTTP closed. Erasing OTA partition on internal-RAM helper task...");
  esp_err_t err = esp_ota_begin(update, total, &ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[OTA] esp_ota_begin: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "[OTA] Writing %u bytes from PSRAM to flash...", (unsigned int)total);
  for (size_t offset = 0; offset < total; offset += OTA_DL_BUF_SIZE) {
    size_t chunk = MIN((size_t)OTA_DL_BUF_SIZE, total - offset);
    err = esp_ota_write(ota_handle, fw_buf + offset, chunk);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "[OTA] esp_ota_write @%u: %s", (unsigned int)offset,
               esp_err_to_name(err));
      esp_ota_abort(ota_handle);
      return err;
    }
    if ((offset % (256 * 1024)) < OTA_DL_BUF_SIZE) {
      ESP_LOGI(TAG, "[OTA] Flash: %u / %u B", (unsigned int)(offset + chunk),
               (unsigned int)total);
    }
  }

  err = esp_ota_end(ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[OTA] esp_ota_end: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_ota_set_boot_partition(update);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[OTA] set_boot_partition: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "[OTA] Flash complete! Rebooting into new firmware...");
  return ESP_OK;
}

static void ota_flash_task(void *pvParameter)
{
  ota_flash_task_args_t *args = (ota_flash_task_args_t *)pvParameter;
  args->result = flash_from_psram_buffer(args->fw_buf, args->total);
  xTaskNotifyGive(args->waiter);
  vTaskDelete(NULL);
}

static esp_err_t manual_ota_download(void) {
  const char *fw_url = fota_lan_handler_get_url();
  ESP_LOGI(TAG, "[OTA] Downloading firmware from: %s", fw_url);

  esp_err_t ret = ESP_FAIL;
  uint8_t *fw_buf = NULL;
  ota_flash_task_args_t *flash_args = NULL;

  /* Resolve redirects first (GitHub → CDN) */
  static char s_final_url[2048];
  strlcpy(s_final_url, fw_url, sizeof(s_final_url));

  for (int redir = 0; redir < 5; redir++) {
    s_lan_location_header[0] = '\0';
    bool cur_https = (strncmp(s_final_url, "https://", 8) == 0);
    esp_http_client_config_t probe_cfg = {
        .url               = s_final_url,
        .timeout_ms        = 10000,
        .buffer_size       = 2048,
        .buffer_size_tx    = 512,
        .keep_alive_enable = false,
        .event_handler     = lan_ota_http_event_handler,
        .crt_bundle_attach = cur_https ? esp_crt_bundle_attach : NULL,
    };
    esp_http_client_handle_t probe = esp_http_client_init(&probe_cfg);
    if (!probe) { ESP_LOGE(TAG, "[OTA] probe init failed"); return ESP_FAIL; }
    esp_http_client_open(probe, 0);
    esp_http_client_fetch_headers(probe);
    int pstatus = esp_http_client_get_status_code(probe);
    esp_http_client_cleanup(probe);
    if (pstatus == 301 || pstatus == 302 || pstatus == 307 || pstatus == 308) {
      if (s_lan_location_header[0] == '\0') {
        ESP_LOGE(TAG, "[OTA] Redirect %d: no Location header", redir + 1);
        return ESP_FAIL;
      }
      ESP_LOGI(TAG, "[OTA] Redirect %d (%d) → %s", redir + 1, pstatus, s_lan_location_header);
      strlcpy(s_final_url, s_lan_location_header, sizeof(s_final_url));
    } else {
      break;
    }
  }

  ESP_LOGI(TAG, "[OTA] Final URL: %s", s_final_url);
  bool use_https = (strncmp(s_final_url, "https://", 8) == 0);

  esp_http_client_config_t http_cfg = {
      .url               = s_final_url,
      .timeout_ms        = FOTA_CONFIG_LAN_OTA_RECV_TIMEOUT,
      .buffer_size       = OTA_DL_BUF_SIZE,
      .buffer_size_tx    = 2048,
      .keep_alive_enable = false,
      .crt_bundle_attach = use_https ? esp_crt_bundle_attach : NULL,
  };

  esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
  if (!client) {
    ESP_LOGE(TAG, "[OTA] esp_http_client_init failed");
    return ESP_FAIL;
  }

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[OTA] HTTP open failed: %s", esp_err_to_name(err));
    goto cleanup_http;
  }

  int64_t content_len = esp_http_client_fetch_headers(client);
  int http_status = esp_http_client_get_status_code(client);
  ESP_LOGI(TAG, "[OTA] HTTP %d, Content-Length: %lld", http_status, content_len);

  if (http_status != 200) {
    ESP_LOGE(TAG, "[OTA] HTTP error %d", http_status);
    goto cleanup_http;
  }
  if (content_len <= 0 || content_len > (4 * 1024 * 1024)) {
    ESP_LOGE(TAG, "[OTA] Invalid content_len: %lld", content_len);
    goto cleanup_http;
  }

  fw_buf = (uint8_t *)heap_caps_malloc((size_t)content_len,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fw_buf) {
    ESP_LOGE(TAG, "[OTA] PSRAM alloc failed for %lld bytes", content_len);
    goto cleanup_http;
  }
  ESP_LOGI(TAG, "[OTA] PSRAM buf allocated (%lld B). Downloading (no flash ops)...",
           content_len);

  /* ---- Phase 1: stream to PSRAM, zero flash operations ---- */
  int total = 0;
  uint32_t t0 = esp_log_timestamp();
  while (total < (int)content_len) {
    int want = MIN(OTA_DL_BUF_SIZE, (int)content_len - total);
    int len = esp_http_client_read(client, (char *)(fw_buf + total), want);
    if (len == 0) break;
    if (len < 0) {
      ESP_LOGE(TAG, "[OTA] Read error: %d (got %d/%lld)", len, total, content_len);
      goto cleanup_http;
    }
    total += len;
    if (total % (64 * 1024) < OTA_DL_BUF_SIZE) {
      ESP_LOGI(TAG, "[OTA] Download: %d / %lld B (%.1f%%)",
               total, content_len, 100.0f * total / content_len);
    }
    taskYIELD(); /* yield to LwIP between reads */
  }
  {
    uint32_t dl_ms = esp_log_timestamp() - t0;
    ESP_LOGI(TAG, "[OTA] Download done: %d B in %lums (%ld B/s)",
             total, (unsigned long)dl_ms,
             dl_ms ? (long)((int64_t)total * 1000 / dl_ms) : 0);
  }
  if (total != (int)content_len) {
    ESP_LOGE(TAG, "[OTA] Incomplete: got %d, expected %lld", total, content_len);
    goto cleanup_http;
  }

  /* Close HTTP before touching flash */
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  client = NULL;

  /* ---- Phase 2: erase + flash from PSRAM on a small internal-RAM stack ---- */
  flash_args = (ota_flash_task_args_t *)heap_caps_malloc(
      sizeof(*flash_args), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!flash_args) {
    ESP_LOGE(TAG, "[OTA] Failed to allocate flash task args");
    goto cleanup_buf;
  }
  flash_args->waiter = xTaskGetCurrentTaskHandle();
  flash_args->fw_buf = fw_buf;
  flash_args->total = (size_t)total;
  flash_args->result = ESP_FAIL;

  size_t internal_largest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  ESP_LOGI(TAG,
           "[OTA] Spawning internal flash helper (internal_largest=%u, stack=%u)",
           (unsigned int)internal_largest, (unsigned int)OTA_FLASH_TASK_STACK_SIZE);
  BaseType_t flash_task_ret = xTaskCreate(ota_flash_task, "ota_flash_task",
                                          OTA_FLASH_TASK_STACK_SIZE, flash_args,
                                          uxTaskPriorityGet(NULL), NULL);
  if (flash_task_ret != pdPASS) {
    ESP_LOGE(TAG, "[OTA] Failed to create flash helper task (internal largest=%u)",
             (unsigned int)internal_largest);
    goto cleanup_flash_args;
  }

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  err = flash_args->result;
  if (err != ESP_OK) {
    goto cleanup_flash_args;
  }

  ret = ESP_OK;

cleanup_flash_args:
  free(flash_args);

cleanup_buf:
  free(fw_buf);
  return ret;

cleanup_http:
  free(fw_buf);
  if (client) {
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
  }
  return ret;
}


/* ------------------------------------------------------------------ */
/*  OTA task                                                            */
/* ------------------------------------------------------------------ */
void advanced_ota_task(void *pvParameter) {
  ESP_LOGI(TAG, "Starting Advanced OTA (ThingsBoard via FOTA WiFi AP) - V4.0.0");
  ESP_LOGI(TAG, "[OTA] Target: %s", fota_lan_handler_get_url());

  /* Wait for WiFi association + DHCP lease (started in task_start). */
  ESP_LOGI(TAG, "[OTA] Waiting for WiFi connection to FOTA AP \"%s\"...",
           FOTA_CONFIG_LAN_WIFI_AP_SSID);
  if (fota_wifi_wait_connected() != ESP_OK) {
    ESP_LOGE(TAG, "[OTA] Cannot reach FOTA AP — aborting");
    fota_wifi_disconnect();
    esp_restart();
    vTaskDelete(NULL);
    return;
  }

  const int max_retries = 5;
  esp_err_t err = ESP_FAIL;

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    uint32_t t0 = esp_log_timestamp();
    ESP_LOGI(TAG, "OTA attempt %d/%d (t=%lums)", attempt, max_retries,
             (unsigned long)t0);

    /* Quick reachability check against the ThingsBoard host. */
    if (!internet_reachable()) {
      ESP_LOGW(TAG, "[OTA] ThingsBoard not reachable, waiting 10s...");
      vTaskDelay(pdMS_TO_TICKS(10000));
      ESP_LOGE(TAG, "OTA attempt %d skipped (no route to ThingsBoard)", attempt);
      continue;
    }

    err = manual_ota_download();

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
      uint32_t delay_ms = 30000;
      ESP_LOGW(TAG, "Retrying in %lums ...", (unsigned long)delay_ms);
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }

  ESP_LOGE(TAG, "OTA failed after %d attempts, rebooting", max_retries);
  fota_wifi_disconnect();
  esp_restart();
  vTaskDelete(NULL);
}


/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */
void fota_lan_handler_task_start(void) {
  ota_task_close = false;
  get_sha256_of_partitions();
  /* BLE must be off before using WiFi — they share the RF radio.
  * fota_wifi_start() was already called by fota_lan_handler_task_start()
  * (on an internal-RAM stack) before this task was created, so WiFi init
  * and association are already in progress.  We just wait for the IP. */
  ble_disable_sync();
  
  size_t internal_free =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t internal_largest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t psram_free =
      heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    size_t psram_largest =
      heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ESP_LOGI(TAG,
         "Heap before OTA task: total=%d, internal=%d, internal_largest=%d, psram=%d, psram_largest=%d",
         esp_get_free_heap_size(), internal_free, internal_largest,
         psram_free, psram_largest);

  /* Start WiFi on THIS stack (config_handler task, internal RAM) before
   * creating the OTA task.  esp_wifi_init() reads NVS via spi_flash and
   * calls spi_flash_disable_interrupts_caches_and_other_cpu() internally.
   * That function asserts if the calling task stack is in PSRAM.
   * The OTA task only calls fota_wifi_wait_connected() which is safe. */
  ESP_LOGI(TAG, "[OTA] Starting WiFi (internal-RAM context)...");
  if (fota_wifi_start() != ESP_OK) {
    ESP_LOGE(TAG, "[OTA] WiFi start failed — aborting FOTA");
    return;
  }

  /* Keep the main OTA task in PSRAM to avoid large internal-RAM stack
   * allocation failures. Flash erase/write still run on a separate internal-
   * RAM helper task because esp_ota_begin/write can disable flash cache. */
  const UBaseType_t ota_prio = 5;
  BaseType_t ret = xTaskCreateWithCaps(&advanced_ota_task, "advanced_ota_task",
                                       OTA_MAIN_TASK_STACK_SIZE, NULL,
                                       ota_prio, NULL,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ret != pdPASS) {
    ESP_LOGE(TAG,
             "Failed to create OTA task in PSRAM (internal largest=%d, psram largest=%d) — aborting",
             internal_largest, psram_largest);
    fota_wifi_disconnect();
    return;
  }
  ESP_LOGI(TAG, "OTA task created (%uB PSRAM stack + internal flash helper)",
           (unsigned int)OTA_MAIN_TASK_STACK_SIZE);
}

void fota_lan_handler_task_stop(void) { ota_task_close = true; }