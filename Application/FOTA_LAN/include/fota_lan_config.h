#ifndef FOTA_LAN_CONFIG_H
#define FOTA_LAN_CONFIG_H

/* ============================================================
 * ThingsBoard OTA Server Configuration (LAN MCU)
 * ============================================================
 *
 * Set FOTA_CONFIG_LAN_FIRMWARE_URL to the full firmware download URL.
 * This is the compile-time default; the URL can be overridden at
 * runtime via the web config portal (WAN MCU) or the Python desktop
 * app by sending:  CFML:CFFW:<url>
 *
 * ThingsBoard URL format:
 *   http://<host>:<port>/api/v1/<token>/firmware?title=<title>&version=<ver>
 *
 * Examples:
 *   Local: http://192.168.1.100:8080/api/v1/TOKEN/firmware?title=DA2_esp_LAN&version=1.1.2
 *   Cloud: https://demo.thingsboard.io/api/v1/TOKEN/firmware?title=DA2_esp_LAN&version=1.1.2
 * ============================================================ */

/* Full firmware download URL — override via web config or Python app at runtime */
#define FOTA_CONFIG_LAN_FIRMWARE_URL \
    "https://github.com/NguyenHoangTrieu/DA2_esp_release/releases/download/V0.0.1/DA2_esp_LAN.bin"

/* Maximum URL length stored at runtime */
#define FOTA_CONFIG_LAN_FIRMWARE_URL_MAX_LEN  256

/* Use cert bundle for HTTPS URLs.
 * Disabled for plain HTTP; enable if you switch to an https:// URL. */
#define FOTA_CONFIG_LAN_USE_CERT_BUNDLE 0

/* Firmware upgrade URL from stdin (set to 1 if URL is "FROM_STDIN") */
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL_FROM_STDIN 0

/* Skip server certificate CN field check (default: disabled) */
#define FOTA_CONFIG_LAN_SKIP_COMMON_NAME_CHECK 0

/* Skip firmware version check (default: disabled) */
#define FOTA_CONFIG_LAN_SKIP_VERSION_CHECK 0

/* Support firmware upgrade bind specified interface (default: disabled) */
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF 0

/* OTA data bind interface selection */
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_STA 0
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_BIND_IF_ETH 0

/* Use static rx buffer for dynamic buffer after TLS handshake */
#define FOTA_CONFIG_LAN_TLS_DYN_BUF_RX_STATIC 0

/* Enable WiFi connection */
#define FOTA_CONFIG_LAN_CONNECT_WIFI 1

/* -----------------------------------------------------------------------
 * FOTA WiFi AP credentials (must match DA2_esp/Application/FOTA/include/fota_ap.h)
 * The WAN MCU broadcasts this AP specifically for LAN MCU firmware updates.
 * ----------------------------------------------------------------------- */
#define FOTA_CONFIG_LAN_WIFI_AP_SSID   "DA2-FOTA"
#define FOTA_CONFIG_LAN_WIFI_AP_PASS   "da2fota1"
#define FOTA_CONFIG_LAN_WIFI_CONNECT_TIMEOUT_MS  30000

/* Enable Ethernet connection */
#define FOTA_CONFIG_LAN_CONNECT_ETHERNET 0

/* OTA Receive Timeout in milliseconds.
 * 30s is plenty for a local HTTP server — file is ~1.6 MB over LAN. */
#define FOTA_CONFIG_LAN_OTA_RECV_TIMEOUT 300000

/* TCP connect timeout for connectivity pre-check (ms) */
#define FOTA_CONFIG_LAN_CONNECTIVITY_CHECK_TIMEOUT_MS 5000

/* Enable partial HTTP download (for large firmware images) */
#define FOTA_CONFIG_LAN_ENABLE_PARTIAL_HTTP_DOWNLOAD 0

/* HTTP request size for partial download (in bytes) */
#define FOTA_CONFIG_LAN_HTTP_REQUEST_SIZE 4096

/* Enable OTA resumption feature */
#define FOTA_CONFIG_LAN_ENABLE_OTA_RESUMPTION 0

#if FOTA_CONFIG_LAN_CONNECT_WIFI
#define NETIF_DESC_STA "netif_sta"
#endif

#if FOTA_CONFIG_LAN_CONNECT_ETHERNET
#define NETIF_DESC_ETH "netif_eth"
#endif

/* Enable certificate bundle support in mbedTLS */
#define MBEDTLS_CERTIFICATE_BUNDLE 1

/* Enable dynamic buffer support in mbedTLS */
#define MBEDTLS_DYNAMIC_BUFFER 1

#endif /* CONFIG_H */