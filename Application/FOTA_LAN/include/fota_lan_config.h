#ifndef FOTA_LAN_CONFIG_H
#define FOTA_LAN_CONFIG_H

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/* ============================================================
 * ThingsBoard OTA Server Configuration (LAN MCU)
 * ============================================================
 *
 * HOW TO SWITCH SERVERS:
 *   Local Raspberry Pi  → set USE_HTTPS=0, HOST="192.168.x.x", PORT=8080
 *   demo.thingsboard.io → set USE_HTTPS=1, HOST="demo.thingsboard.io", PORT=443
 *
 * HOW TO GET THE DEVICE TOKEN:
 *   ThingsBoard UI → Devices → Your Device → Copy Access Token
 *
 * HOW TO UPDATE FIRMWARE:
 *   ThingsBoard UI → OTA Updates → Upload new .bin → assign to Device Profile
 *   The device automatically downloads the latest assigned firmware.
 * ============================================================ */

/* 1 = local Raspberry Pi (HTTP), 0 = cloud ThingsBoard (HTTPS) */
#define FOTA_CONFIG_LAN_TB_USE_HTTPS        0

/* ThingsBoard host — change to "demo.thingsboard.io" for cloud */
#define FOTA_CONFIG_LAN_TB_HOST             "192.168.1.100"

/* Port: 8080 for local HTTP, 443 for demo.thingsboard.io HTTPS */
#define FOTA_CONFIG_LAN_TB_PORT             8080

/* Device Access Token from ThingsBoard Devices page */
#define FOTA_CONFIG_LAN_TB_DEVICE_TOKEN     "12gxik542xvkuknt5931"

/* Firmware package title and version — must match what was uploaded to
 * ThingsBoard OTA Updates. ThingsBoard returns HTTP 400 without these. */
#define FOTA_CONFIG_LAN_TB_FIRMWARE_TITLE   "DA2_esp_LAN"
#define FOTA_CONFIG_LAN_TB_FIRMWARE_VERSION "1.1.2"

/* Skip TLS certificate verification.
 * Set 1 for local server with self-signed certificate.
 * Set 0 for demo.thingsboard.io (uses public CA, cert-bundle validates it). */
#define FOTA_CONFIG_LAN_TB_SKIP_CERT_VERIFY 1

/* Build the firmware download URL automatically from the above settings.
 * ThingsBoard API: GET /api/v1/{token}/firmware  → returns latest assigned firmware */
#if FOTA_CONFIG_LAN_TB_USE_HTTPS
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL \
    "https://" FOTA_CONFIG_LAN_TB_HOST ":" STR(FOTA_CONFIG_LAN_TB_PORT) \
    "/api/v1/" FOTA_CONFIG_LAN_TB_DEVICE_TOKEN "/firmware" \
    "?title=" FOTA_CONFIG_LAN_TB_FIRMWARE_TITLE \
    "&version=" FOTA_CONFIG_LAN_TB_FIRMWARE_VERSION
#else
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL \
    "http://" FOTA_CONFIG_LAN_TB_HOST ":" STR(FOTA_CONFIG_LAN_TB_PORT) \
    "/api/v1/" FOTA_CONFIG_LAN_TB_DEVICE_TOKEN "/firmware" \
    "?title=" FOTA_CONFIG_LAN_TB_FIRMWARE_TITLE \
    "&version=" FOTA_CONFIG_LAN_TB_FIRMWARE_VERSION
#endif

/* Use cert bundle for HTTPS. Auto-disabled for plain HTTP. */
#if FOTA_CONFIG_LAN_TB_USE_HTTPS && !FOTA_CONFIG_LAN_TB_SKIP_CERT_VERIFY
#define FOTA_CONFIG_LAN_USE_CERT_BUNDLE 1
#else
#define FOTA_CONFIG_LAN_USE_CERT_BUNDLE 0
#endif

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