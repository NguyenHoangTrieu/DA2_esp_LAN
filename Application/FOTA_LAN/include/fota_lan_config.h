#ifndef FOTA_LAN_CONFIG_H
#define FOTA_LAN_CONFIG_H

/* Firmware upgrade URL endpoint */
#define FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL "https://github.com/NguyenHoangTrieu/DA2_esp_release/releases/download/V0.0.1/DA2_esp_LAN.bin"

/* Enable certificate bundle (default: enabled) */
#define FOTA_CONFIG_LAN_USE_CERT_BUNDLE 1

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

/* Enable Ethernet connection */
#define FOTA_CONFIG_LAN_CONNECT_ETHERNET 0

/* OTA Receive Timeout in milliseconds */
#define FOTA_CONFIG_LAN_OTA_RECV_TIMEOUT 5000

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

// Transport Selection
#define PPP_USE_UART_TRANSPORT         1

// UART Configuration
#define PPP_UART_PORT                  UART_NUM_0
#define PPP_UART_TX_PIN                GPIO_NUM_43
#define PPP_UART_RX_PIN                GPIO_NUM_44
#define PPP_UART_BAUDRATE              256000
#define PPP_UART_QUEUE_SIZE            20
#define PPP_UART_RX_BUFFER_SIZE        16*1024

// Global DNS Server (8.8.8.8)
#define PPP_GLOBAL_DNS                 0x08080808

#endif /* CONFIG_H */
