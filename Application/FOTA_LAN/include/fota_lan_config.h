#ifndef FOTA_LAN_CONFIG_H
#define FOTA_LAN_CONFIG_H

#include "driver/gpio.h"

/* Firmware upgrade URL endpoint for LAN MCU */
#define FOTA_LAN_FIRMWARE_UPGRADE_URL "https://github.com/NguyenHoangTrieu/DA2_esp_release/releases/download/V0.0.1/DA2_esp_LAN.bin"

/* Enable certificate bundle (default: enabled) */
#define FOTA_LAN_USE_CERT_BUNDLE 1

/* Skip server certificate CN field check (default: disabled) */
#define FOTA_LAN_SKIP_COMMON_NAME_CHECK 0

/* Skip firmware version check (default: disabled) */
#define FOTA_LAN_SKIP_VERSION_CHECK 0

/* Use static rx buffer for dynamic buffer after TLS handshake */
#define FOTA_LAN_TLS_DYN_BUF_RX_STATIC 0

/* OTA Receive Timeout in milliseconds */
#define FOTA_LAN_OTA_RECV_TIMEOUT 5000

/* Enable partial HTTP download (for large firmware images) */
#define FOTA_LAN_ENABLE_PARTIAL_HTTP_DOWNLOAD 0

/* HTTP request size for partial download (in bytes) */
#define FOTA_LAN_HTTP_REQUEST_SIZE 4096

/* Enable OTA resumption feature */
#define FOTA_LAN_ENABLE_OTA_RESUMPTION 0

/* HTTP Client Configuration */
#define FOTA_LAN_HTTP_BUFFER_SIZE (8 * 1024)
#define FOTA_LAN_HTTP_BUFFER_SIZE_TX (8 * 1024)

/* Enable Anti-Rollback Check */
#define FOTA_LAN_ENABLE_ANTI_ROLLBACK 0

/* OTA URL Size */
#define OTA_URL_SIZE 256

/* Enable certificate bundle support in mbedTLS */
#define MBEDTLS_CERTIFICATE_BUNDLE 1

/* Enable dynamic buffer support in mbedTLS */
#define MBEDTLS_DYNAMIC_BUFFER 1

/* eppp_link UART Configuration (client side - connects to WAN MCU server) */
/* Pins are cross-wired with WAN MCU server side */
#define FOTA_LAN_UART_TX_PIN GPIO_NUM_18       /* TX → WAN RX (GPIO 18) */
#define FOTA_LAN_UART_RX_PIN GPIO_NUM_17       /* RX ← WAN TX (GPIO 17) */
#define FOTA_LAN_UART_BAUD_RATE 115200

/* eppp_link Client Connection Timeout (ms) */
#define FOTA_LAN_EPPP_CONNECT_TIMEOUT_MS 30000 /* 30 seconds */

/* OTA Task Configuration */
#define FOTA_LAN_TASK_STACK_SIZE (32 * 1024)
#define FOTA_LAN_TASK_PRIORITY 5

/* Hash length for SHA256 */
#define HASH_LEN 32

/* OTA Commands (reserved for future separate control channel) */
#define FOTA_LAN_TRIGGER_CMD "START_OTA\n"
#define FOTA_LAN_RESPONSE_OK_CMD "OTA_LAN_OK\n"
#define FOTA_LAN_RESPONSE_FAIL_CMD "OTA_LAN_FAIL\n"

/* Command Lengths */
#define FOTA_LAN_TRIGGER_CMD_LEN (strlen(FOTA_LAN_TRIGGER_CMD))

#endif /* FOTA_LAN_CONFIG_H */
