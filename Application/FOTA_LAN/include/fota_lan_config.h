#ifndef FOTA_LAN_CONFIG_H
#define FOTA_LAN_CONFIG_H

#include "driver/gpio.h"

/* eppp_link UART Configuration (host/client side) */
/* Pins are cross-wired with WAN MCU server side */
#define FOTA_LAN_UART_TX_PIN GPIO_NUM_18  // TX → WAN RX (GPIO 18)
#define FOTA_LAN_UART_RX_PIN GPIO_NUM_17  // RX ← WAN TX (GPIO 17)
#define FOTA_LAN_UART_BAUD_RATE 115200

/* OTA Commands (must match PPP Server) */
#define FOTA_LAN_TRIGGER_CMD "START_OTA\n"
#define FOTA_LAN_RESPONSE_OK_CMD "OTA_LAN_OK\n"
#define FOTA_LAN_RESPONSE_FAIL_CMD "OTA_LAN_FAIL\n"

/* Command Lengths */
#define FOTA_LAN_TRIGGER_CMD_LEN (strlen(FOTA_LAN_TRIGGER_CMD))

/* OTA Task Configuration */
#define FOTA_LAN_TASK_STACK_SIZE (10 * 1024)
#define FOTA_LAN_TASK_PRIORITY (5)

/* Firmware Upgrade URL */
#define FOTA_LAN_FIRMWARE_UPGRADE_URL "https://github.com/NguyenHoangTrieu/DA2_esp_release/releases/download/V0.0.1/DA2_esp_LAN.bin"

/* Advanced OTA Configuration */
#define FOTA_LAN_OTA_RECV_TIMEOUT (5000)
#define FOTA_LAN_USE_CERT_BUNDLE (1)
#define FOTA_LAN_SKIP_VERSION_CHECK (0)

/* HTTP Client Configuration */
#define FOTA_LAN_HTTP_BUFFER_SIZE (8 * 1024)
#define FOTA_LAN_HTTP_BUFFER_SIZE_TX (8 * 1024)

/* eppp_link Client Connection Timeout */
#define FOTA_LAN_EPPP_CONNECT_TIMEOUT_MS (30000)  // 30 seconds

/* OTA URL Size */
#define FOTA_LAN_OTA_URL_SIZE (256)

/* Enable Anti-Rollback Check */
#define FOTA_LAN_ENABLE_ANTI_ROLLBACK (0)

/* Skip Common Name Check */
#define FOTA_LAN_SKIP_COMMON_NAME_CHECK (0)

#endif /* FOTA_LAN_CONFIG_H */
