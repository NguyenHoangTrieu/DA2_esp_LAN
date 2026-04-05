#ifndef FOTA_HANDLER_H
#define FOTA_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_check.h"
#include "string.h"
#include "fota_lan_config.h"

#ifdef FOTA_CONFIG_LAN_USE_CERT_BUNDLE
#include "esp_crt_bundle.h"
#endif

#if FOTA_CONFIG_LAN_BOOTLOADER_APP_ANTI_ROLLBACK
#include "esp_efuse.h"
#endif

#include "nvs.h"
#include "nvs_flash.h"
#include <sys/socket.h>

#if FOTA_CONFIG_LAN_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#define HASH_LEN 32
#define OTA_URL_SIZE 256

void fota_lan_handler_task_start(void);
void fota_lan_handler_task_stop(void);

#endif /* FOTA_HANDLER_H */