#ifndef FOTA_LAN_HANDLER_H
#define FOTA_LAN_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_check.h"
#include "esp_netif.h"

// eppp_link for PPP client connection
#include "eppp_link.h"

#include "driver/uart.h"
#include <string.h>

#include "fota_lan_config.h"

#ifdef FOTA_LAN_USE_CERT_BUNDLE
#include "esp_crt_bundle.h"
#endif

#if FOTA_LAN_ENABLE_ANTI_ROLLBACK
#include "esp_efuse.h"
#endif

#define FOTA_LAN_HASH_LEN 32

/**
 * @brief Initialize the FOTA LAN handler
 * 
 * Initializes the UART and starts the task/mechanism to listen
 * for the OTA trigger command from the WAN MCU via eppp_link.
 */
void fota_lan_init(void);

/**
 * @brief Deinitialize the FOTA LAN handler
 * 
 * Stops all tasks and cleans up resources.
 */
void fota_lan_deinit(void);

#endif /* FOTA_LAN_HANDLER_H */
