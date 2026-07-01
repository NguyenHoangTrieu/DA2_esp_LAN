#ifndef MAIN_DA2_ESP_LAN_H
#define MAIN_DA2_ESP_LAN_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "ble_handler.h"
#include "ble_native_handler.h"
#include "ble_gatt_handler.h"
#include "config_handler.h"
#include "mcu_wan_handler.h"
#include "fota_lan_handler.h"
#include "module_monitor_task.h"
#include "i2c_dev_support.h"
#include "tca_handler.h"
#include "stack_handler.h"

#endif /* MAIN_DA2_ESP_LAN_H */