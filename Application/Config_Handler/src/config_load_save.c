/**
 * @file config_load_save.c
 * @brief Configuration Load/Save for MCU LAN (CAN, LoRa, Thread, Zigbee)
 */

#include "config_handler.h"
#include "can_driver.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "CONFIG_NVS";

/* NVS Namespace */
#define NVS_NAMESPACE "lan_gateway"

/* NVS Keys */
#define NVS_KEY_INITIALIZED "initialized"
#define NVS_KEY_CAN_CONFIG "can_cfg"
#define NVS_KEY_CAN_WHITELIST "can_wlist"

/* CAN Whitelist Size */
#define CAN_MAX_WHITELIST_SIZE MAX_WHITELISTED_IDS

/* External global variables from can_driver */
extern can_config_t g_can_config;
extern uint16_t g_can_whitelist[];
extern uint8_t g_whitelist_count;

/**
 * @brief Open NVS handle
 */
static esp_err_t nvs_open_handle(nvs_handle_t *handle) {
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 * @brief Load CAN configuration from NVS
 */
static esp_err_t load_can_config_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    ESP_LOGI(TAG, "Loading CAN config from NVS...");
    
    err = nvs_open_handle(&nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Read CAN config (baud_rate and operating_mode)
    typedef struct {
        uint32_t baud_rate;
        can_operating_mode_t operating_mode;
    } can_config_persistent_t;

    can_config_persistent_t can_cfg;
    size_t required_size = sizeof(can_config_persistent_t);
    
    err = nvs_get_blob(nvs_handle, NVS_KEY_CAN_CONFIG, &can_cfg, &required_size);
    
    if (err == ESP_OK) {
        // Copy to global config
        g_can_config.baud_rate = can_cfg.baud_rate;
        g_can_config.operating_mode = can_cfg.operating_mode;
        ESP_LOGI(TAG, "CAN config loaded - Baud: %lu, Mode: %d", 
                 g_can_config.baud_rate, g_can_config.operating_mode);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "CAN config not found in NVS, using defaults");
        err = ESP_OK; // Not an error, use defaults
    } else {
        ESP_LOGE(TAG, "Error reading CAN config: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Read CAN whitelist
    typedef struct {
        uint16_t ids[CAN_MAX_WHITELIST_SIZE];
        uint8_t count;
    } can_whitelist_persistent_t;

    can_whitelist_persistent_t whitelist;
    required_size = sizeof(can_whitelist_persistent_t);
    
    err = nvs_get_blob(nvs_handle, NVS_KEY_CAN_WHITELIST, &whitelist, &required_size);
    
    if (err == ESP_OK) {
        // Copy to global whitelist
        g_whitelist_count = whitelist.count;
        if (g_whitelist_count > CAN_MAX_WHITELIST_SIZE) {
            ESP_LOGW(TAG, "Whitelist count %d exceeds max %d, truncating", 
                     g_whitelist_count, CAN_MAX_WHITELIST_SIZE);
            g_whitelist_count = CAN_MAX_WHITELIST_SIZE;
        }
        
        memcpy(g_can_whitelist, whitelist.ids, g_whitelist_count * sizeof(uint16_t));
        
        ESP_LOGI(TAG, "CAN whitelist loaded - Count: %d", g_whitelist_count);
        for (uint8_t i = 0; i < g_whitelist_count; i++) {
            ESP_LOGI(TAG, "  ID[%d]: 0x%03X", i, g_can_whitelist[i]);
        }
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "CAN whitelist not found in NVS, using empty whitelist (accept all)");
        g_whitelist_count = 0;
        err = ESP_OK; // Not an error, use empty whitelist
    } else {
        ESP_LOGE(TAG, "Error reading CAN whitelist: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Save CAN configuration to NVS
 */
esp_err_t save_can_config_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    ESP_LOGI(TAG, "Saving CAN config to NVS...");
    
    err = nvs_open_handle(&nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Prepare CAN config data
    typedef struct {
        uint32_t baud_rate;
        can_operating_mode_t operating_mode;
    } can_config_persistent_t;

    can_config_persistent_t can_cfg = {
        .baud_rate = g_can_config.baud_rate,
        .operating_mode = g_can_config.operating_mode
    };

    // Write CAN config blob
    err = nvs_set_blob(nvs_handle, NVS_KEY_CAN_CONFIG, &can_cfg,
                       sizeof(can_config_persistent_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing CAN config: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Prepare whitelist data
    typedef struct {
        uint16_t ids[CAN_MAX_WHITELIST_SIZE];
        uint8_t count;
    } can_whitelist_persistent_t;

    can_whitelist_persistent_t whitelist = {0};
    whitelist.count = g_whitelist_count;
    
    if (whitelist.count > CAN_MAX_WHITELIST_SIZE) {
        ESP_LOGW(TAG, "Whitelist count %d exceeds max %d, truncating", 
                 whitelist.count, CAN_MAX_WHITELIST_SIZE);
        whitelist.count = CAN_MAX_WHITELIST_SIZE;
    }
    
    memcpy(whitelist.ids, g_can_whitelist, whitelist.count * sizeof(uint16_t));

    // Write whitelist blob
    err = nvs_set_blob(nvs_handle, NVS_KEY_CAN_WHITELIST, &whitelist,
                       sizeof(can_whitelist_persistent_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing CAN whitelist: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CAN config saved - Baud: %lu, Mode: %d, Whitelist count: %d",
                 g_can_config.baud_rate, g_can_config.operating_mode, g_whitelist_count);
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Load all configurations from NVS (call at startup)
 */
static esp_err_t load_all_configs_from_nvs(void) {
    esp_err_t err;
    
    ESP_LOGI(TAG, "Loading all configurations from NVS...");

    // Load CAN config
    err = load_can_config_from_nvs();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load CAN config");
    }

    // TODO: Add LoRa config loading here when implemented
    // TODO: Add Thread config loading here when implemented
    // TODO: Add Zigbee config loading here when implemented

    ESP_LOGI(TAG, "Configuration loading complete");
    return ESP_OK;
}

/**
 * @brief Erase all gateway configurations from NVS
 */
esp_err_t erase_all_configs_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    ESP_LOGW(TAG, "Erasing all configurations from NVS...");
    
    err = nvs_open_handle(&nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Erase all keys in the namespace
    err = nvs_erase_all(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error erasing NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS erase: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "All configurations erased from NVS");
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * @brief Check if this is first boot
 */
static bool is_first_boot(void) {
    nvs_handle_t handle;
    uint8_t initialized = 0;
    
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        esp_err_t err = nvs_get_u8(handle, NVS_KEY_INITIALIZED, &initialized);
        nvs_close(handle);
        return (err != ESP_OK || initialized == 0);
    }
    
    return true; // Namespace doesn't exist = first boot
}

/**
 * @brief Mark system as initialized
 */
static void mark_initialized(void) {
    nvs_handle_t handle;
    
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, NVS_KEY_INITIALIZED, 1);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

/**
 * @brief Initialize configuration - auto-saves defaults on first boot
 */
esp_err_t config_init(void) {
    ESP_LOGI(TAG, "Initializing LAN MCU configuration system...");

    if (is_first_boot()) {
        ESP_LOGI(TAG, "First boot detected - saving default configuration");
        
        // Save default CAN config to NVS
        save_can_config_to_nvs();
        
        // TODO: Add other default configs here (LoRa, Thread, Zigbee)
        
        mark_initialized();
        ESP_LOGI(TAG, "Default configuration saved");
    } else {
        ESP_LOGI(TAG, "Loading existing configuration");
        load_all_configs_from_nvs();
    }

    return ESP_OK;
}
