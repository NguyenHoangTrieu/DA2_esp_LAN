/**
 * @file zigbee_handler.c
 * @brief Zigbee Handler Middleware Implementation
 *
 * Supports two command modes selected per function via the is_hex flag:
 *
 *  is_hex == false (ASCII/AT mode):
 *    - Sends fc->command [+ \r\n when crlf_terminated == true in loaded config]
 *    - Appends data suffix when is_prefix == true
 *    - Matches ASCII prefix in fc->expect_response
 *
 *  is_hex == true (Binary/HEX mode, E180-ZG120B HEX protocol):
 *    - fc->command stores "55 CMD_TYPE CMD_CODE" as 3 hex-byte template
 *    - Firmware builds full frame: [0x55][LEN][CMD_TYPE][CMD_CODE][DATA][XOR]
 *      where LEN = 3 + payload_len, XOR = CMD_TYPE ^ CMD_CODE ^ DATA...
 *    - fc->expect_response stores response prefix as hex-byte string, e.g. "55 00 04"
 *    - Binary prefix is matched via memmem() in the receive buffer
 *
 * Bus-mutex timeout: 5 000 ms.
 */

#include "zigbee_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "json_zigbee_config_parser.h"
#include "module_config_controller.h"
#include "nvs.h"
#include "stack_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ZIGBEE_HANDLER";

/* ===== Configuration Constants ===== */

#define ZIGBEE_BUS_MUTEX_MS     5000
#define ZIGBEE_LISTEN_MUTEX_MS  50
#define ZIGBEE_LISTEN_WINDOW_MS 100
#define ZIGBEE_CHUNK_SIZE       128

/* ===== Static Data ===== */

typedef struct {
    bool initialized;
    zigbee_module_config_t  config[ZIGBEE_MAX_STACKS];
} g_zigbee_t;

static g_zigbee_t g_zigbee = {0};

static SemaphoreHandle_t g_zigbee_mutex                      = NULL;
static SemaphoreHandle_t g_zigbee_bus_mutex[ZIGBEE_MAX_STACKS] = {NULL, NULL};

/**
 * Function name table for function-name-based command routing.
 * Indices match the zigbee_function_id_t enum values.
 */
static const char *s_zigbee_func_names[ZIGBEE_FUNC_COUNT] = {
    "MODULE_HW_RESET",              // 0
    "MODULE_SW_RESET",              // 1
    "MODULE_FACTORY_RESET",         // 2
    "MODULE_GET_INFO",              // 3
    "MODULE_ENTER_HEX_MODE",        // 4
    "MODULE_START_NETWORK",         // 5
    "MODULE_STOP_NETWORK",          // 6
    "MODULE_GET_NET_STATUS",        // 7
    "MODULE_SET_CHANNEL",           // 8
    "MODULE_SET_PANID",             // 9
    "MODULE_SET_TX_POWER",          // 10
    "MODULE_SET_PERMIT_JOIN",       // 11
    "MODULE_NODE_JOIN_NOTIFY",      // 12
    "MODULE_NODE_LEAVE_NOTIFY",     // 13
    "MODULE_NODE_ANNOUNCE_NOTIFY",  // 14
    "MODULE_QUERY_SHORT_ADDR",      // 15
    "MODULE_QUERY_NODE_PORT_INFO",  // 16
    "MODULE_DELETE_NODE",           // 17
    "MODULE_ZCL_READ_ATTR",         // 18
    "MODULE_ZCL_WRITE_ATTR",        // 19
    "MODULE_ZCL_SEND_CONTROL_CMD",  // 20
    "MODULE_ZCL_RECV_CONTROL_CMD",  // 21
    "MODULE_ZCL_RECV_ATTR_REPORT",  // 22
    "MODULE_ZCL_SET_REPORT_RULE",   // 23
    "MODULE_SEND_UNICAST",          // 24
    "MODULE_SEND_BROADCAST",        // 25
    "MODULE_SET_COMM_CONFIG",       // 26
    "MODULE_ENTER_BOOTLOADER",      // 27
    "MODULE_LEAVE_NETWORK",         // 28
    "MODULE_SET_DEVICE_TYPE",       // 29
    "MODULE_QUERY_IEEE_ADDR",       // 30
    "MODULE_ZCL_BIND",             // 31
    "MODULE_ZCL_UNBIND",           // 32
    "MODULE_SEND_MULTICAST",       // 33
    "MODULE_ENTER_AT_MODE",        // 34
    "MODULE_AUTO_FIND_TARGET",     // 35
    "MODULE_ZCL_DISCOVER_ATTR",    // 36
    "MODULE_ZCL_IDENTIFY",         // 37
    "MODULE_ZCL_GET_BIND_TABLE",   // 38
    "MODULE_ENTER_TRANSPARENT_MODE", // 39
    "MODULE_SET_DEST_ADDR",        // 40
    "MODULE_SET_DEST_EP",          // 41
    "MODULE_SET_LP_LEVEL",         // 42
    "MODULE_ENTER_SLEEP",          // 43
    "MODULE_WAKEUP",               // 44
    "MODULE_EXIT_SEND_MODE",         // 45
    "MODULE_BOOT_NOTIFY",          // 46
    "MODULE_NET_STATUS_NOTIFY",    // 47
    "MODULE_FIND_BIND_NOTIFY",     // 48
    "MODULE_SEND_CONFIRM",         // 49
    "MODULE_ZCL_DEFAULT_RSP",      // 50
};

/* ===== Internal Helpers ===== */

static bool is_valid_stack(uint8_t sid) { return (sid == 0 || sid == 1); }

/**
 * @brief Parse a space-separated hex byte string ("55 00 04") into a byte array.
 * @return Number of bytes written to @p buf.
 */
static size_t hex_str_to_bytes(const char *hex_str, uint8_t *buf, size_t out_max) {
    if (!hex_str || !buf || out_max == 0) return 0;
    size_t n = 0;
    const char *p = hex_str;
    while (*p && n < out_max) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        unsigned int bval = 0;
        int consumed = 0;
        /* sscanf with %2x consumes at most 2 hex digits */
        if (sscanf(p, "%2x%n", &bval, &consumed) != 1 || consumed == 0) break;
        buf[n++] = (uint8_t)bval;
        p += consumed;
    }
    return n;
}

/**
 * @brief Format a binary byte buffer as space-separated uppercase hex string.
 *        e.g. {0x55,0x00,0x00} → "55 00 00"
 */
static int bytes_to_hex_str(const uint8_t *bytes, size_t len,
                             char *out, size_t out_max) {
    int pos = 0;
    for (size_t i = 0; i < len && pos + 3 < (int)out_max; i++) {
        pos += snprintf(out + pos, out_max - pos,
                        "%s%02X", (i == 0 ? "" : " "), bytes[i]);
    }
    return pos;
}

static comm_port_type_t get_port(uint8_t sid) {
    const char *p = g_zigbee.config[sid].comm_port_type;
    if (strcmp(p, "uart") == 0) return COMM_PORT_UART;
    if (strcmp(p, "spi")  == 0) return COMM_PORT_SPI;
    if (strcmp(p, "i2c")  == 0) return COMM_PORT_I2C;
    if (strcmp(p, "usb")  == 0) return COMM_PORT_USB;
    return COMM_PORT_MAX;
}

/**
 * @brief Read from the module bus accumulating until a byte pattern is found
 *        in the buffer, or until timeout expires.  Works for both ASCII and
 *        binary responses (uses memmem for matching).
 *
 * @param expect_bytes  Binary pattern to search for; NULL/0 = accept any data.
 * @param expect_len    Length of pattern in bytes.
 */
static esp_err_t zigbee_read_until(uint8_t sid, comm_port_type_t port_type,
                                    const uint8_t *expect_bytes,
                                    size_t expect_len,
                                    uint32_t timeout_ms,
                                    uint8_t *out_buf, size_t out_max,
                                    size_t *out_len) {
    TickType_t start  = xTaskGetTickCount();
    TickType_t limit  = pdMS_TO_TICKS(timeout_ms);
    uint8_t    chunk[ZIGBEE_CHUNK_SIZE];
    size_t     acc    = 0;
    bool       found  = false;
    size_t     pfx_len = expect_len;

    *out_len = 0;

    while ((xTaskGetTickCount() - start) < limit) {
        TickType_t elapsed  = xTaskGetTickCount() - start;
        TickType_t left     = limit - elapsed;
        uint32_t   win_ms   = (uint32_t)(left * portTICK_PERIOD_MS);
        if (win_ms > 200U) win_ms = 200U;

        size_t    chunk_len = 0;
        esp_err_t r = module_bus_read(sid, port_type,
                                      chunk, sizeof(chunk),
                                      win_ms, &chunk_len);
        if (r != ESP_OK && r != ESP_ERR_TIMEOUT) break;

        if (chunk_len > 0) {
            size_t space = out_max - acc - 1; /* reserve 1 byte for NUL */
            size_t copy  = (chunk_len < space) ? chunk_len : space;
            if (copy > 0) {
                memcpy(out_buf + acc, chunk, copy);
                acc += copy;
                out_buf[acc] = '\0'; /* keep NUL-terminated for strstr */
            }
            if (pfx_len == 0) {
                found = true;
            } else if (acc >= pfx_len) {
                if (memmem(out_buf, acc, expect_bytes, pfx_len) != NULL) {
                    found = true;
                }
            }
            if (found) break;
        }
    }

    *out_len = acc;
    return found ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ===== GPIO helpers (mirrors lora_handler.c) ===== */

static void zigbee_apply_gpio(uint8_t sid, const gpio_control_t *list, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        char pin_str[16];
        snprintf(pin_str, sizeof(pin_str), "%d%s", sid, list[i].pin);
        module_gpio_write(sid, pin_str, list[i].state);
    }
}

/* ===== Public API ===== */

esp_err_t zigbee_handler_init(void) {
    if (g_zigbee.initialized) {
        ESP_LOGW(TAG, "Zigbee handler already initialized");
        return ESP_OK;
    }
    if (!g_zigbee_mutex) {
        g_zigbee_mutex = xSemaphoreCreateMutex();
        if (!g_zigbee_mutex) return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < ZIGBEE_MAX_STACKS; i++) {
        if (!g_zigbee_bus_mutex[i]) {
            g_zigbee_bus_mutex[i] = xSemaphoreCreateMutex();
            if (!g_zigbee_bus_mutex[i]) return ESP_ERR_NO_MEM;
        }
    }
    memset(&g_zigbee, 0, sizeof(g_zigbee));
    g_zigbee.initialized = true;
    ESP_LOGI(TAG, "Zigbee handler initialized");
    return ESP_OK;
}

esp_err_t zigbee_handler_load_config(uint8_t stack_id,
                                      const char *json_config,
                                      uint16_t json_len) {
    if (!g_zigbee.initialized)      return ESP_ERR_INVALID_STATE;
    if (!is_valid_stack(stack_id))  return ESP_ERR_INVALID_ARG;
    if (!json_config || json_len == 0) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Loading Zigbee config for stack %d (%d bytes)", stack_id, json_len);

    if (xSemaphoreTake(g_zigbee_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Heap-allocate to avoid stack overflow */
    json_zigbee_module_config_t *parsed =
        (json_zigbee_module_config_t *)calloc(1, sizeof(json_zigbee_module_config_t));
    if (!parsed) {
        xSemaphoreGive(g_zigbee_mutex);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = json_zigbee_config_parse(json_config, parsed);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse Zigbee JSON: %s", esp_err_to_name(ret));
        free(parsed);
        xSemaphoreGive(g_zigbee_mutex);
        return ret;
    }

    /* Flatten into internal config */
    zigbee_module_config_t *dst = &g_zigbee.config[stack_id];
    memset(dst, 0, sizeof(*dst));
    dst->module_id = stack_id;
    strncpy(dst->module_type, parsed->metadata.module_type,
            sizeof(dst->module_type) - 1);
    strncpy(dst->module_name, parsed->metadata.module_name,
            sizeof(dst->module_name) - 1);
    dst->crlf_terminated = parsed->metadata.crlf_terminated;

    switch (parsed->metadata.communication.port_type) {
    case COMM_PORT_UART:
        strncpy(dst->comm_port_type, "uart", sizeof(dst->comm_port_type) - 1);
        dst->baudrate = parsed->metadata.communication.params.uart.baudrate;
        ret = module_config_controller_init_uart(
            stack_id, &parsed->metadata.communication.params.uart);
        break;
    case COMM_PORT_SPI:
        strncpy(dst->comm_port_type, "spi", sizeof(dst->comm_port_type) - 1);
        ret = module_config_controller_init_spi(
            stack_id, &parsed->metadata.communication.params.spi);
        break;
    case COMM_PORT_I2C:
        strncpy(dst->comm_port_type, "i2c", sizeof(dst->comm_port_type) - 1);
        ret = module_config_controller_init_i2c(
            stack_id, &parsed->metadata.communication.params.i2c);
        break;
    case COMM_PORT_USB:
        strncpy(dst->comm_port_type, "usb", sizeof(dst->comm_port_type) - 1);
        ret = module_config_controller_init_usb(
            stack_id, &parsed->metadata.communication.params.usb);
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Comm init failed for stack %d: %s",
                 stack_id, esp_err_to_name(ret));
        free(parsed); xSemaphoreGive(g_zigbee_mutex);
        return ret;
    }

    for (int i = 0; i < ZIGBEE_FUNC_COUNT; i++) {
        dst->functions[i].available = false;
    }

    for (int i = 0; i < ZIGBEE_MAX_FUNCTIONS; i++) {
        json_zigbee_function_config_t *src = &parsed->functions[i];
        if (!src->available ||
            (zigbee_function_id_t)src->function_id >= ZIGBEE_FUNC_COUNT) {
            continue;
        }
        zigbee_function_config_t *d = &dst->functions[src->function_id];
        d->available      = true;
        d->is_hex         = src->is_hex;
        d->is_prefix      = src->is_prefix;
        d->timeout_ms     = src->timeout_ms;
        d->delay_start_ms = src->delay_start_ms;
        d->delay_end_ms   = src->delay_end_ms;
        strncpy(d->command,          src->command,          ZIGBEE_COMMAND_LEN - 1);
        strncpy(d->expect_response,  src->expect_response,  ZIGBEE_RESPONSE_LEN - 1);
        memcpy(d->gpio_start, src->gpio_start, sizeof(src->gpio_start));
        d->gpio_start_count = src->gpio_start_count;
        memcpy(d->gpio_end, src->gpio_end, sizeof(src->gpio_end));
        d->gpio_end_count = src->gpio_end_count;
    }

    /* Default mode: all stacks start in AT mode */

    free(parsed);
    xSemaphoreGive(g_zigbee_mutex);
    ESP_LOGI(TAG, "Zigbee stack %d config loaded OK (module: %s)", stack_id, dst->module_name);
    return ESP_OK;
}

esp_err_t zigbee_handler_execute_function(
    uint8_t stack_id,
    zigbee_function_id_t func_id,
    const uint8_t *data,
    uint8_t data_len,
    zigbee_exec_result_t *result)
{
    if (!g_zigbee.initialized) return ESP_ERR_INVALID_STATE;
    if (!is_valid_stack(stack_id) || func_id >= ZIGBEE_FUNC_COUNT) {
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }

    zigbee_function_config_t *fc = &g_zigbee.config[stack_id].functions[func_id];
    if (!fc->available) {
        ESP_LOGW(TAG, "Function %d not available on stack %d", func_id, stack_id);
        if (result) result->status = ESP_ERR_NOT_SUPPORTED;
        return ESP_ERR_NOT_SUPPORTED;
    }



    TickType_t start_tick = xTaskGetTickCount();
    esp_err_t  ret        = ESP_OK;

    /* Apply GPIO start */
    zigbee_apply_gpio(stack_id, fc->gpio_start, fc->gpio_start_count);
    if (fc->delay_start_ms > 0) vTaskDelay(pdMS_TO_TICKS(fc->delay_start_ms));

    comm_port_type_t port_type = get_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        if (result) result->status = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    /* Acquire bus mutex */
    if (xSemaphoreTake(g_zigbee_bus_mutex[stack_id],
                       pdMS_TO_TICKS(ZIGBEE_BUS_MUTEX_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "[Stack %d] Bus mutex timeout", stack_id);
        if (result) result->status = ESP_ERR_TIMEOUT;
        return ESP_ERR_TIMEOUT;
    }

    if (!fc->is_hex) {
        /* ===== ASCII / AT command ===== */
        size_t cmd_len = strlen(fc->command);
        /* Build: command [+data_suffix] [\r\n if crlf_terminated] */
        char at_buf[ZIGBEE_COMMAND_LEN + 256];
        int  written = 0;
        bool add_crlf = g_zigbee.config[stack_id].crlf_terminated;
        if (cmd_len > 0) {
            if (fc->is_prefix && data && data_len > 0) {
                written = snprintf(at_buf, sizeof(at_buf),
                                   add_crlf ? "%s%.*s\r\n" : "%s%.*s",
                                   fc->command, (int)data_len, (const char *)data);
            } else {
                written = snprintf(at_buf, sizeof(at_buf),
                                   add_crlf ? "%s\r\n" : "%s",
                                   fc->command);
            }
            /* Flush RX ring buffer before sending so stale unsolicited bytes
             * from the module do not contaminate this command's response. */
            module_bus_flush(stack_id, port_type);
            ESP_LOGI(TAG, "ZB conf TX: %.*s (%d bytes)", written, at_buf, written);
            ret = module_bus_write(stack_id, port_type,
                                   (const uint8_t *)at_buf, (size_t)written);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "AT write failed: %s", esp_err_to_name(ret));
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                if (result) result->status = ret;
                return ret;
            }
        }
    } else {
        /* ===== Generic Binary / HEX command =====
         * command field contains space-separated hex bytes (e.g. "55 04 01 05").
         * Decoded bytes are sent as-is.  If is_prefix=true, runtime data bytes
         * are appended directly after the decoded command bytes.
         */
        uint8_t hex_cmd[ZIGBEE_COMMAND_LEN];
        size_t  hex_cmd_len = hex_str_to_bytes(fc->command, hex_cmd, sizeof(hex_cmd));
        if (hex_cmd_len > 0) {
            uint8_t frame[ZIGBEE_COMMAND_LEN + 256];
            memcpy(frame, hex_cmd, hex_cmd_len);
            size_t frame_len = hex_cmd_len;
            if (fc->is_prefix && data && data_len > 0) {
                size_t append = data_len < (sizeof(frame) - frame_len) ? data_len : (sizeof(frame) - frame_len);
                memcpy(frame + frame_len, data, append);
                frame_len += append;
            }
            ret = module_bus_write(stack_id, port_type, frame, frame_len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "HEX write failed: %s", esp_err_to_name(ret));
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                if (result) result->status = ret;
                return ret;
            }
        }
    }

    /* === Read response === */
    size_t resp_len = 0;
    bool skip_read  = (fc->timeout_ms == 0);

    if (!skip_read && result) {
        if (!fc->is_hex) {
            /* ASCII: match by raw bytes of expect_response string */
            uint8_t  resp_pattern[16];
            size_t   resp_pattern_len = 0;
            size_t   ascii_len = strlen(fc->expect_response);
            if (ascii_len > 0) {
                if (ascii_len > sizeof(resp_pattern)) ascii_len = sizeof(resp_pattern);
                memcpy(resp_pattern, fc->expect_response, ascii_len);
                resp_pattern_len = ascii_len;
            }
            ret = zigbee_read_until(
                stack_id, port_type,
                resp_pattern_len > 0 ? resp_pattern : NULL,
                resp_pattern_len,
                fc->timeout_ms,
                result->response, ZIGBEE_RESP_BUF_SIZE - 1,
                &resp_len);
            result->response[resp_len] = '\0';
            if (resp_len > 0) {
                ESP_LOGI(TAG, "ZB RX (%zu bytes): %.*s", resp_len, (int)resp_len, (char *)result->response);
            } else {
                ESP_LOGI(TAG, "ZB RX: (no data)");
            }
            if (ret != ESP_OK && resp_pattern_len > 0) {
                ESP_LOGW(TAG, "ZB response validation failed for func %d (expected: \"%s\")",
                         func_id, fc->expect_response);
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                result->status       = ESP_ERR_INVALID_RESPONSE;
                result->response_len = (uint16_t)resp_len;
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else {
            /* HEX: collect data until binary-decoded expected pattern is found or
             * timeout, then validate by formatting received bytes as "XX XX XX"
             * hex string and using strstr() against expect_response. */
            uint8_t  resp_pattern[16];
            size_t   resp_pattern_len = hex_str_to_bytes(fc->expect_response,
                                                          resp_pattern, sizeof(resp_pattern));
            ret = zigbee_read_until(
                stack_id, port_type,
                resp_pattern_len > 0 ? resp_pattern : NULL,
                resp_pattern_len,
                fc->timeout_ms,
                result->response, ZIGBEE_RESP_BUF_SIZE - 1,
                &resp_len);
            /* Format received binary bytes as space-separated hex string */
            char hex_resp[ZIGBEE_RESP_BUF_SIZE * 3];
            bytes_to_hex_str(result->response, resp_len, hex_resp, sizeof(hex_resp));
            if (resp_len > 0) {
                ESP_LOGI(TAG, "ZB RX %zu bytes (HEX): %s", resp_len, hex_resp);
            } else {
                ESP_LOGI(TAG, "ZB RX: (no data)");
            }
            /* Validate: hex-formatted response must contain expect_response as substring */
            if (strlen(fc->expect_response) > 0 &&
                strstr(hex_resp, fc->expect_response) == NULL) {
                ESP_LOGW(TAG, "ZB response validation failed (expected: \"%s\")",
                         fc->expect_response);
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                result->status       = ESP_ERR_INVALID_RESPONSE;
                result->response_len = (uint16_t)resp_len;
                return ESP_ERR_INVALID_RESPONSE;
            }
            ret = ESP_OK;
        }
    }

    xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);

    /* GPIO end */
    zigbee_apply_gpio(stack_id, fc->gpio_end, fc->gpio_end_count);
    if (fc->delay_end_ms > 0) vTaskDelay(pdMS_TO_TICKS(fc->delay_end_ms));

    uint32_t exec_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    if (result) {
        result->status            = ESP_OK;
        result->response_len      = (uint16_t)resp_len;
        result->execution_time_ms = exec_ms;
    }

    ESP_LOGI(TAG, "Func %d ok on stack %d (%lu ms)", func_id, stack_id, exec_ms);
    return ESP_OK;
}

esp_err_t zigbee_handler_execute_command_with_config(
    uint8_t stack_id,
    const char *command,
    const zigbee_function_config_t *fc,
    zigbee_exec_result_t *result)
{
    if (!g_zigbee.initialized || !is_valid_stack(stack_id) || !command || !fc)
        return ESP_ERR_INVALID_ARG;

    size_t command_len = strlen(command);
    comm_port_type_t port_type = get_port(stack_id);
    if (port_type == COMM_PORT_MAX) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(g_zigbee_bus_mutex[stack_id],
                       pdMS_TO_TICKS(ZIGBEE_BUS_MUTEX_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    TickType_t start_tick = xTaskGetTickCount();
    ESP_LOGI(TAG, "Executing command '%.*s' on stack %d (is_hex=%d)",
             (int)command_len, command, stack_id, fc->is_hex);

    /* GPIO start */
    zigbee_apply_gpio(stack_id, fc->gpio_start, fc->gpio_start_count);
    if (fc->delay_start_ms > 0) vTaskDelay(pdMS_TO_TICKS(fc->delay_start_ms));

    esp_err_t ret = ESP_OK;
    if (!fc->is_hex) {
        /* ASCII path: send command string, append \r\n only when crlf_terminated */
        bool add_crlf = g_zigbee.config[stack_id].crlf_terminated;
        size_t raw_len = command_len;
        while (raw_len > 0 &&
               (command[raw_len - 1] == '\r' || command[raw_len - 1] == '\n')) {
            raw_len--;
        }
        size_t buf_size = raw_len + (add_crlf ? 3 : 1);
        char *buf = malloc(buf_size);
        if (!buf) {
            xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
            return ESP_ERR_NO_MEM;
        }
        memcpy(buf, command, raw_len);
        if (add_crlf) {
            buf[raw_len]     = '\r';
            buf[raw_len + 1] = '\n';
            buf[raw_len + 2] = '\0';
        } else {
            buf[raw_len] = '\0';
        }
        size_t tx_len = raw_len + (add_crlf ? 2 : 0);
        module_bus_flush(stack_id, port_type);
        ESP_LOGI(TAG, "ZB TX: %.*s (%zu bytes)", (int)tx_len, buf, tx_len);
        ret = module_bus_write(stack_id, port_type, (const uint8_t *)buf, tx_len);
        free(buf);
    } else {
        /* HEX path: decode hex-string command, send raw bytes */
        uint8_t hex_bytes[252];
        size_t  hex_len = hex_str_to_bytes(command, hex_bytes, sizeof(hex_bytes));
        if (hex_len == 0) {
            xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
            return ESP_ERR_INVALID_ARG;
        }
        ESP_LOGI(TAG, "ZB TX: HEX (%zu bytes)", hex_len);
        ret = module_bus_write(stack_id, port_type, hex_bytes, hex_len);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ZB TX failed: %s", esp_err_to_name(ret));
        xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
        return ret;
    }

    /* Read response */
    size_t resp_len  = 0;
    bool   skip_read = (fc->timeout_ms == 0);

    if (!skip_read && result) {
        if (!fc->is_hex) {
            /* ASCII: match by raw bytes of expect_response string */
            uint8_t resp_pattern[16];
            size_t  resp_pattern_len = 0;
            size_t  ascii_len = strlen(fc->expect_response);
            if (ascii_len > 0) {
                if (ascii_len > sizeof(resp_pattern)) ascii_len = sizeof(resp_pattern);
                memcpy(resp_pattern, fc->expect_response, ascii_len);
                resp_pattern_len = ascii_len;
            }
            ret = zigbee_read_until(
                stack_id, port_type,
                resp_pattern_len > 0 ? resp_pattern : NULL,
                resp_pattern_len,
                fc->timeout_ms,
                result->response, ZIGBEE_RESP_BUF_SIZE - 1,
                &resp_len);
            result->response[resp_len] = '\0';
            if (resp_len > 0)
                ESP_LOGI(TAG, "ZB RX (%zu bytes): %.*s", resp_len, (int)resp_len, (char *)result->response);
            if (ret != ESP_OK && resp_pattern_len > 0) {
                ESP_LOGW(TAG, "ZB response validation failed (expected: \"%s\")", fc->expect_response);
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                result->status       = ESP_ERR_INVALID_RESPONSE;
                result->response_len = (uint16_t)resp_len;
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else {
            /* HEX: collect data until binary-decoded expected pattern is found or
             * timeout, then validate by formatting received bytes as "XX XX XX"
             * hex string and using strstr() against expect_response. */
            uint8_t resp_pattern[16];
            size_t  resp_pattern_len = hex_str_to_bytes(fc->expect_response,
                                                         resp_pattern, sizeof(resp_pattern));
            ret = zigbee_read_until(
                stack_id, port_type,
                resp_pattern_len > 0 ? resp_pattern : NULL,
                resp_pattern_len,
                fc->timeout_ms,
                result->response, ZIGBEE_RESP_BUF_SIZE - 1,
                &resp_len);
            /* Format received binary bytes as space-separated hex string */
            char hex_resp[ZIGBEE_RESP_BUF_SIZE * 3];
            bytes_to_hex_str(result->response, resp_len, hex_resp, sizeof(hex_resp));
            if (resp_len > 0)
                ESP_LOGI(TAG, "ZB RX %zu bytes (HEX): %s", resp_len, hex_resp);
            /* Validate: hex-formatted response must contain expect_response as substring */
            if (strlen(fc->expect_response) > 0 &&
                strstr(hex_resp, fc->expect_response) == NULL) {
                ESP_LOGW(TAG, "ZB response validation failed (expected: \"%s\")", fc->expect_response);
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                result->status       = ESP_ERR_INVALID_RESPONSE;
                result->response_len = (uint16_t)resp_len;
                return ESP_ERR_INVALID_RESPONSE;
            }
            ret = ESP_OK;
        }
    } else if (!skip_read && !result) {
        uint8_t dummy[64];
        size_t  dummy_len = 0;
        zigbee_read_until(stack_id, port_type, NULL, 0,
                          fc->timeout_ms, dummy, sizeof(dummy), &dummy_len);
    }

    xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);

    /* GPIO end */
    zigbee_apply_gpio(stack_id, fc->gpio_end, fc->gpio_end_count);
    if (fc->delay_end_ms > 0) vTaskDelay(pdMS_TO_TICKS(fc->delay_end_ms));

    uint32_t exec_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    if (result) {
        result->status            = ESP_OK;
        result->response_len      = (uint16_t)resp_len;
        result->execution_time_ms = exec_ms;
    }
    ESP_LOGI(TAG, "Command executed on stack %d (took %lu ms)", stack_id, exec_ms);
    return ESP_OK;
}

esp_err_t zigbee_handler_listen(uint8_t stack_id,
                                 uint8_t *buf,
                                 size_t   max,
                                 size_t  *out_len) {
    if (!g_zigbee.initialized || !is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;
    if (!buf || !out_len) return ESP_ERR_INVALID_ARG;

    *out_len = 0;
    if (xSemaphoreTake(g_zigbee_bus_mutex[stack_id],
                       pdMS_TO_TICKS(ZIGBEE_LISTEN_MUTEX_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    comm_port_type_t port_type = get_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
        return ESP_ERR_INVALID_STATE;
    }

    size_t chunk_len = 0;
    esp_err_t ret = module_bus_read(stack_id, port_type,
                                    buf, max,
                                    ZIGBEE_LISTEN_WINDOW_MS, &chunk_len);

    xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);

    if (chunk_len > 0) {
        *out_len = chunk_len;
        return ESP_OK;
    }
    return (ret == ESP_ERR_TIMEOUT) ? ESP_ERR_TIMEOUT : ret;
}

esp_err_t zigbee_handler_get_function_config(uint8_t stack_id,
                                              zigbee_function_id_t func_id,
                                              zigbee_function_config_t *out) {
    if (!g_zigbee.initialized || !is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;
    if (func_id >= ZIGBEE_FUNC_COUNT || !out) return ESP_ERR_INVALID_ARG;
    *out = g_zigbee.config[stack_id].functions[func_id];
    return ESP_OK;
}

esp_err_t zigbee_handler_get_function_by_command(uint8_t stack_id,
                                                   const char *command,
                                                   zigbee_function_config_t *func_config) {
    if (!g_zigbee.initialized || !command || !func_config) return ESP_ERR_INVALID_ARG;
    if (!is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;

    size_t cmd_len = strlen(command);

    /* Pass 1: match by cfg->command (AT+ prefix or exact) */
    for (int fid = 0; fid < ZIGBEE_FUNC_COUNT; fid++) {
        zigbee_function_config_t *cfg = &g_zigbee.config[stack_id].functions[fid];
        if (!cfg->available) continue;

        size_t cfg_len = strlen(cfg->command);
        /* Strip trailing \r\n */
        while (cfg_len > 0 &&
               (cfg->command[cfg_len - 1] == '\n' ||
                cfg->command[cfg_len - 1] == '\r')) {
            cfg_len--;
        }
        if (cfg_len == 0) continue;

        if (strncmp(cfg->command, "AT+", 3) == 0) {
            /* AT commands: prefix match to allow params after the base command */
            if (cmd_len >= cfg_len &&
                strncmp(command, cfg->command, cfg_len) == 0) {
                memcpy(func_config, cfg, sizeof(zigbee_function_config_t));
                ESP_LOGI(TAG, "ZB match prefix: '%.*s' (fid=%d)", (int)cfg_len, cfg->command, fid);
                return ESP_OK;
            }
        } else {
            /* Non-AT: exact match */
            if (cmd_len == cfg_len &&
                strncmp(command, cfg->command, cfg_len) == 0) {
                memcpy(func_config, cfg, sizeof(zigbee_function_config_t));
                ESP_LOGI(TAG, "ZB match exact: '%.*s' (fid=%d)", (int)cfg_len, cfg->command, fid);
                return ESP_OK;
            }
        }
    }

    ESP_LOGW(TAG, "ZB no config match for '%s' — using defaults", command);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zigbee_handler_get_function_by_name(uint8_t stack_id,
                                               const char *func_name,
                                               zigbee_function_config_t *func_config) {
    if (!g_zigbee.initialized || !func_name || !func_config) return ESP_ERR_INVALID_ARG;
    if (!is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;

    for (int fid = 0; fid < ZIGBEE_FUNC_COUNT; fid++) {
        if (s_zigbee_func_names[fid] == NULL ||
            s_zigbee_func_names[fid][0] == '\0') {
            continue;
        }
        if (strcmp(func_name, s_zigbee_func_names[fid]) == 0) {
            zigbee_function_config_t *cfg = &g_zigbee.config[stack_id].functions[fid];
            if (!cfg->available) {
                ESP_LOGW(TAG, "ZB function '%s' (id=%d) not configured for stack %d",
                         func_name, fid, stack_id);
                return ESP_ERR_NOT_SUPPORTED;
            }
            memcpy(func_config, cfg, sizeof(zigbee_function_config_t));
            ESP_LOGI(TAG, "ZB matched function_name: %s (fid=%d)", func_name, fid);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "ZB no function match for name: %s", func_name);
    return ESP_ERR_NOT_FOUND;
}


/* ===========================================================
 * E180-ZG120B one-time AT mode ensure  —  FULL VERBOSE VERSION
 *
 * Fully hardcoded. Every TX/RX is logged to help diagnose issues.
 *
 * Hardware (fixed):
 *   Bus    : configured via module_config_controller (STACK_UART_PORT), 115200 8N1
 *   RESET# : TCA6416A pin STACK_GPIO_PIN_05 (P05) via stack_handler
 *
 * E180-ZG120B modes (from Ebyte datasheet):
 *   HEX mode    (default) : binary frame protocol, SFD=0x55
 *                           boot msg → "55 0D 80 00 ..." binary frame
 *   AT mode               : ASCII "AT+xxx\r\n" commands
 *                           boot msg → "BOOT=0\r\nVERSION=xx\r\n"
 *   Transparent mode      : raw serial passthrough
 *                           exit with "+++" → returns to HEX mode
 *
 * Switching to AT mode:
 *   HEX  → AT  : send [55 03 00 16 16]  ACK=[55 04 00 16 00 16]
 *   Transp → AT : send "+++" (→ HEX) then [55 03 00 16 16]
 *   AT   → done: verify with "AT+INFO?\r\n", expect "TYPE=" / "NO NET"
 *
 * Flow:
 *   1. NVS flag set?  → quick AT+INFO? verify
 *   2. HW Reset → collect boot msg → detect HEX vs AT boot
 *   3. If HEX boot → send [55 03 00 16 16] → verify
 *   4. Try "+++" + [55 03 00 16 16]  (transparent fallback)
 *   5. Last resort: reset + immediate [55 03 00 16 16]
 *
 * NVS namespace : "zb_init"   key: "at_ok_s{stack_id}"
 * =========================================================== */

#define ZB_AT_NVS_NS   "zb_init"
#define ZB_RESET_PIN   STACK_GPIO_PIN_05   /* TCA P05 = E180 nRESET# */

/* HEX command: Enter AT mode  TYPE=0x00 CODE=0x16 XOR=0x16 */
static const uint8_t k_hex_to_at[]     = { 0x55, 0x03, 0x00, 0x16, 0x16 };
/* Expected ACK from module */
static const uint8_t k_hex_to_at_ack[] = { 0x55, 0x04, 0x00, 0x16, 0x00, 0x16 };

/* ---- logging helper ---- */
/** Print buffer as hex rows + ASCII sidebar */
static void zb_log_buf(const char *label, const uint8_t *buf, size_t len) {
    if (len == 0) {
        ESP_LOGI(TAG, "    [%s] (no data)", label);
        return;
    }
    for (size_t i = 0; i < len; i += 16) {
        size_t row = (len - i < 16) ? (len - i) : 16;
        char hex[64] = {0};
        char asc[24] = {0};
        int  hp = 0;
        for (size_t j = 0; j < row; j++) {
            hp += snprintf(hex + hp, sizeof(hex) - hp, "%02X ", buf[i + j]);
            asc[j] = (buf[i+j] >= 0x20 && buf[i+j] < 0x7F) ? (char)buf[i+j] : '.';
        }
        asc[row] = '\0';
        ESP_LOGI(TAG, "    [%s +%02X] %-48s |%s|", label, (unsigned)i, hex, asc);
    }
}

/* ---- Comm helpers (all go through module_config_controller) ---- */
/** Drain bytes arriving within window_ms into buf; returns byte count */
static size_t zb_bus_drain(uint8_t sid, uint8_t *buf, size_t max, uint32_t ms) {
    comm_port_type_t pt = get_port(sid);
    return module_bus_drain(sid, pt, buf, max, ms);
}

/** Flush RX buffer, then send data with logging */
static void zb_tx(uint8_t sid, const char *label, const uint8_t *data, size_t len) {
    comm_port_type_t pt = get_port(sid);
    module_bus_flush(sid, pt);
    ESP_LOGI(TAG, "  >> TX [%s]  %zu bytes", label, len);
    zb_log_buf(label, data, len);
    module_bus_write(sid, pt, data, len);
}

/** HW reset via TCA P05, wait for module boot (800 ms) */
static void zb_hw_reset(uint8_t sid) {
    ESP_LOGI(TAG, "  [HW RESET] P05 LOW → 120ms → HIGH, waiting 800ms...");
    stack_handler_gpio_write(sid, ZB_RESET_PIN, false);
    vTaskDelay(pdMS_TO_TICKS(120));
    stack_handler_gpio_write(sid, ZB_RESET_PIN, true);
    vTaskDelay(pdMS_TO_TICKS(800));
}

/** Check if buf contains AT-mode boot markers */
static bool zb_is_at_boot(const uint8_t *buf, size_t len) {
    return memmem(buf, len, "BOOT=",   5) != NULL ||
           memmem(buf, len, "VERSION=",8) != NULL;
}

/** Check if buf contains HEX-mode boot frame (SFD=0x55 + reasonable LEN) */
static bool zb_is_hex_boot(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i + 1 < len; i++) {
        if (buf[i] == 0x55 && buf[i+1] >= 3 && buf[i+1] <= 15) return true;
    }
    return false;
}

/** Check if buf contains valid AT+INFO? response */
static bool zb_is_at_info_rsp(const uint8_t *buf, size_t len) {
    return memmem(buf, len, "TYPE=",  5) != NULL ||
           memmem(buf, len, "NO NET", 6) != NULL ||
           memmem(buf, len, "MAC=",   4) != NULL;
}

/* ---- Test function: Send AT+RESET and log raw response ---- */
esp_err_t zigbee_handler_test_at_reset(uint8_t stack_id) {
    if (!is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "\n========== TEST: AT+RESET on Stack %d ==========", stack_id);

    /* Acquire bus mutex */
    if (xSemaphoreTake(g_zigbee_bus_mutex[stack_id], pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "[Stack %d] Cannot acquire bus mutex!", stack_id);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t rx[512] = {0};
    size_t got = 0;
    esp_err_t ret = ESP_OK;

    /* Get port and send AT+RESET */
    comm_port_type_t port_type = get_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        ESP_LOGE(TAG, "[Stack %d] Invalid port type", stack_id);
        xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
        return ESP_ERR_INVALID_STATE;
    }

    /* Flush RX buffer first */
    module_bus_flush(stack_id, port_type);
    ESP_LOGI(TAG, "[Stack %d] Flushed RX buffer", stack_id);

    /* Send AT+RESET */
    const uint8_t *cmd = (const uint8_t *)"AT+RESET";
    size_t cmd_len = 8; /* strlen("AT+RESET") */
    ESP_LOGI(TAG, "[Stack %d] >> Sending command:", stack_id);
    zb_log_buf("AT+RESET", cmd, cmd_len);
    
    ret = module_bus_write(stack_id, port_type, cmd, cmd_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Stack %d] Write failed: %s", stack_id, esp_err_to_name(ret));
        xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
        return ret;
    }

    /* Wait and collect response (2000ms timeout, same as config) */
    ESP_LOGI(TAG, "[Stack %d] Waiting 2000ms for response...", stack_id);
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 2000);

    ESP_LOGI(TAG, "[Stack %d] << Received %zu bytes:", stack_id, got);
    if (got > 0) {
        zb_log_buf("AT+RESET_RESPONSE", rx, got);
        ESP_LOGI(TAG, "[Stack %d] As string: %.*s", stack_id, (int)got, (char *)rx);
    } else {
        ESP_LOGW(TAG, "[Stack %d] NO RESPONSE received!", stack_id);
    }

    xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);

    ESP_LOGI(TAG, "========== TEST COMPLETE ==========\n", stack_id);
    return ESP_OK;
}

/* ---- main function ---- */
#if 1
esp_err_t zigbee_handler_ensure_at_mode(uint8_t stack_id) {
    if (!is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "[Stack %d] ====== E180-ZG120B AT Mode Ensure (verbose) ======", stack_id);

    /* Check NVS */
    bool nvs_ok = false;
    char nvs_key[16];
    snprintf(nvs_key, sizeof(nvs_key), "at_ok_s%d", stack_id);
    {
        nvs_handle_t h;
        if (nvs_open(ZB_AT_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
            uint8_t v = 0;
            nvs_ok = (nvs_get_u8(h, nvs_key, &v) == ESP_OK && v == 1);
            nvs_close(h);
        }
    }
    ESP_LOGI(TAG, "[Stack %d]   NVS flag: %s", stack_id,
             nvs_ok ? "SET (AT mode previously confirmed)" : "NOT SET");

    /* Acquire bus mutex (blocks listener task) */
    if (xSemaphoreTake(g_zigbee_bus_mutex[stack_id], pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "[Stack %d]   Cannot acquire bus mutex!", stack_id);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t rx[512] = {0};
    size_t  got;
    bool    confirmed = false;

    /* ── Phase 1: NVS flag set → quick verify ──────────────────────── */
    if (nvs_ok) {
        ESP_LOGI(TAG, "[Stack %d] -- Phase 1: quick verify (NVS set) --", stack_id);
        zb_tx(stack_id, "AT+INFO?", (const uint8_t *)"AT+INFO?\r\n", 10);
        got = zb_bus_drain(stack_id, rx, sizeof(rx), 1500);
        ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
        zb_log_buf("INFO?-rsp", rx, got);
        if (zb_is_at_info_rsp(rx, got)) {
            ESP_LOGI(TAG, "[Stack %d]   AT mode confirmed (NVS quick verify)", stack_id);
            confirmed = true;
            goto zb_release;
        }
        ESP_LOGW(TAG, "[Stack %d]   Quick verify failed → full ensure", stack_id);
    }

    /* ── Phase 2: HW Reset + detect boot mode ───────────────────────── */
    ESP_LOGI(TAG, "[Stack %d] -- Phase 2: HW Reset + boot mode detection --", stack_id);
    module_bus_flush(stack_id, get_port(stack_id));
    zb_hw_reset(stack_id);
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 300);
    ESP_LOGI(TAG, "[Stack %d]   << Boot messages %zu bytes:", stack_id, got);
    zb_log_buf("BOOT", rx, got);

    bool at_boot  = zb_is_at_boot(rx, got);
    bool hex_boot = zb_is_hex_boot(rx, got);
    ESP_LOGI(TAG, "[Stack %d]   Mode from boot: AT=%d  HEX=%d  SILENT=%d",
             stack_id, at_boot, hex_boot, (got == 0));

    if (at_boot) {
        /* Module already stored AT mode → verify */
        ESP_LOGI(TAG, "[Stack %d]   Booted in AT mode → verify with AT+INFO?", stack_id);
        zb_tx(stack_id, "AT+INFO?", (const uint8_t *)"AT+INFO?\r\n", 10);
        got = zb_bus_drain(stack_id, rx, sizeof(rx), 1500);
        ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
        zb_log_buf("INFO?-rsp", rx, got);
        if (zb_is_at_info_rsp(rx, got)) { confirmed = true; goto zb_release; }
        ESP_LOGW(TAG, "[Stack %d]   AT boot but INFO? failed — continuing", stack_id);
    }

    /* ── Phase 3: HEX→AT command [55 03 00 16 16] ──────────────────── */
    ESP_LOGI(TAG, "[Stack %d] -- Phase 3: HEX→AT command [55 03 00 16 16] --", stack_id);
    zb_tx(stack_id, "HEX->AT", k_hex_to_at, sizeof(k_hex_to_at));
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 800);
    ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
    zb_log_buf("HEX->AT-rsp", rx, got);
    if (memmem(rx, got, k_hex_to_at_ack, sizeof(k_hex_to_at_ack)))
        ESP_LOGI(TAG, "[Stack %d]   ACK [55 04 00 16 00 16] found!", stack_id);
    else
        ESP_LOGW(TAG, "[Stack %d]   No ACK pattern found in response", stack_id);

    zb_tx(stack_id, "AT+INFO?", (const uint8_t *)"AT+INFO?\r\n", 10);
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 1500);
    ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
    zb_log_buf("INFO?-rsp", rx, got);
    if (zb_is_at_info_rsp(rx, got)) { confirmed = true; goto zb_release; }

    /* ── Phase 4: Transparent exit (+++) then HEX→AT ───────────────── */
    ESP_LOGI(TAG, "[Stack %d] -- Phase 4: +++ (exit transparent) then HEX->AT --", stack_id);
    zb_tx(stack_id, "+++", (const uint8_t *)"+++", 3);
    vTaskDelay(pdMS_TO_TICKS(1000));
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 200);
    ESP_LOGI(TAG, "[Stack %d]   << +++ response %zu bytes:", stack_id, got);
    zb_log_buf("+++-rsp", rx, got);

    zb_tx(stack_id, "HEX->AT#2", k_hex_to_at, sizeof(k_hex_to_at));
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 800);
    ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
    zb_log_buf("ACK#2", rx, got);

    zb_tx(stack_id, "AT+INFO?", (const uint8_t *)"AT+INFO?\r\n", 10);
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 1500);
    ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
    zb_log_buf("INFO?-rsp2", rx, got);
    if (zb_is_at_info_rsp(rx, got)) { confirmed = true; goto zb_release; }

    /* ── Phase 5: Last resort — reset + immediate HEX→AT ───────────── */
    ESP_LOGI(TAG, "[Stack %d] -- Phase 5: reset + immediate HEX->AT --", stack_id);
    module_bus_flush(stack_id, get_port(stack_id));
    zb_hw_reset(stack_id);
    zb_tx(stack_id, "HEX->AT#3", k_hex_to_at, sizeof(k_hex_to_at));
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 800);
    ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes:", stack_id, got);
    zb_log_buf("ACK#3", rx, got);

    zb_tx(stack_id, "AT+INFO?", (const uint8_t *)"AT+INFO?\r\n", 10);
    got = zb_bus_drain(stack_id, rx, sizeof(rx), 1500);
    ESP_LOGI(TAG, "[Stack %d]   << RX %zu bytes (final):", stack_id, got);
    zb_log_buf("INFO?-final", rx, got);
    if (zb_is_at_info_rsp(rx, got)) { confirmed = true; }

zb_release:
    xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);

    if (confirmed) {
        ESP_LOGI(TAG, "[Stack %d] ✓ AT mode confirmed — saving NVS flag", stack_id);
        nvs_handle_t h;
        if (nvs_open(ZB_AT_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u8(h, nvs_key, 1);
            nvs_commit(h);
            nvs_close(h);
        }
    } else {
        ESP_LOGE(TAG, "[Stack %d] ✗ FAILED — module did not respond to any command!", stack_id);
        ESP_LOGE(TAG, "[Stack %d]   Possible causes:", stack_id);
        ESP_LOGE(TAG, "[Stack %d]     - UART TX/RX pins swapped?  (TX=15 → E180 RX, RX=16 ← E180 TX)", stack_id);
        ESP_LOGE(TAG, "[Stack %d]     - RESET# pin P05 not driving (check TCA power/I2C)", stack_id);
        ESP_LOGE(TAG, "[Stack %d]     - E180 module power not up (+3V3_CTRL low?)", stack_id);
    }
    ESP_LOGI(TAG, "[Stack %d] ====== AT Mode Ensure Done ======", stack_id);
    return confirmed ? ESP_OK : ESP_ERR_NOT_FOUND;
}
#endif
