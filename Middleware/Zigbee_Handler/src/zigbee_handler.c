/**
 * @file zigbee_handler.c
 * @brief Zigbee Handler Middleware Implementation
 *
 * Supports two command modes selected per function via the is_hex flag:
 *
 *  is_hex == false (ASCII/AT mode):
 *    - Sends fc->command + CRLF (appends data suffix when is_prefix == true)
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool g_zigbee_module_ctrl_initialized = false;

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

static void zigbee_apply_gpio(uint8_t sid, gpio_control_t *list, uint8_t count) {
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

    if (!g_zigbee_module_ctrl_initialized) {
        esp_err_t r2 = module_config_controller_init();
        if (r2 != ESP_OK) {
            ESP_LOGE(TAG, "module_config_controller_init failed");
            free(parsed); xSemaphoreGive(g_zigbee_mutex);
            return r2;
        }
        g_zigbee_module_ctrl_initialized = true;
    }

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
        d->is_async_event = src->is_async_event;
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

esp_err_t zigbee_handler_execute_command_with_config(
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

    if (fc->is_async_event) {
        ESP_LOGW(TAG, "Function %d is async-event only, cannot be directly executed", func_id);
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
        /* Build: command [+data_suffix] \r\n */
        char at_buf[ZIGBEE_COMMAND_LEN + 256];
        int  written = 0;
        if (cmd_len > 0) {
            if (fc->is_prefix && data && data_len > 0) {
                written = snprintf(at_buf, sizeof(at_buf), "%s%.*s\r\n",
                                   fc->command, (int)data_len, (const char *)data);
            } else {
                written = snprintf(at_buf, sizeof(at_buf), "%s\r\n", fc->command);
            }
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
        /* ===== Binary / HEX command =====
         * command field = "55 CMD_TYPE CMD_CODE"  (3 hex bytes, space-separated)
         * Frame built:  [0x55][LEN][CMD_TYPE][CMD_CODE][DATA...][XOR]
         * LEN = 3 + payload_len  (CMD_TYPE + CMD_CODE + DATA + XOR)
         * XOR = CMD_TYPE ^ CMD_CODE ^ DATA...
         */
        uint8_t cmd_bytes[4];
        size_t  cmd_byte_len = hex_str_to_bytes(fc->command, cmd_bytes, sizeof(cmd_bytes));
        if (cmd_byte_len >= 3 && cmd_bytes[0] == 0x55) {
            uint8_t cmd_type    = cmd_bytes[1];
            uint8_t cmd_code    = cmd_bytes[2];
            size_t  payload_len = (fc->is_prefix && data && data_len > 0) ? data_len : 0;
            uint8_t frame[ZIGBEE_COMMAND_LEN + 260];
            frame[0] = 0x55;
            frame[1] = (uint8_t)(3 + payload_len);
            frame[2] = cmd_type;
            frame[3] = cmd_code;
            size_t frame_pos = 4;
            if (payload_len > 0) {
                memcpy(frame + frame_pos, data, payload_len);
                frame_pos += payload_len;
            }
            uint8_t xor_chk = cmd_type ^ cmd_code;
            for (size_t i = 0; i < payload_len; i++) xor_chk ^= data[i];
            frame[frame_pos++] = xor_chk;
            ret = module_bus_write(stack_id, port_type, frame, frame_pos);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "HEX frame write failed: %s", esp_err_to_name(ret));
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
        /* Build pattern for response matching (ASCII or binary) */
        uint8_t  resp_pattern[16];
        size_t   resp_pattern_len = 0;
        if (!fc->is_hex) {
            size_t ascii_len = strlen(fc->expect_response);
            if (ascii_len > 0) {
                if (ascii_len > sizeof(resp_pattern)) ascii_len = sizeof(resp_pattern);
                memcpy(resp_pattern, fc->expect_response, ascii_len);
                resp_pattern_len = ascii_len;
            }
        } else {
            resp_pattern_len = hex_str_to_bytes(fc->expect_response,
                                                resp_pattern, sizeof(resp_pattern));
        }
        ret = zigbee_read_until(
            stack_id, port_type,
            resp_pattern_len > 0 ? resp_pattern : NULL,
            resp_pattern_len,
            fc->timeout_ms,
            result->response, ZIGBEE_RESP_BUF_SIZE - 1,
            &resp_len);

        if (ret != ESP_OK && resp_pattern_len > 0) {
            ESP_LOGW(TAG, "Zigbee response validation failed for func %d", func_id);
            xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
            result->status       = ESP_ERR_INVALID_RESPONSE;
            result->response_len = (uint16_t)resp_len;
            return ESP_ERR_INVALID_RESPONSE;
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
