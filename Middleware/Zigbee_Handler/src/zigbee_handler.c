/**
 * @file zigbee_handler.c
 * @brief Zigbee Handler Middleware Implementation
 *
 * Differs from lora_handler.c in:
 *  - Binary HEX frame building: [0x55][LEN][CMD_TYPE][CMD_CODE][DATA][XOR]
 *  - cmd_type == -1 → AT-mode: send ASCII command[] string
 *  - cmd_type >= 0  → HEX-mode: build binary frame from (cmd_type, cmd_code, data)
 *  - Module mode tracking per stack (AT → HEX transition via MODULE_ENTER_HEX_MODE)
 *  - Response is binary; terminator check uses expect_response_bytes[] (hex pattern)
 *  - Bus-mutex timeout 5 000 ms
 */

#include "zigbee_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "json_zigbee_config_parser.h"
#include "module_config_controller.h"
#include <stdlib.h>
#include <string.h>

static bool g_zigbee_module_ctrl_initialized = false;

static const char *TAG = "ZIGBEE_HANDLER";

/* ===== Configuration Constants ===== */

#define ZIGBEE_FRAME_START      0x55
#define ZIGBEE_BUS_MUTEX_MS     5000
#define ZIGBEE_LISTEN_MUTEX_MS  50
#define ZIGBEE_LISTEN_WINDOW_MS 100
#define ZIGBEE_CHUNK_SIZE       128

/* ===== Static Data ===== */

typedef enum {
    ZIGBEE_MODULE_MODE_AT  = 0,
    ZIGBEE_MODULE_MODE_HEX = 1
} zigbee_module_mode_t;

static struct {
    bool initialized;
    zigbee_module_config_t  config[ZIGBEE_MAX_STACKS];
    zigbee_module_mode_t    mode[ZIGBEE_MAX_STACKS];
} g_zigbee = {0};

static SemaphoreHandle_t g_zigbee_mutex                      = NULL;
static SemaphoreHandle_t g_zigbee_bus_mutex[ZIGBEE_MAX_STACKS] = {NULL, NULL};

/* ===== Internal Helpers ===== */

static bool is_valid_stack(uint8_t sid) { return (sid == 0 || sid == 1); }

static comm_port_type_t get_port(uint8_t sid) {
    const char *p = g_zigbee.config[sid].comm_port_type;
    if (strcmp(p, "uart") == 0) return COMM_PORT_UART;
    if (strcmp(p, "spi")  == 0) return COMM_PORT_SPI;
    if (strcmp(p, "i2c")  == 0) return COMM_PORT_I2C;
    if (strcmp(p, "usb")  == 0) return COMM_PORT_USB;
    return COMM_PORT_MAX;
}

/**
 * @brief Build a binary HEX frame for the Zigbee module.
 *
 * Frame: [0x55][LEN][CMD_TYPE][CMD_CODE][DATA 0-252][XOR]
 * LEN = 2 + data_len
 * XOR = CMD_TYPE ^ CMD_CODE ^ data[0..n-1]
 *
 * @return ESP_OK; ESP_ERR_INVALID_SIZE if data_len > 252 or frame_out too small
 */
static esp_err_t build_hex_frame(uint8_t cmd_type, uint8_t cmd_code,
                                  const uint8_t *data, uint8_t data_len,
                                  uint8_t *frame_out, uint8_t *frame_len_out) {
    if (data_len > 252) {
        ESP_LOGE(TAG, "Data too long for Zigbee frame: %d bytes", data_len);
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t total  = 4 + data_len + 1; // SOF + LEN + CMD_TYPE + CMD_CODE + data + XOR
    uint8_t len    = 2 + data_len;
    uint8_t xor_cs = cmd_type ^ cmd_code;
    for (uint8_t i = 0; i < data_len; i++) {
        xor_cs ^= data[i];
    }
    uint8_t idx = 0;
    frame_out[idx++] = ZIGBEE_FRAME_START;
    frame_out[idx++] = len;
    frame_out[idx++] = cmd_type;
    frame_out[idx++] = cmd_code;
    for (uint8_t i = 0; i < data_len; i++) {
        frame_out[idx++] = data[i];
    }
    frame_out[idx++] = xor_cs;
    *frame_len_out = total;
    return ESP_OK;
}

/**
 * @brief Read from UART accumulating binary data until expect_bytes found or timeout.
 *
 * When expect_len == 0 the function returns after receiving any data within timeout.
 */
static esp_err_t zigbee_read_until(uint8_t sid, comm_port_type_t port_type,
                                    const uint8_t *expect_bytes, uint8_t expect_len,
                                    uint32_t timeout_ms,
                                    uint8_t *out_buf, size_t out_max,
                                    size_t *out_len) {
    TickType_t start  = xTaskGetTickCount();
    TickType_t limit  = pdMS_TO_TICKS(timeout_ms);
    uint8_t    chunk[ZIGBEE_CHUNK_SIZE];
    size_t     acc    = 0;
    bool       found  = false;

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
            size_t space = out_max - acc;
            size_t copy  = (chunk_len < space) ? chunk_len : space;
            if (copy > 0) {
                memcpy(out_buf + acc, chunk, copy);
                acc += copy;
            }
            if (expect_len > 0 && acc >= expect_len) {
                // Scan accumulated buffer for the pattern
                for (size_t i = 0; i <= acc - expect_len; i++) {
                    if (memcmp(out_buf + i, expect_bytes, expect_len) == 0) {
                        found = true;
                        break;
                    }
                }
            } else if (expect_len == 0) {
                found = true;
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
        dst->functions[i].cmd_type = -1;
        dst->functions[i].cmd_code = -1;
        strncpy(dst->functions[i].response_format, "ascii",
                sizeof(dst->functions[i].response_format) - 1);
    }

    for (int i = 0; i < ZIGBEE_MAX_FUNCTIONS; i++) {
        json_zigbee_function_config_t *src = &parsed->functions[i];
        if (!src->available ||
            (zigbee_function_id_t)src->function_id >= ZIGBEE_FUNC_COUNT) {
            continue;
        }
        zigbee_function_config_t *d = &dst->functions[src->function_id];
        d->available          = true;
        d->cmd_type           = src->cmd_type;
        d->cmd_code           = src->cmd_code;
        d->is_prefix          = src->is_prefix;
        d->is_async_event     = src->is_async_event;
        d->timeout_ms         = src->timeout_ms;
        d->expect_response_len= src->expect_response_len;
        d->delay_start_ms     = src->delay_start_ms;
        d->delay_end_ms       = src->delay_end_ms;
        strncpy(d->command, src->command, ZIGBEE_COMMAND_LEN - 1);
        strncpy(d->response_format, src->response_format, 7);
        memcpy(d->expect_response_bytes, src->expect_response_bytes,
               ZIGBEE_RESPONSE_BYTES);
        memcpy(d->gpio_start, src->gpio_start, sizeof(src->gpio_start));
        d->gpio_start_count = src->gpio_start_count;
        memcpy(d->gpio_end, src->gpio_end, sizeof(src->gpio_end));
        d->gpio_end_count = src->gpio_end_count;
    }

    /* Default mode: AT for stack that just loaded */
    g_zigbee.mode[stack_id] = ZIGBEE_MODULE_MODE_AT;

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

    if (fc->cmd_type == -1) {
        /* ===== AT-mode command ===== */
        size_t cmd_len = strlen(fc->command);
        char at_buf[ZIGBEE_COMMAND_LEN + 4];
        if (cmd_len > 0) {
            snprintf(at_buf, sizeof(at_buf), "%s\r\n", fc->command);
            ret = module_bus_write(stack_id, port_type,
                                   (const uint8_t *)at_buf, strlen(at_buf));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "AT write failed: %s", esp_err_to_name(ret));
                xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
                if (result) result->status = ret;
                return ret;
            }
            /* If this is the ENTER_HEX_MODE command, track mode change */
            if (func_id == ZIGBEE_FUNC_ENTER_HEX_MODE) {
                vTaskDelay(pdMS_TO_TICKS(200));
                g_zigbee.mode[stack_id] = ZIGBEE_MODULE_MODE_HEX;
                ESP_LOGI(TAG, "Stack %d mode → HEX", stack_id);
            }
        }
    } else {
        /* ===== Binary HEX-mode frame ===== */
        uint8_t frame[ZIGBEE_FRAME_MAX_LEN];
        uint8_t frame_len = 0;
        ret = build_hex_frame((uint8_t)fc->cmd_type, (uint8_t)fc->cmd_code,
                              data, data_len, frame, &frame_len);
        if (ret != ESP_OK) {
            xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
            if (result) result->status = ret;
            return ret;
        }
        ret = module_bus_write(stack_id, port_type, frame, frame_len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "HEX frame write failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(g_zigbee_bus_mutex[stack_id]);
            if (result) result->status = ret;
            return ret;
        }
    }

    /* === Read response === */
    size_t resp_len = 0;
    bool skip_read  = (fc->timeout_ms == 0);

    if (!skip_read && result) {
        ret = zigbee_read_until(
            stack_id, port_type,
            fc->expect_response_len > 0 ? fc->expect_response_bytes : NULL,
            fc->expect_response_len,
            fc->timeout_ms,
            result->response, ZIGBEE_RESP_BUF_SIZE,
            &resp_len);

        if (ret != ESP_OK && fc->expect_response_len > 0) {
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
