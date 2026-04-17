/**
 * @file lora_handler.c
 * @brief LoRa Handler Middleware Implementation
 *
 * Mirrors ble_handler.c with LoRa-specific differences:
 *  - Generic CRLF append (no AT prefix guard) for RAK3172 / RN2483 / Seeed E5 compatibility
 *  - Bus-mutex timeout 10 000 ms (JOIN may take up to 10 s)
 *  - Per-chunk read timeout 500 ms (LoRa bus may be slower between bursts)
 *  - Listener read window 100 ms
 *  - No enter_cmd_mode function
 */

#include "lora_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "json_lora_config_parser.h"
#include "module_config_controller.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "LORA_HANDLER";

/* ===== Configuration Constants ===== */

#define LORA_BINARY_CMD_MARKER 0xC0   // Binary protocol marker
#define LORA_CMD_MAX_LEN 128          // Max command string length
#define LORA_RESPONSE_MAX_LEN 2048    // Max response accumulation buffer
#define LORA_RESPONSE_CHUNK 128       // Bus read chunk size per iteration
#define LORA_MAX_STACKS 2             // Number of stacks (0 and 1)

/* ===== GPIO Pin ID Sentinels ===== */
// LoRa uses numeric-only GPIO pins: "X1"-"X9" where X is stack ID digit.
#define LORA_GPIO_PIN_ID_RESET 10     // Sentinel for RST pin ("XR" in JSON)

/**
 * @brief Read from bus (module_bus_read), accumulating chunks until expect_response found or timeout.
 *
 * LoRa version uses 500 ms per-chunk timeout (vs 200 ms for BLE) because
 * LoRa bus responses can have longer inter-character gaps between bursts
 * (e.g., JOIN accept lines, link-check replies, downlink windows 1 & 2).
 *
 * @param stack_id         Stack identifier
 * @param port_type        Communication port
 * @param expect_response  Terminator string to wait for (NULL/"" to skip check)
 * @param timeout_ms       Maximum total wait time in milliseconds
 * @param out_buf          Caller-allocated buffer (must be '\0'-initialised)
 * @param out_max          sizeof(out_buf) including null terminator
 * @param out_len          Bytes written to out_buf (not counting '\0')
 * @return ESP_OK if terminator found; ESP_ERR_TIMEOUT otherwise
 */
static esp_err_t lora_read_until_terminator(uint8_t stack_id,
                                             comm_port_type_t port_type,
                                             const char *expect_response,
                                             uint32_t timeout_ms,
                                             char *out_buf,
                                             size_t out_max,
                                             size_t *out_len)
{
    size_t    expect_len  = (expect_response != NULL) ? strlen(expect_response) : 0;
    TickType_t start_tick  = xTaskGetTickCount();
    TickType_t timeout_tick = pdMS_TO_TICKS(timeout_ms);
    uint8_t   chunk[LORA_RESPONSE_CHUNK];
    size_t    acc   = 0;
    bool      found = false;

    out_buf[0] = '\0';
    *out_len   = 0;

    while ((xTaskGetTickCount() - start_tick) < timeout_tick) {
        TickType_t elapsed  = xTaskGetTickCount() - start_tick;
        TickType_t left     = timeout_tick - elapsed;
        uint32_t   chunk_ms = (uint32_t)(left * portTICK_PERIOD_MS);
        /* LoRa bus bursts can have 500 ms gaps between lines */
        if (chunk_ms > 500U) chunk_ms = 500U;

        size_t    chunk_len = 0;
        esp_err_t r = module_bus_read(stack_id, port_type,
                                      chunk, sizeof(chunk) - 1,
                                      chunk_ms, &chunk_len);
        if (r != ESP_OK && r != ESP_ERR_TIMEOUT) {
            break; /* Hard bus error */
        }

        if (chunk_len > 0) {
            size_t raw_len = chunk_len;
            size_t space   = out_max - 1 - acc;
            if (chunk_len > space) chunk_len = space;
            if (chunk_len > 0) {
                memcpy(out_buf + acc, chunk, chunk_len);
                acc        += chunk_len;
                out_buf[acc] = '\0';
            }
            if (expect_len > 0) {
                chunk[raw_len] = '\0';
                if (strstr(out_buf, expect_response) != NULL ||
                    (space == 0 && strstr((char *)chunk, expect_response) != NULL)) {
                    found = true;
                    break;
                }
            }
            if (expect_len == 0) {
                found = true;
                break;
            }
        }
    }

    *out_len = acc;
    return found ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ===== Hex helpers ===== */

static size_t hex_str_to_bytes(const char *hex_str, uint8_t *buf, size_t out_max) {
    if (!hex_str || !buf || out_max == 0) return 0;
    size_t n = 0;
    const char *p = hex_str;
    while (*p && n < out_max) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        unsigned int bval = 0;
        int consumed = 0;
        if (sscanf(p, "%2x%n", &bval, &consumed) != 1 || consumed == 0) break;
        buf[n++] = (uint8_t)bval;
        p += consumed;
    }
    return n;
}

/**
 * @brief Format a binary byte buffer as space-separated uppercase hex string.
 *        e.g. {0x55,0x00,0x03} → "55 00 03"
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

/**
 * @brief Binary read variant: accumulate raw bytes and match with memmem.
 *        Used for is_hex=true commands where response is binary, not ASCII.
 *        Data collection with early exit on memmem; caller validates with strstr.
 */
static esp_err_t lora_read_until_binary(uint8_t stack_id,
                                         comm_port_type_t port_type,
                                         const uint8_t *expect_bytes,
                                         size_t expect_len,
                                         uint8_t *out_buf,
                                         size_t out_max,
                                         size_t *out_len,
                                         uint32_t timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    TickType_t limit = pdMS_TO_TICKS(timeout_ms);
    uint8_t    chunk[LORA_RESPONSE_CHUNK];
    size_t     acc   = 0;
    bool       found = false;
    *out_len = 0;

    while ((xTaskGetTickCount() - start) < limit) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        TickType_t left    = limit - elapsed;
        uint32_t   win_ms  = (uint32_t)(left * portTICK_PERIOD_MS);
        if (win_ms > 500U) win_ms = 500U;

        size_t    chunk_len = 0;
        esp_err_t r = module_bus_read(stack_id, port_type,
                                      chunk, sizeof(chunk), win_ms, &chunk_len);
        if (r != ESP_OK && r != ESP_ERR_TIMEOUT) break;

        if (chunk_len > 0) {
            size_t space = out_max - acc;
            size_t copy  = chunk_len < space ? chunk_len : space;
            if (copy > 0) {
                memcpy(out_buf + acc, chunk, copy);
                acc += copy;
            }
            if (expect_len == 0) {
                found = true;
                break;
            }
            if (acc >= expect_len &&
                memmem(out_buf, acc, expect_bytes, expect_len) != NULL) {
                found = true;
                break;
            }
        }
    }

    *out_len = acc;
    return found ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ===== Static Data ===== */

static struct {
    bool initialized;
    lora_module_config_t config[LORA_MAX_STACKS];
} g_lora_handler = {0};

/** Protects g_lora_handler from multi-stack race conditions */
static SemaphoreHandle_t g_lora_handler_mutex = NULL;

/**
 * Per-stack bus mutex: serialises UART access between the command execution
 * task and the background listener task so the listener cannot consume bytes
 * that belong to a pending command response.
 */
static SemaphoreHandle_t g_lora_bus_mutex[LORA_MAX_STACKS] = {NULL, NULL};

/* ===== Helper Functions ===== */

static bool lora_is_valid_stack_id(uint8_t stack_id) {
    return (stack_id == 0 || stack_id == 1);
}

static comm_port_type_t lora_get_comm_port(uint8_t stack_id) {
    if (!lora_is_valid_stack_id(stack_id)) {
        return COMM_PORT_MAX;
    }
    const char *port = g_lora_handler.config[stack_id].comm_port_type;
    if (strcmp(port, "uart") == 0) return COMM_PORT_UART;
    if (strcmp(port, "spi")  == 0) return COMM_PORT_SPI;
    if (strcmp(port, "i2c")  == 0) return COMM_PORT_I2C;
    if (strcmp(port, "usb")  == 0) return COMM_PORT_USB;
    return COMM_PORT_MAX;
}

static bool lora_parse_pin_id(const char *pin_str, uint8_t *pin_out) {
    if (!pin_str || strlen(pin_str) < 2 || !pin_out) {
        return false;
    }
    /* "XR" -> RST sentinel */
    char second = pin_str[1];
    if (second == 'R' || second == 'r') {
        *pin_out = LORA_GPIO_PIN_ID_RESET;
        return true;
    }

    /* Numeric GPIO pin: "01"-"09" or "GPIO1"-"GPIO9" */
    const char *digits = pin_str;
    if (strncmp(pin_str, "GPIO", 4) == 0) {
        digits = pin_str + 4;
    }
    if (*digits == '\0') return false;

    char *end_ptr = NULL;
    long pin_val = strtol(digits, &end_ptr, 10);
    if (end_ptr == digits || pin_val < 1 || pin_val > 17) return false;

    *pin_out = (uint8_t)pin_val;
    return true;
}

static void lora_format_pin_str(uint8_t stack_id, uint8_t pin_id,
                                 char *pin_str, size_t sz) {
    /* Format as "X%02d" so module_gpio_write receives e.g. "004" or "010" */
    snprintf(pin_str, sz, "%d%02d", stack_id, (int)pin_id);
}

static lora_function_config_t *
lora_get_function_config(uint8_t stack_id, lora_function_id_t func_id) {
    if (!lora_is_valid_stack_id(stack_id) || func_id >= LORA_FUNC_COUNT) {
        return NULL;
    }
    return &g_lora_handler.config[stack_id].functions[func_id];
}

static bool lora_validate_command_string(const char *cmd, size_t max_len) {
    if (!cmd) return false;
    size_t cmd_len = strlen(cmd);
    if (cmd_len == 0 || cmd_len > max_len) {
        ESP_LOGW(TAG, "Command length invalid: %zu (max: %zu)", cmd_len, max_len);
        return false;
    }
    /* Binary format */
    if (cmd[0] == (char)LORA_BINARY_CMD_MARKER) return true;

    /* Generic printable ASCII validation (LoRa uses "AT+", "mac ", "sys ", "radio ") */
    for (size_t i = 0; i < cmd_len; i++) {
        if (cmd[i] < 0x20 || cmd[i] > 0x7E) {
            if (cmd[i] != '\r' && cmd[i] != '\n' && cmd[i] != '\t') {
                ESP_LOGW(TAG, "Suspicious char at index %zu: 0x%02X", i, (uint8_t)cmd[i]);
                return false;
            }
        }
    }
    return true;
}

/* ===== Internal Execute ===== */

/**
 * @brief Execute a LoRa function with optional parameter.
 *
 * Bus mutex is held for the write+read cycle with a 10 000 ms timeout to
 * accommodate JOIN (may wait for network ack in both RX windows ~5-10 s).
 */
static esp_err_t lora_execute_function_internal(uint8_t stack_id,
                                                 lora_function_id_t func_id,
                                                 const char *param,
                                                 lora_exec_result_t *result) {
    if (!lora_is_valid_stack_id(stack_id) || func_id >= LORA_FUNC_COUNT) {
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }

    lora_function_config_t *func_cfg = lora_get_function_config(stack_id, func_id);
    if (!func_cfg || !func_cfg->available) {
        ESP_LOGW(TAG, "Function %d not configured for stack %d", func_id, stack_id);
        if (result) result->status = ESP_ERR_NOT_SUPPORTED;
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGD(TAG, "Executing LoRa function %d on stack %d", func_id, stack_id);

    esp_err_t  ret        = ESP_OK;
    TickType_t start_tick = xTaskGetTickCount();

    /* Step 1: GPIO start sequences */
    for (uint8_t i = 0; i < func_cfg->gpio_start_count; i++) {
        char pin_str[8];
        lora_format_pin_str(stack_id, func_cfg->gpio_start[i], pin_str, sizeof(pin_str));
        bool state = func_cfg->gpio_start_state[i];
        ret = module_gpio_write(stack_id, pin_str, state);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed GPIO start pin %s: %s", pin_str, esp_err_to_name(ret));
            if (result) result->status = ret;
            return ret;
        }
        ESP_LOGD(TAG, "GPIO pin %s set to %d", pin_str, state);
    }

    /* Step 2: delay_start_ms */
    if (func_cfg->delay_start_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(func_cfg->delay_start_ms));
    }

    /* Build final command (substitute {PARAM} placeholder) */
    char final_command[LORA_CMD_MAX_LEN] = {0};
    if (param && strstr(func_cfg->command, "{PARAM}")) {
        char *src = func_cfg->command;
        char *dest = final_command;
        size_t dest_remaining = sizeof(final_command) - 1;
        while (*src && dest_remaining > 0) {
            if (strncmp(src, "{PARAM}", 7) == 0) {
                size_t param_len = strlen(param);
                if (param_len > dest_remaining) {
                    ESP_LOGE(TAG, "Parameter too long");
                    if (result) result->status = ESP_ERR_INVALID_SIZE;
                    return ESP_ERR_INVALID_SIZE;
                }
                memcpy(dest, param, param_len);
                dest += param_len;
                dest_remaining -= param_len;
                src += 7;
            } else {
                *dest++ = *src++;
                dest_remaining--;
            }
        }
        *dest = '\0';
    } else {
        strncpy(final_command, func_cfg->command, sizeof(final_command) - 1);
    }

    size_t cmd_len    = strlen(final_command);
    size_t expect_len = strlen(func_cfg->expect_response);
    bool is_gpio_only = (cmd_len == 0 && expect_len == 0);

    if (is_gpio_only) {
        ESP_LOGI(TAG, "GPIO-only function %d - no command/response expected", func_id);
        for (uint8_t i = 0; i < func_cfg->gpio_end_count; i++) {
            char pin_str[8];
            lora_format_pin_str(stack_id, func_cfg->gpio_end[i], pin_str, sizeof(pin_str));
            module_gpio_write(stack_id, pin_str, func_cfg->gpio_end_state[i]);
        }
        if (func_cfg->delay_end_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(func_cfg->delay_end_ms));
        }
        uint32_t exec_time = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        if (result) {
            result->status = ESP_OK;
            snprintf(result->response, sizeof(result->response), "GPIO_OK");
            result->response_len = 7;
            result->execution_time_ms = exec_time;
        }
        ESP_LOGI(TAG, "GPIO-only function %d completed (took %lu ms)", func_id, exec_time);
        return ESP_OK;
    }

    /* Validate command string */
    if (!lora_validate_command_string(final_command, sizeof(final_command))) {
        ESP_LOGE(TAG, "Command validation failed for function %d", func_id);
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }

    /* Step 3: Send command */
    comm_port_type_t port_type = lora_get_comm_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        ESP_LOGE(TAG, "Invalid comm port for stack %d", stack_id);
        if (result) result->status = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    /* Acquire per-stack bus mutex (10 s timeout for JOIN) */
    if (xSemaphoreTake(g_lora_bus_mutex[stack_id], pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "[Stack %d] Bus mutex timeout (internal exec)", stack_id);
        if (result) result->status = ESP_ERR_TIMEOUT;
        return ESP_ERR_TIMEOUT;
    }

    /* Build write buffer based on is_hex flag */
    char    lora_cmd_buf[LORA_CMD_MAX_LEN] = {0};
    uint8_t hex_cmd_buf[LORA_CMD_MAX_LEN];
    const uint8_t *write_ptr;
    size_t         write_len;

    if (!func_cfg->is_hex) {
        /*
         * ASCII path: append \r\n only if is_crlf_terminated is set.
         */
        strncpy(lora_cmd_buf, final_command, sizeof(lora_cmd_buf) - 1);
        lora_cmd_buf[sizeof(lora_cmd_buf) - 1] = '\0';
        if (g_lora_handler.config[stack_id].crlf_terminated) {
            size_t buf_len = strlen(lora_cmd_buf);
            if (buf_len < 2 || lora_cmd_buf[buf_len - 2] != '\r' || lora_cmd_buf[buf_len - 1] != '\n') {
                if (buf_len <= sizeof(lora_cmd_buf) - 3) {
                    strcat(lora_cmd_buf, "\r\n");
                }
            }
        }
        write_ptr = (const uint8_t *)lora_cmd_buf;
        write_len = strlen(lora_cmd_buf);
    } else {
        /* HEX path: decode command hex string and send raw bytes, no CRLF */
        write_len = hex_str_to_bytes(final_command, hex_cmd_buf, sizeof(hex_cmd_buf));
        if (write_len == 0) {
            ESP_LOGE(TAG, "Failed to decode hex command for function %d", func_id);
            xSemaphoreGive(g_lora_bus_mutex[stack_id]);
            if (result) result->status = ESP_ERR_INVALID_ARG;
            return ESP_ERR_INVALID_ARG;
        }
        write_ptr = hex_cmd_buf;
    }

    ret = module_bus_write(stack_id, port_type, write_ptr, write_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
        xSemaphoreGive(g_lora_bus_mutex[stack_id]);
        if (result) result->status = ret;
        return ret;
    }

    static char response_buffer[LORA_RESPONSE_MAX_LEN];
    memset(response_buffer, 0, LORA_RESPONSE_MAX_LEN);
    size_t response_len = 0;
    bool skip_read = (expect_len == 0 && func_cfg->timeout_ms == 0);

    if (!skip_read) {
        bool response_valid;
        if (!func_cfg->is_hex) {
            /* ASCII response matching */
            ret = lora_read_until_terminator(stack_id, port_type,
                                             expect_len > 0 ? func_cfg->expect_response : NULL,
                                             func_cfg->timeout_ms,
                                             response_buffer, sizeof(response_buffer),
                                             &response_len);
            response_valid = (ret == ESP_OK);
            if (!response_valid && expect_len > 0) {
                ESP_LOGW(TAG, "Response validation failed: expected '%s'",
                         func_cfg->expect_response);
                xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                if (result) {
                    result->status = ESP_ERR_INVALID_RESPONSE;
                    snprintf(result->response, sizeof(result->response), "%s",
                             response_len > 0 ? response_buffer : "TIMEOUT");
                    result->response_len = (uint16_t)response_len;
                }
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else {
            /* HEX: collect bytes until binary-decoded pattern found or timeout,
             * then validate by formatting bytes as "XX XX XX" string and strstr. */
            uint8_t resp_pattern[16];
            size_t  resp_plen = hex_str_to_bytes(func_cfg->expect_response,
                                                  resp_pattern, sizeof(resp_pattern));
            ret = lora_read_until_binary(stack_id, port_type,
                                          resp_plen > 0 ? resp_pattern : NULL,
                                          resp_plen,
                                          (uint8_t *)response_buffer,
                                          sizeof(response_buffer),
                                          &response_len,
                                          func_cfg->timeout_ms);
            /* Convert received bytes to hex string for logging and validation */
            char hex_resp[LORA_RESPONSE_MAX_LEN * 3];
            bytes_to_hex_str((const uint8_t *)response_buffer, response_len,
                             hex_resp, sizeof(hex_resp));
            if (response_len > 0) {
                ESP_LOGI(TAG, "LoRa RX %zu bytes (HEX): %s", response_len, hex_resp);
            } else {
                ESP_LOGI(TAG, "LoRa RX: (no data)");
            }
            if (strlen(func_cfg->expect_response) > 0 &&
                strstr(hex_resp, func_cfg->expect_response) == NULL) {
                ESP_LOGW(TAG, "LoRa HEX response validation failed (expected: \"%s\")",
                         func_cfg->expect_response);
                xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                if (result) {
                    result->status = ESP_ERR_INVALID_RESPONSE;
                    memcpy(result->response, response_buffer, response_len);
                    result->response_len = (uint16_t)response_len;
                }
                return ESP_ERR_INVALID_RESPONSE;
            }
            response_valid = true;
        }
        if (response_len > 0 && !func_cfg->is_hex) {
            ESP_LOGD(TAG, "Received response (%u bytes)", (unsigned)response_len);
        }
    } else {
        ESP_LOGD(TAG, "Skipping response read (no response expected, timeout=0)");
    }

    xSemaphoreGive(g_lora_bus_mutex[stack_id]);

    /* Step 6: GPIO end sequences */
    for (uint8_t i = 0; i < func_cfg->gpio_end_count; i++) {
        char pin_str[8];
        lora_format_pin_str(stack_id, func_cfg->gpio_end[i], pin_str, sizeof(pin_str));
        ret = module_gpio_write(stack_id, pin_str, func_cfg->gpio_end_state[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed GPIO end pin %s: %s", pin_str, esp_err_to_name(ret));
        }
    }

    /* Step 7: delay_end_ms */
    if (func_cfg->delay_end_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(func_cfg->delay_end_ms));
    }

    uint32_t exec_time = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    if (result) {
        result->status = ESP_OK;
        if (!func_cfg->is_hex) {
            snprintf(result->response, sizeof(result->response), "%s",
                     response_len > 0 ? response_buffer : "OK");
        } else {
            memcpy(result->response, response_buffer, response_len);
        }
        result->response_len    = (uint16_t)response_len;
        result->execution_time_ms = exec_time;
    }

    ESP_LOGI(TAG, "Function %d executed successfully on stack %d (took %lu ms)",
             func_id, stack_id, exec_time);
    return ESP_OK;
}

/* ===== Public API Implementation ===== */

esp_err_t lora_handler_init(void) {
    if (g_lora_handler.initialized) {
        ESP_LOGW(TAG, "LoRa handler already initialized");
        return ESP_OK;
    }

    if (!g_lora_handler_mutex) {
        g_lora_handler_mutex = xSemaphoreCreateMutex();
        if (!g_lora_handler_mutex) {
            ESP_LOGE(TAG, "Failed to create LoRa handler mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    for (int i = 0; i < LORA_MAX_STACKS; i++) {
        if (!g_lora_bus_mutex[i]) {
            g_lora_bus_mutex[i] = xSemaphoreCreateMutex();
            if (!g_lora_bus_mutex[i]) {
                ESP_LOGE(TAG, "Failed to create bus mutex for stack %d", i);
                return ESP_ERR_NO_MEM;
            }
        }
    }

    memset(&g_lora_handler, 0, sizeof(g_lora_handler));
    g_lora_handler.initialized = true;
    ESP_LOGI(TAG, "LoRa handler initialized successfully");
    return ESP_OK;
}

esp_err_t lora_handler_load_config(uint8_t stack_id, const char *json_config,
                                    uint16_t json_len) {
    if (!g_lora_handler.initialized) {
        ESP_LOGE(TAG, "LoRa handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (!lora_is_valid_stack_id(stack_id) || !json_config || json_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Loading LoRa config for stack %d (%d bytes)", stack_id, json_len);

    if (xSemaphoreTake(g_lora_handler_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire LoRa handler mutex");
        return ESP_ERR_TIMEOUT;
    }

    /* Heap-allocate parsed config to avoid stack overflow of ~5 KB struct */
    json_lora_module_config_t *parsed =
        (json_lora_module_config_t *)calloc(1, sizeof(json_lora_module_config_t));
    if (!parsed) {
        ESP_LOGE(TAG, "Failed to allocate parsed config buffer");
        xSemaphoreGive(g_lora_handler_mutex);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = json_lora_config_parse(json_config, parsed);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse LoRa JSON config: %s", esp_err_to_name(ret));
        free(parsed);
        xSemaphoreGive(g_lora_handler_mutex);
        return ret;
    }

    memset(&g_lora_handler.config[stack_id], 0, sizeof(lora_module_config_t));
    g_lora_handler.config[stack_id].module_id = stack_id;
    strncpy(g_lora_handler.config[stack_id].module_type,
            parsed->metadata.module_type,
            sizeof(g_lora_handler.config[stack_id].module_type) - 1);
    strncpy(g_lora_handler.config[stack_id].module_name,
            parsed->metadata.module_name,
            sizeof(g_lora_handler.config[stack_id].module_name) - 1);
    g_lora_handler.config[stack_id].crlf_terminated = parsed->metadata.crlf_terminated;

    switch (parsed->metadata.communication.port_type) {
    case COMM_PORT_UART:
        strncpy(g_lora_handler.config[stack_id].comm_port_type, "uart",
                sizeof(g_lora_handler.config[stack_id].comm_port_type) - 1);
        g_lora_handler.config[stack_id].baudrate =
            parsed->metadata.communication.params.uart.baudrate;
        ret = module_config_controller_init_uart(
            stack_id, &parsed->metadata.communication.params.uart);
        break;
    case COMM_PORT_SPI:
        strncpy(g_lora_handler.config[stack_id].comm_port_type, "spi",
                sizeof(g_lora_handler.config[stack_id].comm_port_type) - 1);
        ret = module_config_controller_init_spi(
            stack_id, &parsed->metadata.communication.params.spi);
        break;
    case COMM_PORT_I2C:
        strncpy(g_lora_handler.config[stack_id].comm_port_type, "i2c",
                sizeof(g_lora_handler.config[stack_id].comm_port_type) - 1);
        ret = module_config_controller_init_i2c(
            stack_id, &parsed->metadata.communication.params.i2c);
        break;
    case COMM_PORT_USB:
        strncpy(g_lora_handler.config[stack_id].comm_port_type, "usb",
                sizeof(g_lora_handler.config[stack_id].comm_port_type) - 1);
        ret = module_config_controller_init_usb(
            stack_id, &parsed->metadata.communication.params.usb);
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init communication for stack %d: %s", stack_id,
                 esp_err_to_name(ret));
        free(parsed);
        xSemaphoreGive(g_lora_handler_mutex);
        return ret;
    }

    for (int i = 0; i < LORA_FUNC_COUNT; i++) {
        g_lora_handler.config[stack_id].functions[i].available = false;
    }

    for (int i = 0; i < LORA_MAX_FUNCTIONS; i++) {
        json_lora_function_config_t *src = &parsed->functions[i];
        if (!src->available ||
            (lora_function_id_t)src->function_id >= LORA_FUNC_COUNT) {
            continue;
        }

        lora_function_config_t *dst =
            &g_lora_handler.config[stack_id].functions[src->function_id];
        dst->available = true;
        dst->is_hex    = src->is_hex;
        dst->is_prefix = src->is_prefix;
        strncpy(dst->command, src->command, sizeof(dst->command) - 1);
        strncpy(dst->expect_response, src->expect_response,
                sizeof(dst->expect_response) - 1);
        dst->delay_start_ms = src->delay_start_ms;
        dst->delay_end_ms   = src->delay_end_ms;
        dst->timeout_ms     = src->timeout_ms;

        dst->gpio_start_count = 0;
        for (uint8_t j = 0; j < src->gpio_start_count; j++) {
            uint8_t pin_id = 0;
            if (!lora_parse_pin_id(src->gpio_start[j].pin, &pin_id)) {
                ESP_LOGE(TAG, "Invalid GPIO start pin: %s", src->gpio_start[j].pin);
                free(parsed);
                xSemaphoreGive(g_lora_handler_mutex);
                return ESP_ERR_INVALID_ARG;
            }
            dst->gpio_start[dst->gpio_start_count]       = pin_id;
            dst->gpio_start_state[dst->gpio_start_count] = src->gpio_start[j].state;
            dst->gpio_start_count++;
        }

        dst->gpio_end_count = 0;
        for (uint8_t j = 0; j < src->gpio_end_count; j++) {
            uint8_t pin_id = 0;
            if (!lora_parse_pin_id(src->gpio_end[j].pin, &pin_id)) {
                ESP_LOGE(TAG, "Invalid GPIO end pin: %s", src->gpio_end[j].pin);
                free(parsed);
                xSemaphoreGive(g_lora_handler_mutex);
                return ESP_ERR_INVALID_ARG;
            }
            dst->gpio_end[dst->gpio_end_count]       = pin_id;
            dst->gpio_end_state[dst->gpio_end_count] = src->gpio_end[j].state;
            dst->gpio_end_count++;
        }
    }

    free(parsed);
    xSemaphoreGive(g_lora_handler_mutex);
    ESP_LOGI(TAG, "LoRa config loaded for stack %d", stack_id);
    return ESP_OK;
}

/* ===== Core Functions ===== */

esp_err_t lora_handler_hw_reset(uint8_t stack_id) {
    if (!g_lora_handler.initialized) {
        ESP_LOGE(TAG, "LoRa handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    lora_exec_result_t result = {0};
    esp_err_t ret = lora_execute_function_internal(stack_id, LORA_FUNC_HW_RESET,
                                                    NULL, &result);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Hardware reset executed on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Hardware reset failed on stack %d: %s", stack_id, result.response);
    }
    return ret;
}

esp_err_t lora_handler_sw_reset(uint8_t stack_id) {
    if (!g_lora_handler.initialized) {
        ESP_LOGE(TAG, "LoRa handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    lora_exec_result_t result = {0};
    esp_err_t ret = lora_execute_function_internal(stack_id, LORA_FUNC_SW_RESET,
                                                    NULL, &result);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Software reset executed on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Software reset failed on stack %d: %s", stack_id, result.response);
    }
    return ret;
}

esp_err_t lora_handler_factory_reset(uint8_t stack_id) {
    if (!g_lora_handler.initialized) {
        ESP_LOGE(TAG, "LoRa handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    lora_exec_result_t result = {0};
    esp_err_t ret = lora_execute_function_internal(stack_id, LORA_FUNC_FACTORY_RESET,
                                                    NULL, &result);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Factory reset executed on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Factory reset failed on stack %d: %s", stack_id, result.response);
    }
    return ret;
}

esp_err_t lora_handler_get_info(uint8_t stack_id, char *buffer, size_t max_len) {
    if (!g_lora_handler.initialized || !buffer || max_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    lora_exec_result_t result = {0};
    esp_err_t ret = lora_execute_function_internal(stack_id, LORA_FUNC_GET_INFO,
                                                    NULL, &result);
    if (ret == ESP_OK) {
        strncpy(buffer, result.response, max_len - 1);
        buffer[max_len - 1] = '\0';
        ESP_LOGI(TAG, "Get info on stack %d: %s", stack_id, buffer);
    } else {
        ESP_LOGE(TAG, "Get info failed on stack %d", stack_id);
    }
    return ret;
}

esp_err_t lora_handler_join(uint8_t stack_id) {
    if (!g_lora_handler.initialized) {
        ESP_LOGE(TAG, "LoRa handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    lora_exec_result_t result = {0};
    esp_err_t ret = lora_execute_function_internal(stack_id, LORA_FUNC_JOIN,
                                                    NULL, &result);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "JOIN executed on stack %d: %s", stack_id, result.response);
    } else {
        ESP_LOGE(TAG, "JOIN failed on stack %d: %s", stack_id, result.response);
    }
    return ret;
}

esp_err_t lora_handler_get_join_status(uint8_t stack_id, char *buffer,
                                        size_t max_len) {
    if (!g_lora_handler.initialized || !buffer || max_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    lora_exec_result_t result = {0};
    esp_err_t ret = lora_execute_function_internal(stack_id, LORA_FUNC_GET_JOIN_STATUS,
                                                    NULL, &result);
    if (ret == ESP_OK) {
        strncpy(buffer, result.response, max_len - 1);
        buffer[max_len - 1] = '\0';
        ESP_LOGI(TAG, "Join status on stack %d: %s", stack_id, buffer);
    } else {
        ESP_LOGE(TAG, "Get join status failed on stack %d", stack_id);
    }
    return ret;
}

/* ===== Command Matching ===== */

/**
 * Function name table for GPIO-only trigger resolution.
 * Indices match the lora_function_id_t enum values.
 */
static const char *s_lora_func_names[LORA_FUNC_COUNT] = {
    "MODULE_HW_RESET",          // 0
    "MODULE_SW_RESET",          // 1
    "MODULE_GET_INFO",          // 2
    "MODULE_FACTORY_RESET",     // 3
    "MODULE_SET_REGION",        // 4
    "MODULE_SET_CLASS",         // 5
    "MODULE_SET_JOIN_MODE",     // 6
    "MODULE_SET_DEVEUI",        // 7
    "MODULE_GET_DEVEUI",        // 8
    "MODULE_SET_APPEUI",        // 9
    "MODULE_SET_APPKEY",        // 10
    "MODULE_JOIN",              // 11
    "MODULE_GET_JOIN_STATUS",   // 12
    "MODULE_SET_DEVADDR",       // 13
    "MODULE_SET_NWKSKEY",       // 14
    "MODULE_SET_APPSKEY",       // 15
    "MODULE_SET_DR",            // 16
    "MODULE_SET_ADR",           // 17
    "MODULE_SET_TXP",           // 18
    "MODULE_SET_CHANNEL",       // 19
    "MODULE_SET_CONFIRM",       // 20
    "MODULE_SET_PUBLIC_NET",    // 21
    "MODULE_SEND_UNCONFIRMED",  // 22
    "MODULE_SEND_CONFIRMED",    // 23
    "MODULE_READ_RECV",         // 24
    "MODULE_SET_PORT",          // 25
    "MODULE_GET_DEVADDR",       // 26
    "MODULE_SET_RETRY",         // 27
    "MODULE_SET_REPT",          // 28
    "MODULE_SET_RXWIN2",        // 29
    "MODULE_SET_DELAY",         // 30
    "MODULE_SEND_HEX",          // 31
    "MODULE_SEND_CONFIRMED_HEX",// 32
    "MODULE_CHECK_PAYLOAD_LEN", // 33
    "MODULE_GET_VDD",           // 34
    "MODULE_LOWPOWER",          // 35
    "MODULE_LOWPOWER_AUTO_ON",  // 36
    "MODULE_LOWPOWER_AUTO_OFF", // 37
    "MODULE_WAKEUP_NOTIFY",     // 38
    "MODULE_ENTER_P2P_MODE",    // 39
    "MODULE_SET_P2P_CONFIG",    // 40
    "MODULE_SEND_P2P_PKT",      // 41
    "MODULE_ENTER_P2P_RX",      // 42
    "",                         // 43 reserved
};

esp_err_t lora_handler_get_function_by_command(uint8_t stack_id,
                                                const char *command,
                                                lora_function_config_t *func_config) {
    if (!g_lora_handler.initialized || !command || !func_config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!lora_is_valid_stack_id(stack_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t cmd_len = strlen(command);

    /* Pass 1: prefix / exact match on cfg->command */
    for (int func_id = 0; func_id < LORA_FUNC_COUNT; func_id++) {
        lora_function_config_t *cfg =
            &g_lora_handler.config[stack_id].functions[func_id];
        if (!cfg->available) continue;

        size_t cfg_cmd_len = strlen(cfg->command);
        /* Strip trailing \r\n for comparison */
        while (cfg_cmd_len > 0 &&
               (cfg->command[cfg_cmd_len - 1] == '\n' ||
                cfg->command[cfg_cmd_len - 1] == '\r')) {
            cfg_cmd_len--;
        }
        if (cfg_cmd_len == 0) continue; /* GPIO-only — handled in pass 2 */

        /* AT+ commands: prefix match.  Others: exact match. */
        if (strncmp(cfg->command, "AT+", 3) == 0) {
            if (cmd_len >= cfg_cmd_len &&
                strncmp(command, cfg->command, cfg_cmd_len) == 0) {
                memcpy(func_config, cfg, sizeof(lora_function_config_t));
                ESP_LOGI(TAG, "Matched prefix: %.*s (func_id=%d)",
                         (int)cfg_cmd_len, cfg->command, func_id);
                return ESP_OK;
            }
        } else {
            if (cmd_len == cfg_cmd_len &&
                strncmp(command, cfg->command, cfg_cmd_len) == 0) {
                memcpy(func_config, cfg, sizeof(lora_function_config_t));
                ESP_LOGI(TAG, "Matched exact: %.*s (func_id=%d)",
                         (int)cfg_cmd_len, cfg->command, func_id);
                return ESP_OK;
            }
        }
    }

    /* Pass 2: function_name fallback (GPIO-only triggers) */
    for (int func_id = 0; func_id < LORA_FUNC_COUNT; func_id++) {
        lora_function_config_t *cfg =
            &g_lora_handler.config[stack_id].functions[func_id];
        if (!cfg->available) continue;
        if (s_lora_func_names[func_id] != NULL &&
            s_lora_func_names[func_id][0] != '\0' &&
            strcmp(command, s_lora_func_names[func_id]) == 0) {
            memcpy(func_config, cfg, sizeof(lora_function_config_t));
            ESP_LOGI(TAG, "Matched function_name: %s (func_id=%d)",
                     s_lora_func_names[func_id], func_id);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No function match for command: %s", command);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t lora_handler_get_function_by_name(uint8_t stack_id,
                                             const char *func_name,
                                             lora_function_config_t *func_config) {
    if (!g_lora_handler.initialized || !func_name || !func_config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!lora_is_valid_stack_id(stack_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int func_id = 0; func_id < LORA_FUNC_COUNT; func_id++) {
        if (s_lora_func_names[func_id] == NULL ||
            s_lora_func_names[func_id][0] == '\0') {
            continue;
        }
        if (strcmp(func_name, s_lora_func_names[func_id]) == 0) {
            lora_function_config_t *cfg =
                &g_lora_handler.config[stack_id].functions[func_id];
            if (!cfg->available) {
                ESP_LOGW(TAG, "Function '%s' (id=%d) not configured for stack %d",
                         func_name, func_id, stack_id);
                return ESP_ERR_NOT_SUPPORTED;
            }
            memcpy(func_config, cfg, sizeof(lora_function_config_t));
            ESP_LOGI(TAG, "Matched function_name: %s (func_id=%d)", func_name, func_id);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No function match for name: %s", func_name);
    return ESP_ERR_NOT_FOUND;
}

/* ===== Execute With Config (Task Layer) ===== */

esp_err_t lora_handler_execute_command_with_config(uint8_t stack_id,
                                                    const char *command,
                                                    const lora_function_config_t *func_config,
                                                    lora_exec_result_t *result) {
    if (!g_lora_handler.initialized || !command || !func_config) {
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }
    if (!lora_is_valid_stack_id(stack_id)) {
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Executing command '%s' on stack %d with JSON config", command, stack_id);

    esp_err_t  ret        = ESP_OK;
    TickType_t start_tick = xTaskGetTickCount();

    /* Step 1: GPIO start sequences */
    for (uint8_t i = 0; i < func_config->gpio_start_count; i++) {
        char pin_str[8];
        lora_format_pin_str(stack_id, func_config->gpio_start[i], pin_str, sizeof(pin_str));
        ret = module_gpio_write(stack_id, pin_str, func_config->gpio_start_state[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed GPIO start pin %s: %s", pin_str, esp_err_to_name(ret));
            if (result) result->status = ret;
            return ret;
        }
    }

    /* Step 2: delay_start_ms */
    if (func_config->delay_start_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(func_config->delay_start_ms));
    }

    /* Step 3: Validate and send command */
    size_t cmd_len    = strlen(command);
    size_t expect_len = strlen(func_config->expect_response);
    bool is_gpio_only = (strlen(func_config->command) == 0 && expect_len == 0);

    if (is_gpio_only) {
        ESP_LOGI(TAG, "GPIO-only function - no command/response expected");
    } else {
        if (!lora_validate_command_string(command, LORA_CMD_MAX_LEN)) {
            ESP_LOGE(TAG, "Command validation failed");
            if (result) result->status = ESP_ERR_INVALID_ARG;
            return ESP_ERR_INVALID_ARG;
        }

        comm_port_type_t port_type = lora_get_comm_port(stack_id);
        if (port_type == COMM_PORT_MAX) {
            ESP_LOGE(TAG, "Invalid comm port for stack %d", stack_id);
            if (result) result->status = ESP_ERR_INVALID_STATE;
            return ESP_ERR_INVALID_STATE;
        }

        /* Acquire bus mutex (10 s) */
        if (xSemaphoreTake(g_lora_bus_mutex[stack_id], pdMS_TO_TICKS(10000)) != pdTRUE) {
            ESP_LOGE(TAG, "[Stack %d] Bus mutex timeout (execute_with_config)", stack_id);
            if (result) result->status = ESP_ERR_TIMEOUT;
            return ESP_ERR_TIMEOUT;
        }

        static char response_buffer[LORA_RESPONSE_MAX_LEN];
        memset(response_buffer, 0, LORA_RESPONSE_MAX_LEN);
        size_t response_len = 0;
        if (!func_config->is_hex) {
            /* ASCII path: send command string with optional CRLF */
            char lora_cmd_buf[LORA_CMD_MAX_LEN] = {0};
            const uint8_t *write_ptr = (const uint8_t *)command;
            size_t write_len = cmd_len;
            if (g_lora_handler.config[stack_id].crlf_terminated &&
                (cmd_len < 2 ||
                 (command[cmd_len - 2] != '\r' || command[cmd_len - 1] != '\n'))) {
                strncpy(lora_cmd_buf, command, sizeof(lora_cmd_buf) - 3);
                lora_cmd_buf[sizeof(lora_cmd_buf) - 3] = '\0';
                strcat(lora_cmd_buf, "\r\n");
                write_ptr = (const uint8_t *)lora_cmd_buf;
                write_len = strlen(lora_cmd_buf);
                ESP_LOGD(TAG, "Appended CRLF to LoRa command (execute_with_config)");
            }

            ret = module_bus_write(stack_id, port_type, write_ptr, write_len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
                xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                if (result) result->status = ret;
                return ret;
            }

            /* Read ASCII response */
            bool skip_read = (expect_len == 0 && func_config->timeout_ms == 0);
            if (!skip_read) {
                ret = lora_read_until_terminator(stack_id, port_type,
                                                  expect_len > 0 ? func_config->expect_response : NULL,
                                                  func_config->timeout_ms,
                                                  response_buffer, sizeof(response_buffer),
                                                  &response_len);

                bool response_valid = (ret == ESP_OK);
                if (!response_valid && expect_len > 0) {
                    ESP_LOGW(TAG, "Response validation failed: expected '%s'",
                             func_config->expect_response);
                    xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                    if (result) {
                        result->status = ESP_ERR_INVALID_RESPONSE;
                        snprintf(result->response, sizeof(result->response), "%s",
                                 response_len > 0 ? response_buffer : "TIMEOUT");
                        result->response_len = (uint16_t)response_len;
                    }
                    return ESP_ERR_INVALID_RESPONSE;
                }

                if (result && response_len > 0) {
                    snprintf(result->response, sizeof(result->response), "%s",
                             response_buffer);
                    result->response_len = (uint16_t)response_len;
                }
            }
        } else {
            /* HEX path: decode command hex string, send raw bytes, binary response */
            uint8_t hex_cmd_buf[LORA_CMD_MAX_LEN];
            size_t  hex_len = hex_str_to_bytes(command, hex_cmd_buf, sizeof(hex_cmd_buf));
            if (hex_len == 0) {
                ESP_LOGE(TAG, "Failed to decode HEX command string for LoRa");
                xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                if (result) result->status = ESP_ERR_INVALID_ARG;
                return ESP_ERR_INVALID_ARG;
            }
            ESP_LOGD(TAG, "LoRa HEX TX: %zu bytes", hex_len);
            ret = module_bus_write(stack_id, port_type, hex_cmd_buf, hex_len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send HEX command: %s", esp_err_to_name(ret));
                xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                if (result) result->status = ret;
                return ret;
            }

            /* Read binary response */
            bool skip_read = (func_config->timeout_ms == 0);
            if (!skip_read) {
                uint8_t resp_pattern[16];
                size_t  resp_plen = hex_str_to_bytes(func_config->expect_response,
                                                      resp_pattern, sizeof(resp_pattern));
                ret = lora_read_until_binary(stack_id, port_type,
                                             resp_plen > 0 ? resp_pattern : NULL,
                                             resp_plen,
                                             (uint8_t *)response_buffer,
                                             sizeof(response_buffer),
                                             &response_len,
                                             func_config->timeout_ms);
                /* Convert received bytes to hex string for logging and validation */
                char hex_resp[LORA_RESPONSE_MAX_LEN * 3];
                bytes_to_hex_str((const uint8_t *)response_buffer, response_len,
                                 hex_resp, sizeof(hex_resp));
                if (response_len > 0) {
                    ESP_LOGI(TAG, "LoRa RX %zu bytes (HEX): %s", response_len, hex_resp);
                } else {
                    ESP_LOGI(TAG, "LoRa RX: (no data)");
                }
                if (strlen(func_config->expect_response) > 0 &&
                    strstr(hex_resp, func_config->expect_response) == NULL) {
                    ESP_LOGW(TAG, "LoRa HEX response validation failed (expected: \"%s\")",
                             func_config->expect_response);
                    xSemaphoreGive(g_lora_bus_mutex[stack_id]);
                    if (result) {
                        result->status = ESP_ERR_INVALID_RESPONSE;
                        memcpy(result->response, response_buffer, response_len);
                        result->response_len = (uint16_t)response_len;
                    }
                    return ESP_ERR_INVALID_RESPONSE;
                }
                if (result && response_len > 0) {
                    memcpy(result->response, response_buffer, response_len);
                    result->response_len = (uint16_t)response_len;
                }
            }
        }

        xSemaphoreGive(g_lora_bus_mutex[stack_id]);
    }

    /* Step 5: GPIO end sequences */
    for (uint8_t i = 0; i < func_config->gpio_end_count; i++) {
        char pin_str[8];
        lora_format_pin_str(stack_id, func_config->gpio_end[i], pin_str, sizeof(pin_str));
        ret = module_gpio_write(stack_id, pin_str, func_config->gpio_end_state[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed GPIO end pin %s: %s", pin_str, esp_err_to_name(ret));
        }
    }

    /* Step 6: delay_end_ms */
    if (func_config->delay_end_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(func_config->delay_end_ms));
    }

    uint32_t exec_time = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    if (result) {
        result->status = ESP_OK;
        if (result->response[0] == '\0') {
            snprintf(result->response, sizeof(result->response), "OK");
        }
        result->execution_time_ms = exec_time;
    }

    ESP_LOGI(TAG, "Command executed successfully on stack %d (took %lu ms)",
             stack_id, exec_time);
    return ESP_OK;
}

esp_err_t lora_handler_send_binary_command(uint8_t stack_id,
                                            const uint8_t *cmd_bytes,
                                            uint16_t cmd_len,
                                            uint8_t *response,
                                            uint16_t resp_len,
                                            uint16_t timeout_ms) {
    if (!lora_is_valid_stack_id(stack_id) || !cmd_bytes || cmd_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    comm_port_type_t port_type = lora_get_comm_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        ESP_LOGE(TAG, "Invalid comm port for stack %d", stack_id);
        return ESP_ERR_INVALID_STATE;
    }

    /* Acquire bus mutex (10 s) */
    if (xSemaphoreTake(g_lora_bus_mutex[stack_id], pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "[Stack %d] Bus mutex timeout (binary cmd)", stack_id);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = module_bus_write(stack_id, port_type, cmd_bytes, cmd_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send binary command: %s", esp_err_to_name(ret));
        xSemaphoreGive(g_lora_bus_mutex[stack_id]);
        return ret;
    }

    if (response && resp_len > 0) {
        size_t received_len = 0;
        ret = module_bus_read(stack_id, port_type, response, resp_len,
                              timeout_ms, &received_len);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Failed to read binary response: %s", esp_err_to_name(ret));
            xSemaphoreGive(g_lora_bus_mutex[stack_id]);
            return ret;
        }
        ESP_LOGD(TAG, "Binary response received: %zu bytes", received_len);
    }

    xSemaphoreGive(g_lora_bus_mutex[stack_id]);
    return ESP_OK;
}

/**
 * @brief Listen for unsolicited data from the LoRa module (background listener).
 *
 * Tries to acquire the per-stack bus mutex with a 50 ms timeout.  If the
 * command task currently owns the bus the function returns ESP_ERR_TIMEOUT
 * immediately so the caller (listener task) can yield and retry.
 *
 * @param stack_id  Stack ID (0 or 1)
 * @param buf       Caller-allocated output buffer
 * @param max       Buffer size in bytes (including null terminator)
 * @param out_len   Bytes written to buf (excluding null terminator)
 * @return ESP_OK with data, ESP_ERR_TIMEOUT if bus busy or no data
 */
esp_err_t lora_handler_listen(uint8_t stack_id, char *buf, size_t max,
                               size_t *out_len) {
    if (!lora_is_valid_stack_id(stack_id) || !buf || !out_len || max < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;
    buf[0]   = '\0';

    if (!g_lora_handler.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    comm_port_type_t port_type = lora_get_comm_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Non-blocking trylock – if command task holds the bus, yield immediately */
    if (xSemaphoreTake(g_lora_bus_mutex[stack_id], pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t   chunk[LORA_RESPONSE_CHUNK];
    size_t    chunk_len = 0;
    /* 100 ms read window (wider than BLE's 50 ms for slower LoRa bus) */
    esp_err_t ret = module_bus_read(stack_id, port_type, chunk,
                                    sizeof(chunk) - 1, 100, &chunk_len);

    if (chunk_len > 0) {
        size_t copy_len = (chunk_len < max - 1) ? chunk_len : max - 1;
        memcpy(buf, chunk, copy_len);
        buf[copy_len] = '\0';
        *out_len = copy_len;
        ret = ESP_OK;
    } else {
        ret = ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(g_lora_bus_mutex[stack_id]);
    return ret;
}
