/**
 * @file lora_handler_task.c
 * @brief LoRa Handler Task Implementation with Multi-Stack Support
 *
 * Mirrors ble_handler_task.c with LoRa-specific differences:
 *  - Response prefix "CFLR:" (vs "CFBL:" for BLE)
 *  - Startup sequence: hw_reset → 500 ms delay → get_info (no enter_cmd_mode)
 *  - HANDLER_LORA forwarded to mcu_wan_enqueue_uplink()
 */

#include "lora_handler_task.h"
#include "lora_handler.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "LORA_TASK";

/* ===== Configuration ===== */
#define LORA_UPLINK_TASK_STACK_SIZE    (24 * 1024)   // PSRAM stack
#define LORA_DOWNLINK_TASK_STACK_SIZE  (24 * 1024)   // PSRAM stack
#define LORA_LISTENER_TASK_STACK_SIZE  (8  * 1024)   // PSRAM stack
#define LORA_UPLINK_TASK_PRIORITY      5
#define LORA_DOWNLINK_TASK_PRIORITY    6
#define LORA_LISTENER_TASK_PRIORITY    4   // Lower than command tasks
#define LORA_UPLINK_QUEUE_SIZE         20
#define LORA_DOWNLINK_QUEUE_SIZE       20
#define LORA_COMMAND_QUEUE_SIZE        10
#define LORA_MAX_STACKS                2
#define LORA_UPLINK_BATCH_MAX          8
#define LORA_UPLINK_BATCH_FLUSH_MS     50
#define LORA_LISTEN_BUFFER_SIZE        512  // Unsolicited event receive buffer

/* ===== Static Data ===== */

static struct {
    bool          running[LORA_MAX_STACKS];
    TaskHandle_t  uplink_task_handle[LORA_MAX_STACKS];
    TaskHandle_t  downlink_task_handle[LORA_MAX_STACKS];
    TaskHandle_t  listener_task_handle[LORA_MAX_STACKS]; // Background bus listener
    StackType_t  *uplink_stack[LORA_MAX_STACKS];    // PSRAM stack buffers
    StackType_t  *downlink_stack[LORA_MAX_STACKS];
    StackType_t  *listener_stack[LORA_MAX_STACKS];
    StaticTask_t *uplink_tcb[LORA_MAX_STACKS];      // internal SRAM TCBs
    StaticTask_t *downlink_tcb[LORA_MAX_STACKS];
    StaticTask_t *listener_tcb[LORA_MAX_STACKS];
    QueueHandle_t uplink_queue[LORA_MAX_STACKS];
    QueueHandle_t downlink_queue[LORA_MAX_STACKS];
    QueueHandle_t command_queue[LORA_MAX_STACKS];
    /* PSRAM-backed queues */
    uint8_t       *ul_q_storage[LORA_MAX_STACKS];
    StaticQueue_t *ul_q_tcb[LORA_MAX_STACKS];
    uint8_t       *dl_q_storage[LORA_MAX_STACKS];
    StaticQueue_t *dl_q_tcb[LORA_MAX_STACKS];
    uint8_t       *cmd_q_storage[LORA_MAX_STACKS];
    StaticQueue_t *cmd_q_tcb[LORA_MAX_STACKS];
} g_lora_task = {0};

typedef struct {
    uint8_t stack_id;
} lora_task_context_t;

/* ===== Helper Functions ===== */

static inline bool lora_is_valid_stack(uint8_t stack_id) {
    return (stack_id < LORA_MAX_STACKS);
}

bool lora_handler_is_running(uint8_t stack_id) {
    if (!lora_is_valid_stack(stack_id)) return false;
    return g_lora_task.running[stack_id];
}

/**
 * @brief Convert binary buffer to space-separated hex string.
 *        e.g. {0x55,0x0D,0x01} → "55 0D 01"
 */
static int lora_bytes_to_hex_str(const uint8_t *bytes, size_t len,
                                  char *out, size_t out_max) {
    int pos = 0;
    for (size_t i = 0; i < len && pos + 3 < (int)out_max; i++) {
        pos += snprintf(out + pos, out_max - pos,
                        "%s%02X", (i == 0 ? "" : " "), bytes[i]);
    }
    return pos;
}

/**
 * @brief Parse a space-separated hex byte string into a binary buffer.
 *        e.g. "55 00 00" → {0x55, 0x00, 0x00}
 * @return Number of bytes written to @p buf.
 */
static size_t lora_hex_str_to_bytes(const char *hex_str, uint8_t *buf, size_t out_max) {
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
 * @brief Return true if the majority of loaded functions for this stack are
 *        HEX mode (is_hex=true).  Samples 3 well-known function names.
 */
static bool lora_stack_is_hex_mode(uint8_t sid) {
    /* Cache result per stack — refresh every 5 s to pick up config reloads
     * without spamming get_function_by_name on every listener loop tick. */
    static bool   cached_result[LORA_MAX_STACKS]    = {false, false};
    static TickType_t cached_at[LORA_MAX_STACKS]    = {0, 0};
    const  TickType_t CACHE_TTL = pdMS_TO_TICKS(5000);

    TickType_t now = xTaskGetTickCount();
    if ((now - cached_at[sid]) < CACHE_TTL) {
        return cached_result[sid];
    }

    /* Probe using function names that exist in all Wio-E5 configs */
    static const char *probe[] = {
        "MODULE_GET_INFO", "MODULE_SW_RESET", "MODULE_FACTORY_RESET"
    };
    int hex_cnt = 0, ascii_cnt = 0;
    for (int i = 0; i < 3; i++) {
        lora_function_config_t fc;
        if (lora_handler_get_function_by_name(sid, probe[i], &fc) == ESP_OK) {
            if (fc.is_hex) hex_cnt++;
            else           ascii_cnt++;
        }
    }
    bool result = (hex_cnt > 0 && hex_cnt >= ascii_cnt);
    cached_result[sid] = result;
    cached_at[sid]     = now;
    return result;
}

/* ===== Task Implementations ===== */

/**
 * @brief Uplink task – collect data from LoRa module and forward to WAN MCU.
 */
static void lora_uplink_task(void *pvParameters) {
    lora_task_context_t *ctx = (lora_task_context_t *)pvParameters;
    uint8_t stack_id = ctx->stack_id;

    ESP_LOGI(TAG, "[Stack %d] LoRa uplink task started", stack_id);

    lora_uplink_packet_t batch[LORA_UPLINK_BATCH_MAX];
    uint8_t    batch_count = 0;
    TickType_t last_flush  = xTaskGetTickCount();

    while (g_lora_task.running[stack_id]) {
        lora_uplink_packet_t uplink;

        if (xQueueReceive(g_lora_task.uplink_queue[stack_id], &uplink,
                          pdMS_TO_TICKS(LORA_UPLINK_BATCH_FLUSH_MS)) == pdTRUE) {
            if (batch_count < LORA_UPLINK_BATCH_MAX) {
                batch[batch_count++] = uplink;
            }
        }

        TickType_t now = xTaskGetTickCount();

        if (batch_count > 0 &&
            ((now - last_flush) >= pdMS_TO_TICKS(LORA_UPLINK_BATCH_FLUSH_MS) ||
             batch_count >= LORA_UPLINK_BATCH_MAX)) {

            for (uint8_t i = 0; i < batch_count; i++) {
                uint8_t packet[1 + 256];
                packet[0] = stack_id;
                memcpy(&packet[1], batch[i].payload, batch[i].payload_len);

                if (!mcu_wan_enqueue_uplink(HANDLER_LORA, packet,
                                             1 + batch[i].payload_len)) {
                    ESP_LOGW(TAG, "[Stack %d] Uplink enqueue failed", stack_id);
                }
            }

            batch_count = 0;
            last_flush  = now;
        }
    }

    ESP_LOGI(TAG, "[Stack %d] LoRa uplink task exiting", stack_id);
    free(ctx);
    vTaskDelete(NULL);
}

/**
 * @brief Downlink task – execute commands and forward responses to server.
 */
static void lora_downlink_task(void *pvParameters) {
    lora_task_context_t *ctx = (lora_task_context_t *)pvParameters;
    uint8_t stack_id = ctx->stack_id;

    ESP_LOGI(TAG, "[Stack %d] LoRa downlink task started", stack_id);

    while (g_lora_task.running[stack_id]) {
        /* Process command queue first (higher priority) */
        lora_command_request_t cmd_req;
        if (xQueueReceive(g_lora_task.command_queue[stack_id], &cmd_req, 0) == pdTRUE) {
            if (!g_lora_task.running[stack_id]) break;

            bool is_hex_cmd = cmd_req.func_config.is_hex;
            if (is_hex_cmd) {
                /* Parse hex string to actual bytes before logging to avoid
                 * printing ASCII byte-codes of the command characters. */
                uint8_t parsed_cmd[256];
                size_t  parsed_len = lora_hex_str_to_bytes(cmd_req.command,
                                                            parsed_cmd, sizeof(parsed_cmd));
                char hex_tmp[512];
                lora_bytes_to_hex_str(parsed_cmd, parsed_len, hex_tmp, sizeof(hex_tmp));
                ESP_LOGI(TAG, "[Stack %d] Processing HEX cmd: %s (%zu bytes)",
                         stack_id, hex_tmp, parsed_len);
            } else {
                ESP_LOGI(TAG, "[Stack %d] Processing AT cmd: %s", stack_id, cmd_req.command);
            }
            lora_exec_result_t result = {0};
            esp_err_t ret = lora_handler_execute_command_with_config(
                stack_id, cmd_req.command, &cmd_req.func_config, &result);

            /* is_hex_cmd already set above */
            if (ret == ESP_OK) {
                if (is_hex_cmd && result.response_len > 0) {
                    char hex_resp[512];
                    lora_bytes_to_hex_str((const uint8_t *)result.response,
                                          result.response_len, hex_resp, sizeof(hex_resp));
                    ESP_LOGI(TAG, "[Stack %d] Command OK, HEX reply: %s", stack_id, hex_resp);
                } else {
                    ESP_LOGI(TAG, "[Stack %d] Command OK: %s", stack_id, result.response);
                }
            } else {
                if (is_hex_cmd && result.response_len > 0) {
                    char hex_resp[512];
                    lora_bytes_to_hex_str((const uint8_t *)result.response,
                                          result.response_len, hex_resp, sizeof(hex_resp));
                    ESP_LOGE(TAG, "[Stack %d] Command FAIL: %s | HEX reply: %s",
                             stack_id, esp_err_to_name(ret), hex_resp);
                } else {
                    ESP_LOGE(TAG, "[Stack %d] Command FAIL: %s (status=%s)",
                             stack_id, result.response, esp_err_to_name(ret));
                }
            }

            /* Forward response to WAN MCU → PC App
             * Format: "CFLR:<stack_id>:<OK|FAIL>:<response>" */
            {
                char *resp_packet = (char *)malloc(3072);
                char *clean_resp  = (char *)malloc(2048);
                if (!resp_packet || !clean_resp) {
                    ESP_LOGE(TAG, "[Stack %d] Failed to alloc response buffers", stack_id);
                    free(resp_packet);
                    free(clean_resp);
                } else {
                    uint16_t actual_resp_len = (result.response_len > 0)
                        ? result.response_len
                        : (uint16_t)strlen(result.response);

                    int resp_len;
                    if (is_hex_cmd && actual_resp_len > 0) {
                        /* HEX mode: send binary response as hex string */
                        lora_bytes_to_hex_str((const uint8_t *)result.response,
                                              actual_resp_len, clean_resp, 2048);
                        resp_len = (ret == ESP_OK)
                            ? snprintf(resp_packet, 3072, "CFLR:%d:OK:%s", stack_id, clean_resp)
                            : snprintf(resp_packet, 3072, "CFLR:%d:FAIL:%s:%s",
                                       stack_id, esp_err_to_name(ret), clean_resp);
                    } else {
                        /* ASCII/AT mode: normalise \r\n to \x1E record-separator */
                        int ci = 0;
                        for (int i = 0; i < actual_resp_len && ci < 2047; i++) {
                            char c = result.response[i];
                            if (c == '\r') continue;
                            if (c == '\n') {
                                if (ci > 0 && clean_resp[ci - 1] != '\x1E')
                                    clean_resp[ci++] = '\x1E';
                                continue;
                            }
                            clean_resp[ci++] = c;
                        }
                        while (ci > 0 && clean_resp[ci - 1] == '\x1E') ci--;
                        clean_resp[ci] = '\0';
                        if (ret == ESP_OK) {
                            resp_len = snprintf(resp_packet, 3072,
                                                "CFLR:%d:OK:%s", stack_id, clean_resp);
                        } else {
                            if (ci > 0) {
                                resp_len = snprintf(resp_packet, 3072,
                                                    "CFLR:%d:FAIL:%s:%s",
                                                    stack_id, esp_err_to_name(ret), clean_resp);
                            } else {
                                resp_len = snprintf(resp_packet, 3072,
                                                    "CFLR:%d:FAIL:%s:NOREPLY",
                                                    stack_id, esp_err_to_name(ret));
                            }
                        }
                    }

                    if (resp_len > 0 && resp_len < 3072) {
                        if (!mcu_wan_enqueue_uplink(HANDLER_LORA,
                                                     (uint8_t *)resp_packet,
                                                     (uint16_t)resp_len)) {
                            ESP_LOGW(TAG, "[Stack %d] Failed to enqueue response to WAN",
                                     stack_id);
                        }
                    }
                    free(resp_packet);
                    free(clean_resp);
                }
            }
            continue;
        }

        /* Process downlink data packets */
        lora_downlink_packet_t downlink;
        if (xQueueReceive(g_lora_task.downlink_queue[stack_id], &downlink,
                          pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }

        if (!g_lora_task.running[stack_id]) break;

        esp_err_t ret = lora_handler_send_binary_command(
            stack_id, downlink.payload, downlink.payload_len, NULL, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[Stack %d] Failed to send downlink: %s",
                     stack_id, esp_err_to_name(ret));
        }
    }

    ESP_LOGI(TAG, "[Stack %d] LoRa downlink task exiting", stack_id);
    free(ctx);
    vTaskDelete(NULL);
}

/**
 * @brief Background listener task – receives unsolicited events from the LoRa
 *        module (e.g., +EVT:JOIN_FAILED, +EVT:RX1, downlink payload) and
 *        forwards them to the server as "CFLR:<stack>:EVT:<data>" frames.
 */
static void lora_listener_task(void *pvParameters) {
    lora_task_context_t *ctx = (lora_task_context_t *)pvParameters;
    uint8_t stack_id = ctx->stack_id;
    free(ctx);

    ESP_LOGI(TAG, "[Stack %d] LoRa listener task started", stack_id);

    /* hex_str is only needed in HEX mode but allocate regardless for simplicity */
    char *listen_buf = (char *)malloc(LORA_LISTEN_BUFFER_SIZE);
    char *clean_buf  = (char *)malloc(LORA_LISTEN_BUFFER_SIZE * 3);  /* larger for hex output */
    char *evt_packet = (char *)malloc(LORA_LISTEN_BUFFER_SIZE * 3 + 32);

    if (!listen_buf || !clean_buf || !evt_packet) {
        ESP_LOGE(TAG, "[Stack %d] Failed to alloc listener buffers", stack_id);
        free(listen_buf);
        free(clean_buf);
        free(evt_packet);
        vTaskDelete(NULL);
        return;
    }
    bool hex_mode = lora_stack_is_hex_mode(stack_id);
    while (g_lora_task.running[stack_id]) {
        memset(listen_buf, 0, LORA_LISTEN_BUFFER_SIZE);
        size_t    recv_len = 0;
        esp_err_t ret = lora_handler_listen(stack_id, listen_buf,
                                             LORA_LISTEN_BUFFER_SIZE - 1, &recv_len);

        if (ret == ESP_OK && recv_len > 0) {
            int pkt_len;
            if (hex_mode) {
                lora_bytes_to_hex_str((const uint8_t *)listen_buf, recv_len,
                                      clean_buf, LORA_LISTEN_BUFFER_SIZE * 3);
                ESP_LOGI(TAG, "[Stack %d] Listener RX %u bytes (HEX): %s",
                         stack_id, (unsigned)recv_len, clean_buf);
                pkt_len = snprintf(evt_packet, LORA_LISTEN_BUFFER_SIZE * 3 + 32,
                                   "CFLR:%d:EVT:%s", stack_id, clean_buf);
            } else {
                /* ASCII/AT mode: normalise \r\n to \x1E */
                int ci = 0;
                for (size_t i = 0; i < recv_len && ci < (int)(LORA_LISTEN_BUFFER_SIZE - 1); i++) {
                    char c = listen_buf[i];
                    if (c == '\r') continue;
                    if (c == '\n') {
                        if (ci > 0 && clean_buf[ci - 1] != '\x1E')
                            clean_buf[ci++] = '\x1E';
                        continue;
                    }
                    clean_buf[ci++] = c;
                }
                while (ci > 0 && clean_buf[ci - 1] == '\x1E') ci--;
                clean_buf[ci] = '\0';
                ESP_LOGI(TAG, "[Stack %d] Listener RX %u bytes (ASCII): %s",
                         stack_id, (unsigned)recv_len, clean_buf);
                pkt_len = (ci > 0)
                    ? snprintf(evt_packet, LORA_LISTEN_BUFFER_SIZE * 3 + 32,
                               "CFLR:%d:EVT:%s", stack_id, clean_buf)
                    : 0;
            }
            if (pkt_len > 0) {
                if (!mcu_wan_enqueue_uplink(HANDLER_LORA,
                                             (uint8_t *)evt_packet,
                                             (uint16_t)pkt_len)) {
                    ESP_LOGW(TAG, "[Stack %d] Failed to enqueue EVT to WAN", stack_id);
                } else {
                    ESP_LOGD(TAG, "[Stack %d] EVT forwarded: %s", stack_id, evt_packet);
                }
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            /* Bus busy (command in progress) or no data – yield briefly */
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            /* Unexpected error – back off */
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    ESP_LOGI(TAG, "[Stack %d] LoRa listener task exiting", stack_id);
    free(listen_buf);
    free(clean_buf);
    free(evt_packet);
    vTaskDelete(NULL);
}

/* ===== Public API ===== */

esp_err_t lora_handler_task_start(uint8_t stack_id) {
    if (!lora_is_valid_stack(stack_id)) {
        ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
        return ESP_ERR_INVALID_ARG;
    }
    if (g_lora_task.running[stack_id]) {
        ESP_LOGW(TAG, "[Stack %d] LoRa handler tasks already running", stack_id);
        return ESP_OK;
    }

    /* Create queues */
    if (!g_lora_task.uplink_queue[stack_id]) {
        g_lora_task.ul_q_storage[stack_id] = heap_caps_malloc(LORA_UPLINK_QUEUE_SIZE * sizeof(lora_uplink_packet_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        g_lora_task.ul_q_tcb[stack_id]     = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_lora_task.ul_q_storage[stack_id] || !g_lora_task.ul_q_tcb[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to alloc PSRAM for uplink queue", stack_id);
            if (g_lora_task.ul_q_storage[stack_id]) heap_caps_free(g_lora_task.ul_q_storage[stack_id]);
            if (g_lora_task.ul_q_tcb[stack_id]) heap_caps_free(g_lora_task.ul_q_tcb[stack_id]);
            return ESP_ERR_NO_MEM;
        }
        g_lora_task.uplink_queue[stack_id] = xQueueCreateStatic(LORA_UPLINK_QUEUE_SIZE,
                                                            sizeof(lora_uplink_packet_t),
                                                            g_lora_task.ul_q_storage[stack_id],
                                                            g_lora_task.ul_q_tcb[stack_id]);
    }
    if (!g_lora_task.downlink_queue[stack_id]) {
        g_lora_task.dl_q_storage[stack_id] = heap_caps_malloc(LORA_DOWNLINK_QUEUE_SIZE * sizeof(lora_downlink_packet_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        g_lora_task.dl_q_tcb[stack_id]     = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_lora_task.dl_q_storage[stack_id] || !g_lora_task.dl_q_tcb[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to alloc PSRAM for downlink queue", stack_id);
            if (g_lora_task.dl_q_storage[stack_id]) heap_caps_free(g_lora_task.dl_q_storage[stack_id]);
            if (g_lora_task.dl_q_tcb[stack_id]) heap_caps_free(g_lora_task.dl_q_tcb[stack_id]);
            return ESP_ERR_NO_MEM;
        }
        g_lora_task.downlink_queue[stack_id] = xQueueCreateStatic(LORA_DOWNLINK_QUEUE_SIZE,
                                                              sizeof(lora_downlink_packet_t),
                                                              g_lora_task.dl_q_storage[stack_id],
                                                              g_lora_task.dl_q_tcb[stack_id]);
    }
    if (!g_lora_task.command_queue[stack_id]) {
        g_lora_task.cmd_q_storage[stack_id] = heap_caps_malloc(LORA_COMMAND_QUEUE_SIZE * sizeof(lora_command_request_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        g_lora_task.cmd_q_tcb[stack_id]     = heap_caps_malloc(sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_lora_task.cmd_q_storage[stack_id] || !g_lora_task.cmd_q_tcb[stack_id]) {
            ESP_LOGE(TAG, "[Stack %d] Failed to alloc PSRAM for command queue", stack_id);
            if (g_lora_task.cmd_q_storage[stack_id]) heap_caps_free(g_lora_task.cmd_q_storage[stack_id]);
            if (g_lora_task.cmd_q_tcb[stack_id]) heap_caps_free(g_lora_task.cmd_q_tcb[stack_id]);
            return ESP_ERR_NO_MEM;
        }
        g_lora_task.command_queue[stack_id] = xQueueCreateStatic(LORA_COMMAND_QUEUE_SIZE,
                                                             sizeof(lora_command_request_t),
                                                             g_lora_task.cmd_q_storage[stack_id],
                                                             g_lora_task.cmd_q_tcb[stack_id]);
    }

    /* Init middleware once */
    static bool middleware_init = false;
    if (!middleware_init) {
        if (lora_handler_init() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize LoRa middleware");
            return ESP_FAIL;
        }
        middleware_init = true;
    }

    /* Allocate task contexts */
    lora_task_context_t *uplink_ctx   = (lora_task_context_t *)malloc(sizeof(lora_task_context_t));
    lora_task_context_t *downlink_ctx = (lora_task_context_t *)malloc(sizeof(lora_task_context_t));
    if (!uplink_ctx || !downlink_ctx) {
        ESP_LOGE(TAG, "[Stack %d] Failed to alloc task contexts", stack_id);
        free(uplink_ctx);
        free(downlink_ctx);
        return ESP_ERR_NO_MEM;
    }
    uplink_ctx->stack_id   = stack_id;
    downlink_ctx->stack_id = stack_id;

    g_lora_task.running[stack_id] = true;

    char task_name[16];

    /* ---- Uplink task (PSRAM stack) ---- */
    g_lora_task.uplink_stack[stack_id] = heap_caps_malloc(LORA_UPLINK_TASK_STACK_SIZE,
                                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_lora_task.uplink_tcb[stack_id]   = heap_caps_malloc(sizeof(StaticTask_t),
                                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!g_lora_task.uplink_stack[stack_id] || !g_lora_task.uplink_tcb[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to alloc uplink stack/TCB", stack_id);
        heap_caps_free(g_lora_task.uplink_stack[stack_id]); g_lora_task.uplink_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.uplink_tcb[stack_id]);   g_lora_task.uplink_tcb[stack_id]   = NULL;
        g_lora_task.running[stack_id] = false;
        free(uplink_ctx); free(downlink_ctx);
        return ESP_ERR_NO_MEM;
    }
    snprintf(task_name, sizeof(task_name), "lora_ul_s%d", stack_id);
    g_lora_task.uplink_task_handle[stack_id] = xTaskCreateStaticPinnedToCore(
        lora_uplink_task, task_name, LORA_UPLINK_TASK_STACK_SIZE / sizeof(StackType_t),
        uplink_ctx, LORA_UPLINK_TASK_PRIORITY,
        g_lora_task.uplink_stack[stack_id], g_lora_task.uplink_tcb[stack_id], tskNO_AFFINITY);
    if (!g_lora_task.uplink_task_handle[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to create uplink task", stack_id);
        heap_caps_free(g_lora_task.uplink_stack[stack_id]); g_lora_task.uplink_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.uplink_tcb[stack_id]);   g_lora_task.uplink_tcb[stack_id]   = NULL;
        g_lora_task.running[stack_id] = false;
        free(uplink_ctx); free(downlink_ctx);
        return ESP_FAIL;
    }

    /* ---- Downlink task (PSRAM stack) ---- */
    g_lora_task.downlink_stack[stack_id] = heap_caps_malloc(LORA_DOWNLINK_TASK_STACK_SIZE,
                                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_lora_task.downlink_tcb[stack_id]   = heap_caps_malloc(sizeof(StaticTask_t),
                                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!g_lora_task.downlink_stack[stack_id] || !g_lora_task.downlink_tcb[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to alloc downlink stack/TCB", stack_id);
        vTaskDelete(g_lora_task.uplink_task_handle[stack_id]);
        g_lora_task.uplink_task_handle[stack_id] = NULL;
        heap_caps_free(g_lora_task.uplink_stack[stack_id]);   g_lora_task.uplink_stack[stack_id]   = NULL;
        heap_caps_free(g_lora_task.uplink_tcb[stack_id]);     g_lora_task.uplink_tcb[stack_id]     = NULL;
        heap_caps_free(g_lora_task.downlink_stack[stack_id]); g_lora_task.downlink_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.downlink_tcb[stack_id]);   g_lora_task.downlink_tcb[stack_id]   = NULL;
        g_lora_task.running[stack_id] = false;
        free(downlink_ctx);
        return ESP_ERR_NO_MEM;
    }
    snprintf(task_name, sizeof(task_name), "lora_dl_s%d", stack_id);
    g_lora_task.downlink_task_handle[stack_id] = xTaskCreateStaticPinnedToCore(
        lora_downlink_task, task_name, LORA_DOWNLINK_TASK_STACK_SIZE / sizeof(StackType_t),
        downlink_ctx, LORA_DOWNLINK_TASK_PRIORITY,
        g_lora_task.downlink_stack[stack_id], g_lora_task.downlink_tcb[stack_id], tskNO_AFFINITY);
    if (!g_lora_task.downlink_task_handle[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to create downlink task", stack_id);
        vTaskDelete(g_lora_task.uplink_task_handle[stack_id]);
        g_lora_task.uplink_task_handle[stack_id] = NULL;
        heap_caps_free(g_lora_task.uplink_stack[stack_id]);   g_lora_task.uplink_stack[stack_id]   = NULL;
        heap_caps_free(g_lora_task.uplink_tcb[stack_id]);     g_lora_task.uplink_tcb[stack_id]     = NULL;
        heap_caps_free(g_lora_task.downlink_stack[stack_id]); g_lora_task.downlink_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.downlink_tcb[stack_id]);   g_lora_task.downlink_tcb[stack_id]   = NULL;
        g_lora_task.running[stack_id] = false;
        free(downlink_ctx);
        return ESP_FAIL;
    }

    /* ---- Listener task (PSRAM stack, non-fatal) ---- */
    lora_task_context_t *listener_ctx = (lora_task_context_t *)malloc(sizeof(lora_task_context_t));
    if (!listener_ctx) {
        ESP_LOGW(TAG, "[Stack %d] Failed to alloc listener ctx (non-fatal)", stack_id);
    } else {
        listener_ctx->stack_id = stack_id;
        g_lora_task.listener_stack[stack_id] = heap_caps_malloc(LORA_LISTENER_TASK_STACK_SIZE,
                                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        g_lora_task.listener_tcb[stack_id]   = heap_caps_malloc(sizeof(StaticTask_t),
                                                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_lora_task.listener_stack[stack_id] || !g_lora_task.listener_tcb[stack_id]) {
            ESP_LOGW(TAG, "[Stack %d] Failed to alloc listener stack/TCB (non-fatal)", stack_id);
            heap_caps_free(g_lora_task.listener_stack[stack_id]); g_lora_task.listener_stack[stack_id] = NULL;
            heap_caps_free(g_lora_task.listener_tcb[stack_id]);   g_lora_task.listener_tcb[stack_id]   = NULL;
            free(listener_ctx);
        } else {
            snprintf(task_name, sizeof(task_name), "lora_ls_s%d", stack_id);
            g_lora_task.listener_task_handle[stack_id] = xTaskCreateStaticPinnedToCore(
                lora_listener_task, task_name, LORA_LISTENER_TASK_STACK_SIZE / sizeof(StackType_t),
                listener_ctx, LORA_LISTENER_TASK_PRIORITY,
                g_lora_task.listener_stack[stack_id], g_lora_task.listener_tcb[stack_id], tskNO_AFFINITY);
            if (!g_lora_task.listener_task_handle[stack_id]) {
                ESP_LOGW(TAG, "[Stack %d] Failed to create listener task (non-fatal)", stack_id);
                heap_caps_free(g_lora_task.listener_stack[stack_id]); g_lora_task.listener_stack[stack_id] = NULL;
                heap_caps_free(g_lora_task.listener_tcb[stack_id]);   g_lora_task.listener_tcb[stack_id]   = NULL;
                free(listener_ctx);
            }
        }
    }

    ESP_LOGI(TAG, "[Stack %d] LoRa handler tasks started", stack_id);
    return ESP_OK;
}

esp_err_t lora_handler_task_stop(uint8_t stack_id) {
    if (!lora_is_valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;
    if (!g_lora_task.running[stack_id]) {
        ESP_LOGW(TAG, "[Stack %d] LoRa handler tasks not running", stack_id);
        return ESP_OK;
    }

    g_lora_task.running[stack_id] = false;

    if (g_lora_task.uplink_task_handle[stack_id]) {
        vTaskDelete(g_lora_task.uplink_task_handle[stack_id]);
        g_lora_task.uplink_task_handle[stack_id] = NULL;
        heap_caps_free(g_lora_task.uplink_stack[stack_id]); g_lora_task.uplink_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.uplink_tcb[stack_id]);   g_lora_task.uplink_tcb[stack_id]   = NULL;
    }
    if (g_lora_task.downlink_task_handle[stack_id]) {
        vTaskDelete(g_lora_task.downlink_task_handle[stack_id]);
        g_lora_task.downlink_task_handle[stack_id] = NULL;
        heap_caps_free(g_lora_task.downlink_stack[stack_id]); g_lora_task.downlink_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.downlink_tcb[stack_id]);   g_lora_task.downlink_tcb[stack_id]   = NULL;
    }
    if (g_lora_task.listener_task_handle[stack_id]) {
        vTaskDelete(g_lora_task.listener_task_handle[stack_id]);
        g_lora_task.listener_task_handle[stack_id] = NULL;
        heap_caps_free(g_lora_task.listener_stack[stack_id]); g_lora_task.listener_stack[stack_id] = NULL;
        heap_caps_free(g_lora_task.listener_tcb[stack_id]);   g_lora_task.listener_tcb[stack_id]   = NULL;
    }
    if (g_lora_task.uplink_queue[stack_id]) {
        g_lora_task.uplink_queue[stack_id] = NULL;
    }
    if (g_lora_task.downlink_queue[stack_id]) {
        g_lora_task.downlink_queue[stack_id] = NULL;
    }
    if (g_lora_task.command_queue[stack_id]) {
        g_lora_task.command_queue[stack_id] = NULL;
    }

    /* Free PSRAM queues and internal TCBs */
    heap_caps_free(g_lora_task.ul_q_storage[stack_id]); g_lora_task.ul_q_storage[stack_id] = NULL;
    heap_caps_free(g_lora_task.ul_q_tcb[stack_id]);     g_lora_task.ul_q_tcb[stack_id] = NULL;
    heap_caps_free(g_lora_task.dl_q_storage[stack_id]); g_lora_task.dl_q_storage[stack_id] = NULL;
    heap_caps_free(g_lora_task.dl_q_tcb[stack_id]);     g_lora_task.dl_q_tcb[stack_id] = NULL;
    heap_caps_free(g_lora_task.cmd_q_storage[stack_id]); g_lora_task.cmd_q_storage[stack_id] = NULL;
    heap_caps_free(g_lora_task.cmd_q_tcb[stack_id]);    g_lora_task.cmd_q_tcb[stack_id] = NULL;

    ESP_LOGI(TAG, "[Stack %d] LoRa handler tasks stopped", stack_id);
    return ESP_OK;
}

esp_err_t lora_handler_task_load_config(uint8_t stack_id,
                                          const char *json_config,
                                          uint16_t len) {
    if (!lora_is_valid_stack(stack_id) || !json_config || len == 0) {
        ESP_LOGE(TAG, "Invalid config parameters");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "[Stack %d] Loading JSON config (%d bytes)", stack_id, len);

    esp_err_t ret = lora_handler_load_config(stack_id, json_config, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Stack %d] Failed to load config: %s", stack_id, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "[Stack %d] Config loaded – running startup sequence", stack_id);

    /*
     * LoRa startup sequence (no enter_cmd_mode):
     *   1. Hardware reset (pulls RST low then high, waits for module boot)
     *   2. 500 ms delay to allow firmware banner output
     *   3. Get module info to confirm UART link
     */
    lora_handler_hw_reset(stack_id);
    vTaskDelay(pdMS_TO_TICKS(500));

    char info_buf[256] = {0};
    ret = lora_handler_get_info(stack_id, info_buf, sizeof(info_buf));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[Stack %d] Get info after reset failed (non-fatal): %s",
                 stack_id, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "[Stack %d] Module info: %s", stack_id, info_buf);
    }

    return ESP_OK;
}

bool lora_handler_task_enqueue_uplink(uint8_t stack_id,
                                       const uint8_t *data,
                                       uint16_t len) {
    if (!lora_is_valid_stack(stack_id) ||
        !g_lora_task.running[stack_id] ||
        !g_lora_task.uplink_queue[stack_id] ||
        !data || len == 0) {
        return false;
    }

    lora_uplink_packet_t packet = {
        .stack_id     = stack_id,
        .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS,
        .payload_len  = (len > sizeof(packet.payload)) ? sizeof(packet.payload) : len,
    };
    memcpy(packet.payload, data, packet.payload_len);

    return (xQueueSend(g_lora_task.uplink_queue[stack_id], &packet,
                       pdMS_TO_TICKS(100)) == pdTRUE);
}

bool lora_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len) {
    if (!data || len < 1) {
        ESP_LOGE(TAG, "Invalid downlink data (need >= 1 byte)");
        return false;
    }

    uint8_t stack_id = data[0];
    if (!lora_is_valid_stack(stack_id) ||
        !g_lora_task.running[stack_id] ||
        !g_lora_task.downlink_queue[stack_id]) {
        ESP_LOGE(TAG, "Stack %d not running or invalid", stack_id);
        return false;
    }

    lora_downlink_packet_t packet = {
        .stack_id    = stack_id,
        .timeout_ms  = 1000,
        .payload_len = ((uint16_t)(len - 1) > sizeof(packet.payload))
                            ? sizeof(packet.payload) : (uint16_t)(len - 1),
    };
    memcpy(packet.payload, &data[1], packet.payload_len);

    return (xQueueSend(g_lora_task.downlink_queue[stack_id], &packet,
                       pdMS_TO_TICKS(100)) == pdTRUE);
}

esp_err_t lora_handler_task_execute_command(const lora_command_request_t *request) {
    if (!request || request->stack_id >= LORA_MAX_STACKS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t stack_id = request->stack_id;
    if (!g_lora_task.running[stack_id] || !g_lora_task.command_queue[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Task not running or command queue uninitialised", stack_id);
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(g_lora_task.command_queue[stack_id], request, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "[Stack %d] Command queue full, dropping command", stack_id);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "[Stack %d] Command enqueued: %s", stack_id, request->command);
    return ESP_OK;
}
