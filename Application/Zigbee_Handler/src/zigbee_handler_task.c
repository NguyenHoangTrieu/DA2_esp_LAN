/**
 * @file zigbee_handler_task.c
 * @brief Zigbee Handler Task Implementation
 *
 * Mirrors lora_handler_task.c with Zigbee-specific differences:
 *  - Response prefix "CFZB:" forwarded to mcu_wan_enqueue_uplink(HANDLER_ZIGBEE)
 *  - All commands use ASCII AT format (unified with BLE/LoRa)
 *  - Listener forwards ASCII async events as "CFZB:<stack>:EVT:<text>"
 *  - Startup sequence: HW_RESET → 500 ms → GET_INFO
 */

#include "zigbee_handler_task.h"
#include "zigbee_handler.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "ZIGBEE_TASK";

/* ===== Configuration ===== */
#define ZIGBEE_UPLINK_STACK_SIZE    (24  * 1024)
#define ZIGBEE_DOWNLINK_STACK_SIZE  (32 * 1024)
#define ZIGBEE_LISTENER_STACK_SIZE  (16  * 1024)
#define ZIGBEE_UPLINK_PRIO          5
#define ZIGBEE_DOWNLINK_PRIO        6
#define ZIGBEE_LISTENER_PRIO        4
#define ZIGBEE_UPLINK_QUEUE_SZ      20
#define ZIGBEE_COMMAND_QUEUE_SZ     10
#define ZIGBEE_MAX_STACKS           2
#define ZIGBEE_UPLINK_BATCH_MAX     8
#define ZIGBEE_BATCH_FLUSH_MS       50
#define ZIGBEE_LISTEN_BUFFER_SIZE   512
#define ZIGBEE_RESP_PACKET_SIZE     2048

/* ===== Static Data ===== */

static struct {
    bool         running[ZIGBEE_MAX_STACKS];
    TaskHandle_t uplink_handle[ZIGBEE_MAX_STACKS];
    TaskHandle_t downlink_handle[ZIGBEE_MAX_STACKS];
    TaskHandle_t listener_handle[ZIGBEE_MAX_STACKS];
    QueueHandle_t uplink_queue[ZIGBEE_MAX_STACKS];
    QueueHandle_t command_queue[ZIGBEE_MAX_STACKS];
    /* PSRAM-backed stacks (avoids DRAM exhaustion) + internal-SRAM TCBs */
    StackType_t  *ul_stack[ZIGBEE_MAX_STACKS];
    StaticTask_t *ul_tcb[ZIGBEE_MAX_STACKS];
    StackType_t  *dl_stack[ZIGBEE_MAX_STACKS];
    StaticTask_t *dl_tcb[ZIGBEE_MAX_STACKS];
    StackType_t  *ls_stack[ZIGBEE_MAX_STACKS];
    StaticTask_t *ls_tcb[ZIGBEE_MAX_STACKS];
} g_zb_task = {0};

typedef struct { uint8_t stack_id; } zb_task_ctx_t;

/* ===== Internal Helpers ===== */

static inline bool valid_stack(uint8_t sid) { return (sid < ZIGBEE_MAX_STACKS); }

/**
 * @brief Convert binary buffer to space-separated hex string.
 *        e.g. {0x55,0x80,0x03} → "55 80 03"
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

/* ===== Task Implementations ===== */

static void zigbee_uplink_task(void *pv) {
    zb_task_ctx_t *ctx = (zb_task_ctx_t *)pv;
    uint8_t sid = ctx->stack_id;
    ESP_LOGI(TAG, "[Stack %d] Uplink task started", sid);

    zigbee_uplink_packet_t batch[ZIGBEE_UPLINK_BATCH_MAX];
    uint8_t    batch_cnt = 0;
    TickType_t last_flush = xTaskGetTickCount();

    while (g_zb_task.running[sid]) {
        zigbee_uplink_packet_t pkt;
        if (xQueueReceive(g_zb_task.uplink_queue[sid], &pkt,
                          pdMS_TO_TICKS(ZIGBEE_BATCH_FLUSH_MS)) == pdTRUE) {
            if (batch_cnt < ZIGBEE_UPLINK_BATCH_MAX) {
                batch[batch_cnt++] = pkt;
            }
        }
        TickType_t now = xTaskGetTickCount();
        if (batch_cnt > 0 &&
            ((now - last_flush) >= pdMS_TO_TICKS(ZIGBEE_BATCH_FLUSH_MS) ||
             batch_cnt >= ZIGBEE_UPLINK_BATCH_MAX)) {
            for (uint8_t i = 0; i < batch_cnt; i++) {
                uint8_t packet[1 + 256];
                packet[0] = sid;
                memcpy(&packet[1], batch[i].payload, batch[i].payload_len);
                if (!mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, packet,
                                             1 + batch[i].payload_len)) {
                    ESP_LOGW(TAG, "[Stack %d] Uplink enqueue failed", sid);
                }
            }
            batch_cnt  = 0;
            last_flush = now;
        }
    }

    ESP_LOGI(TAG, "[Stack %d] Uplink task exiting", sid);
    free(ctx);
    vTaskDelete(NULL);
}

static void zigbee_downlink_task(void *pv) {
    zb_task_ctx_t *ctx = (zb_task_ctx_t *)pv;
    uint8_t sid = ctx->stack_id;
    ESP_LOGI(TAG, "[Stack %d] Downlink task started", sid);

    while (g_zb_task.running[sid]) {
        zigbee_command_request_t req;
        if (xQueueReceive(g_zb_task.command_queue[sid], &req, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }
        if (!g_zb_task.running[sid]) break;

        ESP_LOGI(TAG, "[Stack %d] Processing command: %.*s",
                 sid, req.command_len, req.command);

        zigbee_exec_result_t result = {0};
        esp_err_t ret = zigbee_handler_execute_command_raw(
            sid, req.command, req.command_len, &req.func_config, &result);

        /* Build response packet "CFZB:<stack>:OK/FAIL:<cmd>[:<hex_response>]" */
        char *resp_pkt = (char *)malloc(ZIGBEE_RESP_PACKET_SIZE);
        if (!resp_pkt) {
            ESP_LOGE(TAG, "[Stack %d] Failed alloc resp buffer", sid);
            continue;
        }

        int pkt_len;
        if (ret == ESP_OK) {
            if (result.response_len > 0) {
                /* Zigbee responses are ASCII — forward as-is */
                result.response[result.response_len] = '\0';
                ESP_LOGI(TAG, "[Stack %d] Command OK: %s", sid, (char *)result.response);
                pkt_len = snprintf(resp_pkt, ZIGBEE_RESP_PACKET_SIZE,
                                   "CFZB:%d:OK:%.*s:%s",
                                   sid, req.command_len, req.command,
                                   (char *)result.response);
            } else {
                ESP_LOGI(TAG, "[Stack %d] Command OK", sid);
                pkt_len = snprintf(resp_pkt, ZIGBEE_RESP_PACKET_SIZE,
                                   "CFZB:%d:OK:%.*s",
                                   sid, req.command_len, req.command);
            }
        } else {
            if (result.response_len > 0) {
                result.response[result.response_len] = '\0';
                ESP_LOGW(TAG, "[Stack %d] Command FAIL: %s | Module replied: %.*s",
                         sid, esp_err_to_name(ret),
                         (int)result.response_len, (char *)result.response);
                pkt_len = snprintf(resp_pkt, ZIGBEE_RESP_PACKET_SIZE,
                                   "CFZB:%d:FAIL:%.*s:%s:%s",
                                   sid, req.command_len, req.command,
                                   esp_err_to_name(ret),
                                   (char *)result.response);
            } else {
                ESP_LOGW(TAG, "[Stack %d] Command FAIL: %s | No response from module",
                         sid, esp_err_to_name(ret));
                pkt_len = snprintf(resp_pkt, ZIGBEE_RESP_PACKET_SIZE,
                                   "CFZB:%d:FAIL:%.*s:%s:NOREPLY",
                                   sid, req.command_len, req.command,
                                   esp_err_to_name(ret));
            }
        }

        if (pkt_len > 0 && pkt_len < ZIGBEE_RESP_PACKET_SIZE) {
            if (!mcu_wan_enqueue_uplink(HANDLER_ZIGBEE,
                                         (uint8_t *)resp_pkt,
                                         (uint16_t)pkt_len)) {
                ESP_LOGW(TAG, "[Stack %d] Failed to enqueue response", sid);
            }
        }
        free(resp_pkt);
    }  /* end while (g_zb_task.running[sid]) */

    ESP_LOGI(TAG, "[Stack %d] Downlink task exiting", sid);
    free(ctx);
    vTaskDelete(NULL);
}

/**
 * @brief Background listener: receives unsolicited ASCII async events
 *        (e.g. +JOIN:, +LEFT:, +ATTRREPORT:) and forwards as "CFZB:<stack>:EVT:<text>".
 */
static void zigbee_listener_task(void *pv) {
    zb_task_ctx_t *ctx = (zb_task_ctx_t *)pv;
    uint8_t sid = ctx->stack_id;
    free(ctx);
    ESP_LOGI(TAG, "[Stack %d] Listener task started", sid);

    uint8_t *listen_buf = (uint8_t *)malloc(ZIGBEE_LISTEN_BUFFER_SIZE);
    char    *hex_str    = (char    *)malloc(ZIGBEE_LISTEN_BUFFER_SIZE * 3 + 8);
    char    *evt_pkt    = (char    *)malloc(ZIGBEE_LISTEN_BUFFER_SIZE * 3 + 32);

    if (!listen_buf || !hex_str || !evt_pkt) {
        ESP_LOGE(TAG, "[Stack %d] Listener alloc failed", sid);
        free(listen_buf); free(hex_str); free(evt_pkt);
        vTaskDelete(NULL);
        return;
    }

    while (g_zb_task.running[sid]) {
        size_t    recv_len = 0;
        esp_err_t ret = zigbee_handler_listen(sid, listen_buf,
                                              ZIGBEE_LISTEN_BUFFER_SIZE,
                                              &recv_len);

        if (ret == ESP_OK && recv_len > 0) {
            /* Format as space-separated hex string */
            bytes_to_hex_str(listen_buf, recv_len, hex_str,
                             ZIGBEE_LISTEN_BUFFER_SIZE * 3 + 8);

            int pkt_len = snprintf(evt_pkt,
                                   ZIGBEE_LISTEN_BUFFER_SIZE * 3 + 32,
                                   "CFZB:%d:EVT:%s", sid, hex_str);
            if (pkt_len > 0) {
                if (!mcu_wan_enqueue_uplink(HANDLER_ZIGBEE,
                                             (uint8_t *)evt_pkt,
                                             (uint16_t)pkt_len)) {
                    ESP_LOGW(TAG, "[Stack %d] EVT enqueue failed", sid);
                }
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    ESP_LOGI(TAG, "[Stack %d] Listener task exiting", sid);
    free(listen_buf); free(hex_str); free(evt_pkt);
    vTaskDelete(NULL);
}

/* ===== Public API ===== */

bool zigbee_handler_is_running(uint8_t stack_id) {
    if (!valid_stack(stack_id)) return false;
    return g_zb_task.running[stack_id];
}

esp_err_t zigbee_handler_task_start(uint8_t stack_id) {
    if (!valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;
    if (g_zb_task.running[stack_id]) {
        ESP_LOGW(TAG, "[Stack %d] Already running", stack_id);
        return ESP_OK;
    }

    if (!g_zb_task.uplink_queue[stack_id]) {
        g_zb_task.uplink_queue[stack_id] = xQueueCreate(ZIGBEE_UPLINK_QUEUE_SZ,
                                                          sizeof(zigbee_uplink_packet_t));
        if (!g_zb_task.uplink_queue[stack_id]) return ESP_ERR_NO_MEM;
    }
    if (!g_zb_task.command_queue[stack_id]) {
        g_zb_task.command_queue[stack_id] = xQueueCreate(ZIGBEE_COMMAND_QUEUE_SZ,
                                                           sizeof(zigbee_command_request_t));
        if (!g_zb_task.command_queue[stack_id]) return ESP_ERR_NO_MEM;
    }

    static bool mw_init = false;
    if (!mw_init) {
        if (zigbee_handler_init() != ESP_OK) {
            ESP_LOGE(TAG, "Middleware init failed");
            return ESP_FAIL;
        }
        mw_init = true;
    }

    /* Allocate PSRAM stacks (avoid DRAM exhaustion) + internal-SRAM TCBs */
    g_zb_task.ul_stack[stack_id] = heap_caps_malloc(ZIGBEE_UPLINK_STACK_SIZE,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_zb_task.ul_tcb[stack_id]   = heap_caps_malloc(sizeof(StaticTask_t),
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_zb_task.dl_stack[stack_id] = heap_caps_malloc(ZIGBEE_DOWNLINK_STACK_SIZE,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_zb_task.dl_tcb[stack_id]   = heap_caps_malloc(sizeof(StaticTask_t),
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_zb_task.ls_stack[stack_id] = heap_caps_malloc(ZIGBEE_LISTENER_STACK_SIZE,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_zb_task.ls_tcb[stack_id]   = heap_caps_malloc(sizeof(StaticTask_t),
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!g_zb_task.ul_stack[stack_id] || !g_zb_task.ul_tcb[stack_id] ||
        !g_zb_task.dl_stack[stack_id] || !g_zb_task.dl_tcb[stack_id] ||
        !g_zb_task.ls_stack[stack_id] || !g_zb_task.ls_tcb[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to alloc PSRAM task stacks/TCBs", stack_id);
        heap_caps_free(g_zb_task.ul_stack[stack_id]); g_zb_task.ul_stack[stack_id] = NULL;
        heap_caps_free(g_zb_task.ul_tcb[stack_id]);   g_zb_task.ul_tcb[stack_id]   = NULL;
        heap_caps_free(g_zb_task.dl_stack[stack_id]); g_zb_task.dl_stack[stack_id] = NULL;
        heap_caps_free(g_zb_task.dl_tcb[stack_id]);   g_zb_task.dl_tcb[stack_id]   = NULL;
        heap_caps_free(g_zb_task.ls_stack[stack_id]); g_zb_task.ls_stack[stack_id] = NULL;
        heap_caps_free(g_zb_task.ls_tcb[stack_id]);   g_zb_task.ls_tcb[stack_id]   = NULL;
        return ESP_ERR_NO_MEM;
    }

    zb_task_ctx_t *ul_ctx = malloc(sizeof(zb_task_ctx_t));
    zb_task_ctx_t *dl_ctx = malloc(sizeof(zb_task_ctx_t));
    if (!ul_ctx || !dl_ctx) { free(ul_ctx); free(dl_ctx); return ESP_ERR_NO_MEM; }
    ul_ctx->stack_id = dl_ctx->stack_id = stack_id;

    g_zb_task.running[stack_id] = true;

    char name[16];
    snprintf(name, sizeof(name), "zb_ul_s%d", stack_id);
    g_zb_task.uplink_handle[stack_id] = xTaskCreateStaticPinnedToCore(
        zigbee_uplink_task, name,
        ZIGBEE_UPLINK_STACK_SIZE / sizeof(StackType_t),
        ul_ctx, ZIGBEE_UPLINK_PRIO,
        g_zb_task.ul_stack[stack_id], g_zb_task.ul_tcb[stack_id], tskNO_AFFINITY);
    if (!g_zb_task.uplink_handle[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to create uplink task", stack_id);
        g_zb_task.running[stack_id] = false;
        free(ul_ctx); free(dl_ctx);
        return ESP_FAIL;
    }
    snprintf(name, sizeof(name), "zb_dl_s%d", stack_id);
    g_zb_task.downlink_handle[stack_id] = xTaskCreateStaticPinnedToCore(
        zigbee_downlink_task, name,
        ZIGBEE_DOWNLINK_STACK_SIZE / sizeof(StackType_t),
        dl_ctx, ZIGBEE_DOWNLINK_PRIO,
        g_zb_task.dl_stack[stack_id], g_zb_task.dl_tcb[stack_id], tskNO_AFFINITY);
    if (!g_zb_task.downlink_handle[stack_id]) {
        ESP_LOGE(TAG, "[Stack %d] Failed to create downlink task", stack_id);
        vTaskDelete(g_zb_task.uplink_handle[stack_id]);
        g_zb_task.uplink_handle[stack_id] = NULL;
        g_zb_task.running[stack_id] = false;
        free(dl_ctx);
        return ESP_FAIL;
    }

    zb_task_ctx_t *ls_ctx = malloc(sizeof(zb_task_ctx_t));
    if (ls_ctx) {
        ls_ctx->stack_id = stack_id;
        snprintf(name, sizeof(name), "zb_ls_s%d", stack_id);
        g_zb_task.listener_handle[stack_id] = xTaskCreateStaticPinnedToCore(
            zigbee_listener_task, name,
            ZIGBEE_LISTENER_STACK_SIZE / sizeof(StackType_t),
            ls_ctx, ZIGBEE_LISTENER_PRIO,
            g_zb_task.ls_stack[stack_id], g_zb_task.ls_tcb[stack_id], tskNO_AFFINITY);
        if (!g_zb_task.listener_handle[stack_id]) {
            ESP_LOGW(TAG, "[Stack %d] Listener task creation failed (non-fatal)", stack_id);
            free(ls_ctx);
        }
    }

    ESP_LOGI(TAG, "[Stack %d] Zigbee tasks started", stack_id);
    return ESP_OK;
}

esp_err_t zigbee_handler_task_stop(uint8_t stack_id) {
    if (!valid_stack(stack_id)) return ESP_ERR_INVALID_ARG;
    if (!g_zb_task.running[stack_id]) return ESP_OK;

    g_zb_task.running[stack_id] = false;
    vTaskDelay(pdMS_TO_TICKS(200));

    if (g_zb_task.uplink_handle[stack_id]) {
        vTaskDelete(g_zb_task.uplink_handle[stack_id]);
        g_zb_task.uplink_handle[stack_id] = NULL;
    }
    if (g_zb_task.downlink_handle[stack_id]) {
        vTaskDelete(g_zb_task.downlink_handle[stack_id]);
        g_zb_task.downlink_handle[stack_id] = NULL;
    }
    if (g_zb_task.listener_handle[stack_id]) {
        vTaskDelete(g_zb_task.listener_handle[stack_id]);
        g_zb_task.listener_handle[stack_id] = NULL;
    }

    /* Free PSRAM stacks and internal TCBs */
    heap_caps_free(g_zb_task.ul_stack[stack_id]); g_zb_task.ul_stack[stack_id] = NULL;
    heap_caps_free(g_zb_task.ul_tcb[stack_id]);   g_zb_task.ul_tcb[stack_id]   = NULL;
    heap_caps_free(g_zb_task.dl_stack[stack_id]); g_zb_task.dl_stack[stack_id] = NULL;
    heap_caps_free(g_zb_task.dl_tcb[stack_id]);   g_zb_task.dl_tcb[stack_id]   = NULL;
    heap_caps_free(g_zb_task.ls_stack[stack_id]); g_zb_task.ls_stack[stack_id] = NULL;
    heap_caps_free(g_zb_task.ls_tcb[stack_id]);   g_zb_task.ls_tcb[stack_id]   = NULL;

    ESP_LOGI(TAG, "[Stack %d] Zigbee tasks stopped", stack_id);
    return ESP_OK;
}

esp_err_t zigbee_handler_task_load_config(uint8_t stack_id,
                                           const char *json_config,
                                           uint16_t len) {
    if (!valid_stack(stack_id) || !json_config || len == 0) return ESP_ERR_INVALID_ARG;

    /* Load JSON into middleware (also initialises UART for this stack) */
    esp_err_t ret = zigbee_handler_load_config(stack_id, json_config, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[Stack %d] Config load failed: %s", stack_id, esp_err_to_name(ret));
        return ret;
    }
    zigbee_exec_result_t res = {0};

    /* HW Reset (GPIO NRST) */
    ret = zigbee_handler_execute_command_with_config(
        stack_id, ZIGBEE_FUNC_HW_RESET, NULL, 0, &res);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[Stack %d] HW reset failed (continuing): %s",
                 stack_id, esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // /* Ensure module is in AT command mode.
    //  * E180-ZG120B boots in HEX binary mode by default — this sends the
    //  * HEX frame [55 03 00 16 16] to switch to AT mode, with fallback phases.
    //  * Must be called after EVERY hardware/software reset. */
    // if (zigbee_handler_ensure_at_mode(stack_id) != ESP_OK) {
    //     ESP_LOGW(TAG, "[Stack %d] AT mode ensure failed — subsequent AT commands may return INVALID",
    //              stack_id);
    // }
    zigbee_handler_test_at_reset(stack_id);  // Best-effort AT reset (some modules may not support)
    /* Get module info */
    ret = zigbee_handler_execute_command_with_config(
        stack_id, ZIGBEE_FUNC_GET_INFO, NULL, 0, &res);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[Stack %d] GET_INFO failed: %s", stack_id, esp_err_to_name(ret));
    } else {
        char info_buf[256] = {0};
        size_t copy_len = res.response_len < sizeof(info_buf) - 1 ? res.response_len : sizeof(info_buf) - 1;
        memcpy(info_buf, res.response, copy_len);
        info_buf[sizeof(info_buf) - 1] = '\0';
        ESP_LOGI(TAG, "[Stack %d] Module info: %s", stack_id, info_buf);
    }

    ESP_LOGI(TAG, "[Stack %d] Config load and init sequence complete", stack_id);
    return ESP_OK;
}

esp_err_t zigbee_handler_task_execute_command(const zigbee_command_request_t *req) {
    if (!req || !valid_stack(req->stack_id)) return ESP_ERR_INVALID_ARG;
    if (!g_zb_task.command_queue[req->stack_id]) return ESP_ERR_INVALID_STATE;

    if (xQueueSend(g_zb_task.command_queue[req->stack_id], req,
                   pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "[Stack %d] Command queue full", req->stack_id);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool zigbee_handler_task_enqueue_uplink(uint8_t stack_id,
                                         const uint8_t *data, uint16_t len) {
    if (!valid_stack(stack_id) || !data || len == 0) return false;
    if (!g_zb_task.uplink_queue[stack_id]) return false;

    zigbee_uplink_packet_t pkt = {0};
    pkt.stack_id    = stack_id;
    pkt.timestamp_ms = 0;
    uint16_t copy    = (len <= 256) ? len : 256;
    memcpy(pkt.payload, data, copy);
    pkt.payload_len  = copy;

    return (xQueueSend(g_zb_task.uplink_queue[stack_id], &pkt, 0) == pdTRUE);
}
