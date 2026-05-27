/**
 * @file module_usb_comm.c
 * @brief Generic USB CDC-ACM Communication Driver — HOST mode.
 *
 * The LAN MCU is the USB HOST. Every module plugged into the USB lane
 * (bench rig, SIM7600, Zigbee coordinator, ...) is a USB DEVICE that
 * exposes a CDC-ACM interface. This driver:
 *   - installs the USB Host Library + CDC-ACM host class driver,
 *   - waits (in a background task) for any CDC-ACM device to enumerate,
 *   - opens it, sets line coding + asserts DTR/RTS,
 *   - funnels the device's bulk-IN data into an RX stream buffer that
 *     module_usb_comm_receive() drains,
 *   - sends via the device's bulk-OUT endpoint (blocking).
 *
 * HARDWARE NOTE: ESP32-S3 has a single USB-OTG controller, so there is
 * exactly ONE physical USB host port — it is NOT per-stack like UART/SPI/
 * I2C. Only one USB host handle may be active at a time; a second init
 * is rejected with ESP_ERR_INVALID_STATE. The OTG D-/D+ pins are fixed
 * (GPIO19/GPIO20) and the port must be able to source VBUS to the device.
 *
 * The USB Serial/JTAG peripheral used by the old implementation is a
 * fixed USB-DEVICE console block and cannot host anything — it was the
 * wrong peripheral for this lane.
 */

#include "module_usb_comm.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_USB";

/* RX stream buffer lives in SPIRAM to avoid competing with internal heap.
 * 4096 bytes holds ~16 × 256 B packets — well above the 50 ms read window. */
#define USB_RX_STREAM_BYTES 4096
/* xStreamBufferCreateStatic needs storage_size + 1 bytes. */
#define USB_RX_STORAGE_BYTES (USB_RX_STREAM_BYTES + 1)
/* USB task stack size (placed in SPIRAM). */
#define USB_TASK_STACK_WORDS (4096 / sizeof(StackType_t))
/* Lower bound for the CDC in/out internal transfer buffers. */
#define USB_MIN_XFER_BYTES 512
/* How long cdc_acm_host_open() waits for a device before retrying. */
#define USB_OPEN_TIMEOUT_MS 1000

/* ===== Internal Handle Structure ===== */

struct module_usb_comm_s {
  uint8_t stack_id;
  usb_cdc_line_coding_t line_coding;
  size_t rx_buffer_size;
  size_t tx_buffer_size;

  StreamBufferHandle_t rx_stream; /* CDC RX data -> consumer            */
  SemaphoreHandle_t mutex;        /* serialises TX + dev-pointer access */
  cdc_acm_dev_hdl_t cdc_dev;      /* open device, NULL when disconnected */
  TaskHandle_t connect_task;      /* background open/reopen loop         */
  volatile bool connected;
  volatile bool running;
  bool initialized;

  /* RX-overflow telemetry. */
  portMUX_TYPE ovf_mux;
  uint32_t rx_overflow_evt;

  /* SPIRAM-backed allocations — kept here for cleanup in deinit.
   * Using static creation so storage lives in SPIRAM, not internal heap.
   * Saves ~16 KB internal: stream buffer (4 KB) + 2 task stacks (4 KB ea). */
  uint8_t *rx_stream_storage;      /* USB_RX_STORAGE_BYTES from SPIRAM   */
  StaticStreamBuffer_t rx_stream_static; /* descriptor, small, in handle  */
  StackType_t *host_lib_stack;     /* USB_TASK_STACK_WORDS from SPIRAM   */
  StaticTask_t *host_lib_tcb;      /* small, internal heap via calloc    */
  StackType_t *connect_stack;      /* USB_TASK_STACK_WORDS from SPIRAM   */
  StaticTask_t *connect_tcb;       /* small, internal heap via calloc    */
};

/* ===== Global host state (single OTG controller) ===== */

static SemaphoreHandle_t s_host_lock = NULL; /* guards install/active    */
static module_usb_comm_handle_t s_active = NULL; /* the one live handle  */
static TaskHandle_t s_host_lib_task = NULL;
static volatile bool s_host_lib_running = false;

/* ===== Host library event pump ===== */

static void usb_host_lib_task(void *arg) {
  (void)arg;
  while (s_host_lib_running) {
    uint32_t flags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
    if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
    }
  }
  vTaskDelete(NULL);
}

/* ===== CDC-ACM callbacks (run in the CDC driver task context) ===== */

static bool usb_cdc_rx_cb(const uint8_t *data, size_t data_len,
                          void *user_arg) {
  module_usb_comm_handle_t h = (module_usb_comm_handle_t)user_arg;
  if (h && h->rx_stream && data_len) {
    /* Non-blocking: if the consumer is too slow the surplus is dropped
     * here, which is the correct "RX overflow" signal for the lane. */
    size_t sent = xStreamBufferSend(h->rx_stream, data, data_len, 0);
    if (sent < data_len) {
      portENTER_CRITICAL_SAFE(&h->ovf_mux);
      h->rx_overflow_evt++;
      portEXIT_CRITICAL_SAFE(&h->ovf_mux);
    }
  }
  return true; /* data consumed; driver may recycle the buffer */
}

static void usb_cdc_event_cb(const cdc_acm_host_dev_event_data_t *event,
                             void *user_ctx) {
  module_usb_comm_handle_t h = (module_usb_comm_handle_t)user_ctx;
  switch (event->type) {
  case CDC_ACM_HOST_DEVICE_DISCONNECTED:
    ESP_LOGW(TAG, "USB Stack%d device disconnected",
             h ? h->stack_id : 0xFF);
    xSemaphoreTake(h->mutex, portMAX_DELAY);
    h->connected = false;
    if (h->cdc_dev) {
      cdc_acm_host_close(h->cdc_dev);
      h->cdc_dev = NULL;
    }
    xSemaphoreGive(h->mutex);
    break;
  case CDC_ACM_HOST_ERROR:
    ESP_LOGE(TAG, "USB Stack%d CDC error %d", h ? h->stack_id : 0xFF,
             event->data.error);
    break;
  default:
    break;
  }
}

/* ===== Background connect / reconnect loop ===== */

static void usb_connect_task(void *arg) {
  module_usb_comm_handle_t h = (module_usb_comm_handle_t)arg;

  const cdc_acm_host_device_config_t dev_cfg = {
      .connection_timeout_ms = USB_OPEN_TIMEOUT_MS,
      .out_buffer_size = h->tx_buffer_size,
      .in_buffer_size = h->rx_buffer_size,
      .user_arg = h,
      .event_cb = usb_cdc_event_cb,
      .data_cb = usb_cdc_rx_cb,
  };

  while (h->running) {
    if (h->connected) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    cdc_acm_dev_hdl_t dev = NULL;
    /* CDC_HOST_ANY_VID/PID -> bind to the first CDC-ACM device that
     * enumerates, interface 0. Blocks up to connection_timeout_ms. */
    esp_err_t ret = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0,
                                      &dev_cfg, &dev);
    if (ret != ESP_OK) {
      /* No device yet (ESP_ERR_NOT_FOUND) — keep polling. */
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    cdc_acm_line_coding_t lc = {
        .dwDTERate = h->line_coding.bit_rate ? h->line_coding.bit_rate
                                             : 115200,
        .bCharFormat = h->line_coding.stop_bits, /* 0=1,1=1.5,2=2 stop */
        .bParityType = h->line_coding.parity,    /* 0=N,1=O,2=E        */
        .bDataBits = h->line_coding.data_bits ? h->line_coding.data_bits
                                              : 8,
    };
    /* Best-effort: a native-USB CDC device ignores line coding, a real
     * UART bridge (e.g. modem) honours it. Don't fail the link on it. */
    (void)cdc_acm_host_line_coding_set(dev, &lc);
    /* DTR/RTS asserted so the device starts streaming. */
    (void)cdc_acm_host_set_control_line_state(dev, true, true);

    xSemaphoreTake(h->mutex, portMAX_DELAY);
    h->cdc_dev = dev;
    h->connected = true;
    xSemaphoreGive(h->mutex);

    ESP_LOGI(TAG, "USB Stack%d device opened (CDC-ACM, %lu bps)",
             h->stack_id, (unsigned long)lc.dwDTERate);
  }

  vTaskDelete(NULL);
}

/* ===== Public API Implementation ===== */

esp_err_t module_usb_comm_init(const module_usb_config_t *config,
                               module_usb_comm_handle_t *handle) {
  if (!config || !handle) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }
  if (config->stack_id > 1) {
    ESP_LOGE(TAG, "Invalid stack_id: %d (must be 0 or 1)", config->stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  /* One-time creation of the global host lock. */
  if (!s_host_lock) {
    s_host_lock = xSemaphoreCreateMutex();
    if (!s_host_lock)
      return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = ESP_OK;
  xSemaphoreTake(s_host_lock, portMAX_DELAY);

  /* Single OTG controller -> only one USB host may be live. */
  if (s_active) {
    ESP_LOGE(TAG,
             "USB host already active (Stack%d); ESP32-S3 has one OTG port",
             s_active->stack_id);
    xSemaphoreGive(s_host_lock);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Initializing USB host (CDC-ACM) for Stack%d",
           config->stack_id);

  module_usb_comm_handle_t h =
      (module_usb_comm_handle_t)calloc(1, sizeof(struct module_usb_comm_s));
  if (!h) {
    ret = ESP_ERR_NO_MEM;
    goto fail_unlock;
  }

  h->stack_id = config->stack_id;
  h->line_coding = config->line_coding;
  h->ovf_mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  h->rx_buffer_size = config->rx_buffer_size < USB_MIN_XFER_BYTES
                          ? USB_MIN_XFER_BYTES
                          : config->rx_buffer_size;
  h->tx_buffer_size = config->tx_buffer_size < USB_MIN_XFER_BYTES
                          ? USB_MIN_XFER_BYTES
                          : config->tx_buffer_size;

  h->mutex = xSemaphoreCreateMutex();

  /* Stream buffer storage in SPIRAM — keeps internal heap free for tasks
   * that require MALLOC_CAP_INTERNAL (e.g. module_monitor_task). */
  h->rx_stream_storage = (uint8_t *)heap_caps_malloc(
      USB_RX_STORAGE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!h->mutex || !h->rx_stream_storage) {
    ret = ESP_ERR_NO_MEM;
    goto fail_free;
  }
  h->rx_stream = xStreamBufferCreateStatic(USB_RX_STREAM_BYTES, 1,
                                           h->rx_stream_storage,
                                           &h->rx_stream_static);

  /* Install USB Host Library + event pump. */
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  ret = usb_host_install(&host_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(ret));
    goto fail_free;
  }
  /* Host-lib event pump — stack in SPIRAM, TCB in internal heap (TCB must
   * be accessible to the scheduler which may run DMA/ISR context). */
  h->host_lib_stack = (StackType_t *)heap_caps_malloc(
      USB_TASK_STACK_WORDS * sizeof(StackType_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  h->host_lib_tcb = (StaticTask_t *)heap_caps_malloc(
      sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!h->host_lib_stack || !h->host_lib_tcb) {
    if (h->host_lib_stack) heap_caps_free(h->host_lib_stack);
    if (h->host_lib_tcb)   heap_caps_free(h->host_lib_tcb);
    h->host_lib_stack = NULL; h->host_lib_tcb = NULL;
    usb_host_uninstall();
    ret = ESP_ERR_NO_MEM;
    goto fail_free;
  }
  s_host_lib_running = true;
  s_host_lib_task = xTaskCreateStatic(
      usb_host_lib_task, "usb_host_lib",
      USB_TASK_STACK_WORDS, NULL, 5,
      h->host_lib_stack, h->host_lib_tcb);
  if (!s_host_lib_task) {
    s_host_lib_running = false;
    usb_host_uninstall();
    ret = ESP_FAIL;
    goto fail_free;
  }

  /* Install CDC-ACM host class driver. */
  const cdc_acm_host_driver_config_t drv_cfg = {
      .driver_task_stack_size = 4096,
      .driver_task_priority = 6,
      .xCoreID = tskNO_AFFINITY,
      .new_dev_cb = NULL,
  };
  ret = cdc_acm_host_install(&drv_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "cdc_acm_host_install failed: %s", esp_err_to_name(ret));
    goto fail_host_lib;
  }

  /* Connect/reconnect loop — stack in SPIRAM. */
  h->connect_stack = (StackType_t *)heap_caps_malloc(
      USB_TASK_STACK_WORDS * sizeof(StackType_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  h->connect_tcb = (StaticTask_t *)heap_caps_malloc(
      sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!h->connect_stack || !h->connect_tcb) {
    if (h->connect_stack) heap_caps_free(h->connect_stack);
    if (h->connect_tcb)   heap_caps_free(h->connect_tcb);
    h->connect_stack = NULL; h->connect_tcb = NULL;
    cdc_acm_host_uninstall();
    ret = ESP_ERR_NO_MEM;
    goto fail_host_lib;
  }
  h->running = true;
  h->connect_task = xTaskCreateStatic(
      usb_connect_task, "usb_connect",
      USB_TASK_STACK_WORDS, h, 5,
      h->connect_stack, h->connect_tcb);
  if (!h->connect_task) {
    h->running = false;
    heap_caps_free(h->connect_stack); h->connect_stack = NULL;
    heap_caps_free(h->connect_tcb);   h->connect_tcb   = NULL;
    cdc_acm_host_uninstall();
    ret = ESP_FAIL;
    goto fail_host_lib;
  }

  h->initialized = true;
  s_active = h;
  *handle = h;
  xSemaphoreGive(s_host_lock);

  ESP_LOGI(TAG,
           "USB host ready for Stack%d (RX_buf=%d, TX_buf=%d, bitrate=%lu) "
           "— waiting for device",
           config->stack_id, (int)h->rx_buffer_size, (int)h->tx_buffer_size,
           (unsigned long)config->line_coding.bit_rate);
  return ESP_OK;

fail_host_lib:
  s_host_lib_running = false;
  usb_host_uninstall();
fail_free:
  if (h->rx_stream)
    vStreamBufferDelete(h->rx_stream);
  if (h->rx_stream_storage)
    heap_caps_free(h->rx_stream_storage);
  if (h->host_lib_stack)
    heap_caps_free(h->host_lib_stack);
  if (h->host_lib_tcb)
    heap_caps_free(h->host_lib_tcb);
  if (h->mutex)
    vSemaphoreDelete(h->mutex);
  free(h);
fail_unlock:
  xSemaphoreGive(s_host_lock);
  return ret;
}

esp_err_t module_usb_comm_send(module_usb_comm_handle_t handle,
                               const uint8_t *data, size_t len,
                               uint32_t timeout_ms) {
  if (!handle || !handle->initialized || !data || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire mutex for send");
    return ESP_ERR_TIMEOUT;
  }

  if (!handle->connected || !handle->cdc_dev) {
    xSemaphoreGive(handle->mutex);
    return ESP_ERR_INVALID_STATE; /* no device plugged in */
  }

  esp_err_t ret =
      cdc_acm_host_data_tx_blocking(handle->cdc_dev, data, len, timeout_ms);
  xSemaphoreGive(handle->mutex);

  if (ret != ESP_OK) {
    ESP_LOGD(TAG, "USB Stack%d TX failed: %s", handle->stack_id,
             esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGD(TAG, "USB Stack%d sent %d bytes", handle->stack_id, (int)len);
  return ESP_OK;
}

esp_err_t module_usb_comm_receive(module_usb_comm_handle_t handle,
                                  uint8_t *buffer, size_t max_len,
                                  size_t *received, uint32_t timeout_ms) {
  if (!handle || !handle->initialized || !buffer || !received ||
      max_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  /* RX is filled by the CDC driver task into rx_stream regardless of
   * which task calls receive(), so no mutex is needed here. */
  size_t got = xStreamBufferReceive(handle->rx_stream, buffer, max_len,
                                    pdMS_TO_TICKS(timeout_ms));
  *received = got;

  if (got > 0) {
    ESP_LOGD(TAG, "USB Stack%d received %d bytes", handle->stack_id,
             (int)got);
  }
  return ESP_OK;
}

esp_err_t module_usb_comm_take_overflow(module_usb_comm_handle_t handle,
                                        uint32_t *rx_overflow_evt) {
  if (!handle || !handle->initialized) {
    return ESP_ERR_INVALID_ARG;
  }
  portENTER_CRITICAL_SAFE(&handle->ovf_mux);
  uint32_t v = handle->rx_overflow_evt;
  handle->rx_overflow_evt = 0;
  portEXIT_CRITICAL_SAFE(&handle->ovf_mux);
  if (rx_overflow_evt)
    *rx_overflow_evt = v;
  return ESP_OK;
}

esp_err_t module_usb_comm_flush(module_usb_comm_handle_t handle) {
  if (!handle || !handle->initialized) {
    return ESP_ERR_INVALID_ARG;
  }
  /* Drop anything already buffered on the RX side. */
  if (handle->rx_stream) {
    xStreamBufferReset(handle->rx_stream);
  }
  return ESP_OK;
}

esp_err_t module_usb_comm_deinit(module_usb_comm_handle_t handle) {
  if (!handle || !handle->initialized) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Deinitializing USB host Stack%d", handle->stack_id);

  xSemaphoreTake(s_host_lock, portMAX_DELAY);

  /* Stop the connect loop and wait for it to exit so it can't reopen
   * the device while we tear down. */
  handle->running = false;
  if (handle->connect_task) {
    /* connect_task self-deletes; give it time to leave cdc_acm_host_open. */
    vTaskDelay(pdMS_TO_TICKS(USB_OPEN_TIMEOUT_MS + 200));
  }

  xSemaphoreTake(handle->mutex, portMAX_DELAY);
  if (handle->cdc_dev) {
    cdc_acm_host_close(handle->cdc_dev);
    handle->cdc_dev = NULL;
  }
  handle->connected = false;
  xSemaphoreGive(handle->mutex);

  cdc_acm_host_uninstall();

  s_host_lib_running = false;
  /* Wake the event pump so it observes the stopped flag and exits. */
  usb_host_uninstall();

  if (handle->rx_stream)
    vStreamBufferDelete(handle->rx_stream);
  if (handle->rx_stream_storage)
    heap_caps_free(handle->rx_stream_storage);
  if (handle->connect_stack)
    heap_caps_free(handle->connect_stack);
  if (handle->connect_tcb)
    heap_caps_free(handle->connect_tcb);
  if (handle->host_lib_stack)
    heap_caps_free(handle->host_lib_stack);
  if (handle->host_lib_tcb)
    heap_caps_free(handle->host_lib_tcb);
  if (handle->mutex)
    vSemaphoreDelete(handle->mutex);

  handle->initialized = false;
  if (s_active == handle)
    s_active = NULL;
  s_host_lib_task = NULL;

  xSemaphoreGive(s_host_lock);

  free(handle);
  return ESP_OK;
}
