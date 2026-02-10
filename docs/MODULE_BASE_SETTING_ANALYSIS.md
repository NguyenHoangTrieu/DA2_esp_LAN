# MODULE BASE SETTING - PHÂN TÍCH TOÀN HỆ THỐNG & DANH SÁCH CÔNG VIỆC

**Ngày tạo:** 2026-02-08  
**Ngày cập nhật:** 2026-02-08 (Added Task 1.0, 1.5 and updated initialization flow)  
**Scope:** DA2_esp_LAN - Module Base Setting (dành cho LAN, chưa áp dụng cho WAN)  
**Tác vụ:** Phân tích gap giữa yêu cầu trong TODO.md và implementation hiện tại

---

## 📋 EXECUTIVE SUMMARY (NEW)

### What Changed in This Update?

**3 major updates:**

1. **Task 1.0: Module Monitor Task (NEW)**
   - Foundation task để quản lý lifecycle của module handlers
   - Chịu trách nhiệm khởi động/dừng BLE/Zigbee/LoRa handler tasks
   - Lưu/load JSON config từ NVS
   - **Initialization logic thay đổi:** KHÔNG tự động parse JSON nữa, chờ JSON config từ PC app

2. **Task 1.5: Config Handler Updates for 3 Flows (NEW)**
   - Merge từ UPDATE_CONFIG_HANDLER.md
   - Implement command parsing cho 3 luồng:
     - Luồng 1: JSON Config (`BL:JSON:<len>:<json>`)
     - Luồng 4: Discovery (`BL:DISC:<timeout>:<stack>`)
     - Luồng 5: Setup Commands (`BL:SETUP:<func>:<stack>:<params>`)
   - 15-20 hours estimated

3. **Task 1.3 DEPRECATED, Task 1.4 Simplified**
   - Task 1.3 functionality merged vào Task 1.0 (NVS) và Task 1.5 (parsing)
   - Task 1.4 giờ chỉ cần forward packets, không parse chi tiết

### Updated Estimates

| Phase | Original | Updated | Delta |
|-------|----------|---------|-------|
| Hotfix | 2h | 2h | 0 |
| Priority 1 | 38-48h | 55-69h | +17-21h |
| Priority 2 | 20-28h | 20-28h | 0 |
| Priority 3 | 8-12h | 8-12h | 0 |
| **Total** | **68-90h** | **85-111h** | **+17-21h** |

**Timeline:** 2.5-3 weeks (vs original 2 weeks)

### New Initialization Flow

**OLD (deprecated):**
```
Boot → Auto parse JSON from NVS → Start handler tasks
```

**NEW (correct):**
```
Boot → Module Monitor starts → Wait for JSON from PC app → 
Parse JSON → Start appropriate handler task (BLE/Zigbee/LoRa)
```

### Priority Order (Updated)

1. 🔴 Hotfixes (2h)
2. 🔴 **Task 1.0: Module Monitor** (10-12h) - **FOUNDATION**
3. 🔴 Task 1.1: BLE Handler Middleware (14-18h)
4. 🔴 Task 1.2: BLE Handler Task (14-16h)
5. 🔴 **Task 1.5: Config Handler 3 Flows** (15-20h) - **CRITICAL**
6. 🔴 Task 1.4: MCU_LAN_Handler (2-3h)
7. 🟡 Task 2.x: Integration & validation

---

## 1. YÊU CẦU HỆ THỐNG (từ TODO.md)

### 1.1 Lý thuyết Chung
Mục tiêu: Tạo gateway có khả năng **configurable & modular** bằng cách sử dụng JSON file để định dạng tập lệnh của các module theo **hướng chức năng** (function-oriented) thay vì định dạng chung (command-format).

**Lợi ích:**
- Không cần nạp firmware lại cho mỗi module khác nhau
- Chỉ cần thay đổi JSON config → gateway sẽ hoạt động với module mới
- Tiết kiệm thời gian phát triển và flash memory

### 1.2 Các Luồng Dữ Liệu Chính (5 luồng)

| # | Luồng | Hướng | Mô tả |
|---|-------|-------|-------|
| 1 | **Config Module** | App → WAN → LAN | JSON config từ app → WAN MCU → SPI → LAN MCU → parse JSON → áp dụng config |
| 2 | **Sensor Data** | LAN → WAN → App | Dữ liệu cảm biến từ LAN → SPI → WAN → UART → Server |
| 3 | **Module Control** | App → WAN → LAN | Lệnh điều khiển từ app → WAN → SPI → LAN → thực hiện lệnh (dựa JSON config) |
| 4 | **Device Discovery** | App → WAN → LAN → App | Scan/discovery device → WAN → SPI → LAN → tìm device → kết quả về app |
| 5 | **Setup Commands** | App → WAN → LAN → App | Reset, set name, set RF params, etc. → WAN → SPI → LAN → thực hiện → kết quả về |

### 1.3 Cấu Trúc JSON Mẫu (Cho BLE)

```json
{
  "module_id": "001",
  "module_type": "BLE",
  "module_name": "JDY-23",
  "module_communication": {
    "port_type": "uart",
    "parameters": {
      "baudrate": 9600,
      "parity": "none",
      "stopbit": 1
    }
  },
  "functions": [
    {
      "function_name": "MODULE_HW_RESET",
      "command": "",
      "gpio_start_control": [{"pin": "01", "state": "LOW"}],
      "delay_start": 100,
      "expect_response": "",
      "timeout": 0,
      "gpio_end_control": [{"pin": "02", "state": "HIGH"}],
      "delay_end": 500
    },
    {
      "function_name": "MODULE_SW_RESET",
      "command": "AT+RST\r\n",
      ...
    },
    ...
  ]
}
```

### 1.4 Các Hàm BLE Cần Hỗ Trợ (20 Functions)

#### Core Functions (15)
1. `MODULE_HW_RESET` - Reset cứng GPIO
2. `MODULE_SW_RESET` - Reset mềm qua lệnh
3. `MODULE_FACTORY_RESET` - Reset về default
4. `MODULE_GET_INFO` - Đọc version/model
5. `MODULE_SET_NAME` - Đặt tên thiết bị
6. `MODULE_SET_COMM_CONFIG` - Cài baud/parity/stop (UART), clock (SPI), addr (I2C)
7. `MODULE_SET_RF_PARAMS` - Cài TX power/channel
8. `MODULE_ENTER_CMD_MODE` - Chuyển sang nhận lệnh AT
9. `MODULE_ENTER_DATA_MODE` - Chuyển sang transparent UART↔RF
10. `MODULE_START_BROADCAST` - Bật quảng bá (slave) hoặc permit join (coordinator)
11. `MODULE_CONNECT` - Kết nối đến địa chỉ (master/central)
12. `MODULE_DISCONNECT` - Ngắt kết nối
13. `MODULE_GET_CONNECTION_STATUS` - Kiểm tra trạng thái link
14. `MODULE_ENTER_SLEEP` - Vào sleep mode
15. `MODULE_WAKEUP` - Đánh thức từ sleep

#### Optional Functions (5)
16. `MODULE_START_DISCOVERY` - Scan network (nếu là central/coordinator)
17. `MODULE_SEND_DATA` - Send data qua command (nếu module không dùng transparent)
18. `MODULE_GET_DIAGNOSTICS` - Đọc RSSI/link quality
19. `MODULE_SET_SECURITY_CONFIG` - Pairing/PIN
20. `MODULE_ENTER_BOOTLOADER` - OTA update module

---

## 2. PHÂN TÍCH IMPLEMENTATION HIỆN TẠI

### 2.1 Các Component Đã Tồn Tại

#### ✅ Đã Implement Đầy Đủ

**A. Middleware/JSON_Config_Parser/**
- `json_config_parser.h/c` - Parser chung cho metadata + communication config
  - ✅ Hỗ trợ 4 loại communication: UART, SPI, I2C, USB
  - ✅ Parse metadata (module_id, module_type, module_name)
  - ✅ Parse communication parameters (baudrate, parity, stopbit, etc.)
  - ✅ GPIO control structure (pin + state)
  
- `json_ble_config_parser.h/c` - Parser riêng cho BLE
  - ✅ Định nghĩa 20 BLE function IDs (JSON_BLE_FUNC_*)
  - ✅ Parse 20 functions từ JSON array
  - ✅ Validate function names
  - ✅ Trích xuất command, GPIO controls, delays, expect_response, timeout
  - ✅ Cấu trúc `json_ble_module_config_t` lưu trữ toàn bộ config

**B. BSP/Module_*_Communication/**
- `module_uart_comm.h/c` - UART interface
  - ✅ Init/deinit UART
  - ✅ Send/receive data qua UART
  - ✅ Baud rate, parity, stopbit config
  
- `module_spi_comm.h/c` - SPI interface
  - ✅ Init/deinit SPI
  - ✅ Send/receive data qua SPI
  - ✅ Clock speed config
  
- `module_i2c_comm.h/c` - I2C interface
  - ✅ Init/deinit I2C
  - ✅ Read/write qua I2C
  - ✅ Address + clock speed config
  
- `module_usb_comm.h/c` - USB interface
  - ⚠️ Header defined nhưng implementation chưa hoàn chỉnh

**C. BSP/stack_handler/**
- ✅ Quản lý GPIO cho các stack (Stack 0, Stack 1)
- ✅ GPIO set/get direction
- ✅ GPIO set/read level
- ✅ Hỗ trợ 8 GPIO pins per stack (GPIO00-GPIO08)

#### ⚠️ Partial Implementation

**A. Middleware/BLE_Handler/**
- Header `ble_handler.h` - ✅ Define API
- Source `ble_handler.c` - ⚠️ Functions exist nhưng:
  - Execute function API defined
  - ⚠️ **Missing:** Chưa có hàm để parse BLE UART data frame (extract MAC + payload)
  - ⚠️ **Missing:** Chưa có hàm quản lý kết nối device
  - ⚠️ **Incomplete:** JSON config loading chưa fully connected

**B. Application/BLE_Handler/**
- Header `ble_handler_task.h` - ✅ API defined
- Source `ble_handler_task.c` - ⚠️ Task skeleton exists nhưng:
  - ⚠️ **Empty implementations** - chỉ có placeholder
  - ⚠️ **Missing:** Uplink queue processing
  - ⚠️ **Missing:** Downlink handling
  - ⚠️ **Missing:** Device discovery logic
  - ⚠️ **Missing:** Connection status tracking

**C. Middleware/Module_Config_Controller/**
- Header `module_config_controller.h` - ✅ API defined
- Source `module_config_controller.c` - ⚠️ Partial:
  - ✅ Module config controller init
  - ✅ UART/SPI/I2C/USB init routing
  - ⚠️ **Missing:** Config storage/persistence
  - ⚠️ **Missing:** Error handling for unsupported comm types

#### ❌ Chưa Implement

**Các lỗi cơ bản:**
1. ❌ **Integration layer đứt gãy** - JSON parser có nhưng chưa kết nối tới module_config_controller
2. ❌ **BLE Handler middleware** - Có skeleton nhưng missing core logic
3. ❌ **BLE Handler task** - Có header nhưng toàn placeholder
4. ❌ **Config Handler update** - Chưa có hàm để nhận JSON config từ MCU WAN

---

## 3. CHI TIẾT CÁC LỖ HỔNG (GAP ANALYSIS)

### 3.1 Layer Architecture Missing

```
❌ App (on WAN MCU) 
   ↓ [JSON config string]
   ❌ UART Handler (WAN MCU) - Chưa handle JSON config receive
   ↓ [JSON → SPI]
   ✅ MCU_WAN_Handler (WAN MCU) - Có thể route qua SPI
   ↓ [JSON → LAN MCU]
   ❌ MCU_LAN_Handler - Chưa route JSON tới config_handler
   ↓
   ❌ Config_Handler (LAN MCU) - Chưa parse JSON module config
   ↓
   ❌ ble_handler_load_config() - Chưa được gọi
   ↓
   ❌ ble_handler_task - Chưa thực hiện (empty implementation)
```

### 3.2 Core Functions Missing

| Component | Function | Status | Chi tiết |
|-----------|----------|--------|---------|
| JSON_BLE_Config_Parser | `json_ble_config_parse()` | ✅ | Parse 20 functions từ JSON |
| BLE_Handler Middleware | `ble_handler_load_config()` | ⚠️ Partial | API exist, impl chưa hoàn chỉnh |
| BLE_Handler Middleware | `ble_execute_function()` | ❌ | Execute AT command từ config |
| BLE_Handler Middleware | `ble_parse_frame()` | ❌ | Parse UART/SPI frame từ device |
| BLE_Handler Task | `ble_handler_uplink_task()` | ❌ | Collect dữ liệu từ device |
| BLE_Handler Task | `ble_handler_downlink_task()` | ❌ | Gửi lệnh tới device |
| Config_Handler | `config_parse_ble_json()` | ❌ | Parse JSON config command |
| Module_Config_Controller | `module_ctrl_load_config()` | ❌ | Load config từ JSON |
| Stack_Handler | GPIO mapping | ⚠️ | Comments confused about pin numbering |

### 3.3 Data Flow Missing/Broken

#### Luồng 1: Config Module (App → LAN)
```
App (WAN MCU):
  ❌ Chưa có mechanism để gửi JSON config

WAN MCU:
  ✅ uart_handler_task có thể receive config từ app
  ❌ Chưa route JSON config qua SPI tới LAN MCU

LAN MCU:
  ❌ mcu_lan_handler chưa recognize JSON config packet
  ❌ config_handler chưa có `config_parse_ble_json()` function
  ❌ ble_handler_load_config() chưa được gọi
```

#### Luồng 2: Sensor Data (LAN → App)
```
BLE Device:
  ✅ Có UART interface để giao tiếp

BLE Handler Task:
  ❌ Chưa có uplink task để collect dữ liệu
  ❌ Chưa có logic để parse UART frame
  ❌ Chưa có batching/queue mechanism

MCU_WAN Handler:
  ✅ Có QSPI interface để truyền dữ liệu

WAN MCU:
  ✅ UART Handler có thể gửi tới app
```

#### Luồng 3: Module Control (App → LAN → Device)
```
App:
  ❌ Chưa có command format để điều khiển module

Config Handler:
  ❌ Chưa handle module control command

BLE Handler Task:
  ❌ Chưa có downlink task
  ❌ Chưa có logic để execute function (gửi AT command)
  ❌ Chưa track device connection status
```

#### Luồng 4 & 5: Discovery & Setup
```
❌ Hoàn toàn chưa implement
```

---

## 4. DANH SÁCH CÔNG VIỆC CẦN LÀMNGAY (TODO LIST)

### � APPROVED ITEMS SUMMARY (From CODE_ANALYSIS_REPORT.md Review)
**12 items marked "duyệt" (approved for deployment):**

#### Critical Bugs (3)
1. module_uart_comm.c:88 - uart_port missing in config struct
2. ble_handler_task.c - Timeout calculation (TickCount ≠ ms)
3. stack_handler.c - GPIO mapping comments wrong

#### Enhancements (9)
4. BLE Handler: Add automatic connection recovery
5. BLE Handler: Command string validation (sanity check)
6. BLE Handler: Support both ASCII (AT) and binary (0xC0) formats
7. BLE Task: Fix & enhance error handling for lost uplink data
8. BLE Task: Support multiple BLE modules on different stacks
9. UART: Add recv_bytes function for fixed byte count
10. UART: Flow control configuration support
11. I2C: Timeout configuration + 10-bit addressing + multi-device support
12. USB: Event callbacks + line coding validation

**Items marked "loại" (rejected/skip):**
- Timeout per function (skip - not needed)
- Dynamic port switching (skip - reinit instead)
- Separate SPI send/receive (skip - full-duplex ok)
- DMA for UART (skip - not critical)
- Burst write for I2C (skip - simpler without)
- Serial number config for USB (skip)
- And 6 other items (see CODE_ANALYSIS_REPORT.md)

---

### �🔴 **PRIORITY 0: HOTFIX - Phải fix ngay (Critical Bugs)**

#### Hotfix 0.1: module_uart_comm.c Compilation Error
**File:** `BSP/Module_UART_Communication/src/module_uart_comm.c` Line 88
**Issue:** `uart_driver_delete(config->uart_port)` - config struct không có field `uart_port`
**Fix:**
- [ ] Save `uart_port` vào internal struct `module_uart_comm_s` during init
- [ ] Use saved `uart_port` value trong deinit

#### Hotfix 0.2: stack_handler.c GPIO Mapping Comments
**File:** `BSP/stack_handler/src/stack_handler.c`
**Issue:** Comments show wrong GPIO pin assignments
**Fix:**
- [ ] Update comments to match actual code implementation
- [ ] Verify GPIO mappings: Stack 1 uses P02-P07, P10-P12 (clarify which GPIO numbers)

#### Hotfix 0.3: ble_handler_task.c Idle Device Cleanup
**File:** `Application/BLE_Handler/src/ble_handler_task.c`
**Issue:** Timeout calculation wrong - timestamps là TickCount chứ không phải milliseconds
**Fix:**
- [ ] Convert TickCount properly: `elapsed = (now - last_activity) / portTICK_PERIOD_MS`
- [ ] Test timeout logic với actual devices

---

### 🔴 **PRIORITY 1: CRITICAL - Phải làm trước

#### Task 1.0: Module Monitor Task (NEW - Foundation)
**File:** `Application/Module_Monitor_Task/src/module_monitor_task.c/.h` (NEW)
**Công việc:**

**Mục đích:** Task này sẽ chịu trách nhiệm:
- Khởi động/dừng các handler tasks (BLE/Zigbee/LoRa) dựa trên JSON config
- Kiểm tra stack ID (module nào đang cắm trên stack nào)
- Quản lý lifecycle của module handlers
- Lưu/load config từ NVS

**API Design:**
```c
// module_monitor_task.h
typedef enum {
    MODULE_TYPE_NONE = 0,
    MODULE_TYPE_BLE = 1,
    MODULE_TYPE_ZIGBEE = 2,
    MODULE_TYPE_LORA = 3,
    MODULE_TYPE_UNKNOWN = 0xFF
} module_type_t;

typedef struct {
    uint8_t stack_id;           // 0 or 1
    module_type_t module_type;  // BLE/Zigbee/LoRa
    bool is_configured;         // Has JSON config
    bool is_running;            // Handler task running
    void *config_data;          // Pointer to parsed config
} module_info_t;

// Public APIs
esp_err_t module_monitor_task_start(void);
esp_err_t module_monitor_task_stop(void);
esp_err_t module_monitor_load_config(uint8_t stack_id, const char *json_str, uint16_t len);
module_type_t module_monitor_get_stack_type(uint8_t stack_id);
bool module_monitor_is_configured(uint8_t stack_id);
esp_err_t module_monitor_start_handler(uint8_t stack_id);
esp_err_t module_monitor_stop_handler(uint8_t stack_id);
```

**Khởi động logic (UPDATED - KHÔNG tự động parse JSON):**
```
1. System boot
   ↓
2. module_monitor_task_start()
   ↓
3. Check NVS for saved JSON config
   ↓
   [If NO JSON config]
   ↓
4. Wait for JSON config từ app (via Config Handler)
   - Hiển thị LED báo "waiting for config"
   - Listen trên config queue
   ↓
5. App sends JSON config qua gateway↔PC connection
   ↓
6. Config Handler receives "BL:JSON:<len>:<json>"
   ↓
7. Config Handler calls module_monitor_load_config(stack_id, json_str, len)
   ↓
8. Module Monitor parses JSON → determines module type
   ↓
9. Module Monitor starts appropriate handler task:
   - BLE → ble_handler_task_start()
   - Zigbee → zigbee_handler_task_start()
   - LoRa → lora_handler_task_start()
   ↓
10. Save JSON config to NVS (for next boot)
    ↓
11. LED báo "configured OK"
```

**Implementation Details:**
- [ ] Create `module_monitor_task.c/.h` files
- [ ] Implement NVS save/load functions:
  - `module_monitor_save_config_to_nvs(uint8_t stack_id, const char *json)`
  - `module_monitor_load_config_from_nvs(uint8_t stack_id, char **json, uint16_t *len)`
  
- [ ] Implement stack type detection:
  - Check GPIO levels (module ID pins)
  - OR parse JSON "module_type" field
  - Return module_type_t enum
  
- [ ] Implement handler task lifecycle management:
  - Track task handles trong module_info_t array[2]
  - Start/stop tasks based on config
  - Handle task cleanup on stop
  
- [ ] Implement config queue processing:
  - Listen trên `monitor_config_queue` (from Config Handler)
  - Parse incoming JSON
  - Route to appropriate handler init
  
- [ ] Implement LED status indication:
  - Waiting for config: LED blink slow
  - Config loaded: LED solid green
  - Config error: LED blink fast red
  - Running: LED off (or dim)
**Implementation Status:** ✅ COMPLETED (11 hours)

**Files Created:**
- `Application/Module_Monitor_Task/include/module_monitor_task.h` (232 lines)
- `Application/Module_Monitor_Task/src/module_monitor_task.c` (622 lines)
- `Application/Module_Monitor_Task/include/module_monitor_task_internal.h`
- `Application/Module_Monitor_Task/CMakeLists.txt`

**Implementation Highlights:**
- ✅ 11 public APIs fully implemented
- ✅ NVS save/load functions (with 8KB size limit sanity check)
- ✅ JSON module type detection (cJSON-based parsing)
- ✅ Task lifecycle management (start/stop/status tracking)
- ✅ Config queue processing (non-blocking queue, 5s timeout)
- ✅ Mutex-protected global state for 2 stacks
- ✅ Auto-loading saved configs from NVS on boot
- ⏸️ LED status indication: Deferred (not critical, can add later)

**Integration Status:**
- Config Handler can now call `module_monitor_load_config()` for JSON routing
- Module info tracking ready for Tasks 1.1/1.2 integration
- Handler task lifecycle ready (TODO: implement actual handler start/stop calls)

**Dependency:** stack_handler (GPIO check), json_ble_config_parser, ble_handler_task API

---

#### Task 1.1: Hoàn thiện BLE Handler Middleware
**File:** `Middleware/BLE_Handler/src/ble_handler.c`
**Công việc:**
- [ ] Implement `ble_handler_load_config(const char *json_str, uint16_t len)`
  - Parse JSON qua `json_ble_config_parse()`
  - Initialize Module Config Controller
  - Route communication port tới đúng BSP init (UART/SPI/I2C/USB)
  - Store config vào global struct
  
- [ ] Implement `ble_execute_function(uint8_t function_id, const char *param)`
  - Lookup function config từ global struct
  - Execute GPIO start controls
  - Send AT command nếu có
  - Wait for expect response hoặc timeout
  - Execute GPIO end controls
  - Return status
  - **[APPROVED]** Add command string validation (sanity check để tránh buffer overflow - check command length, ASCII vs binary format support for both "AT+..." strings và hex binary like "0xC0 0xC0 0xC0")
  
- [ ] Implement `ble_parse_frame(const uint8_t *data, uint16_t len, uint8_t *mac, uint8_t *payload, uint16_t *payload_len)`
  - Parse UART/SPI frame từ BLE device
  - Extract source MAC address (6 bytes)
  - Extract payload data
  - Return success/fail
  
- [ ] Implement `ble_send_command(uint8_t stack_id, const char *cmd, char *response, uint16_t resp_len, uint16_t timeout_ms)`
  - Route cmd tới đúng communication interface (UART/SPI/I2C/USB)
  - Wait for response hoặc timeout
  
- [ ] Implement device management:
  - `ble_add_device(const uint8_t *mac_address)`
  - `ble_remove_device(const uint8_t *mac_address)`
  - `ble_get_device_count()`
  - Internal list của connected devices
  - **[APPROVED]** Implement automatic connection recovery mechanism (retry khi command timeout, fallback to SW reset nếu HW reset fail)

- [ ] **[APPROVED]** Parse JSON config error handling - add detailed error logging (not generic ESP_ERR_INVALID_ARG)

- [ ] **[APPROVED]** Support command string validation cho cả ASCII (AT commands) và binary format (hex like "0xC0 0xC0 0xC0")

**Dependency:** json_ble_config_parser, module_config_controller

---

#### Task 1.2: Hoàn thiện BLE Handler Task
**File:** `Application/BLE_Handler/src/ble_handler_task.c`
**Công việc:**
- [ ] Implement uplink task `ble_handler_uplink_task(void *arg)`
  - Block trên uplink queue
  - Receive packet từ BLE device
  - Parse frame (extract MAC + payload)
  - Batch packets (max 8 hoặc 50ms timeout)
  - Send batched data tới MCU WAN via `mcu_wan_enqueue_uplink()`
  - Track device activity (last_activity_time)
  - Periodic cleanup idle devices (>60s)
  - **[APPROVED]** Fix timeout calculation logic (timestamps là TickCount chứ không phải ms - convert correctly)
  - **[APPROVED]** Add error handling khi enqueue_uplink fails (implement retry logic hoặc log warning để track lost data)
  
- [ ] Implement downlink task `ble_handler_downlink_task(void *arg)`
  - Block trên downlink queue
  - Receive control command từ MCU WAN
  - Lookup device MAC từ command
  - Verify device is connected
  - Execute `ble_execute_function()` hoặc send data
  - Return result
  
- [ ] Implement `ble_handler_task_start()`
  - Create uplink queue (size 20)
  - Create downlink queue (size 20)
  - Create uplink task (priority 4, stack 4KB)
  - Create downlink task (priority 5, stack 4KB)
  
- [ ] Implement `ble_handler_task_stop()`
  - Stop both tasks
  - Delete queues
  - Free resources
  
- [ ] Implement `ble_handler_task_enqueue_uplink(const uint8_t *data, uint16_t len)`
  - Add packet tới uplink queue (non-blocking)
  
- [ ] Implement `ble_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len)`
  - Add packet tới downlink queue (non-blocking)
  
- [ ] Implement discovery handling `ble_handler_task_start_discovery()`
  - Call `ble_execute_function(MODULE_START_DISCOVERY, NULL)`
  - Collect discovered devices từ UART response
  - Return list of devices (MAC + RSSI)

- [ ] **[APPROVED]** Support multiple BLE modules trên stack khác nhau (extend data structures để track stack_id, extend task creation)

**Dependency:** ble_handler middleware, frame_types.h

---

#### Task 1.5: Config Handler Updates for 3 Flows (NEW - from UPDATE_CONFIG_HANDLER.md)
**Files:** 
- `Application/Config_Handler/src/config_handler.c` (DA2_esp - WAN MCU)
- `Application/Config_Handler/src/config_handler.c` (DA2_esp_LAN - LAN MCU)

**Công việc:**

**Mục đích:** Update config_handler để hỗ trợ 3 luồng Module Base Setting:
1. **Luồng 1: JSON Config** - App → WAN → LAN → Module Monitor
2. **Luồng 4: Discovery** - App ⇄ WAN ⇄ LAN ⇄ BLE Handler
3. **Luồng 5: Setup Commands** - App ⇄ WAN ⇄ LAN ⇄ BLE Handler

**Command Formats (see UPDATE_CONFIG_HANDLER.md):**
- JSON Config: `BL:JSON:<len>:<json>` (WAN) → `CFBL:JSON:<len>:<json>` (LAN)
- Discovery: `BL:DISC:<timeout>:<stack>` (WAN) → `CFBL:DISC:<timeout>:<stack>` (LAN)
- Setup: `BL:SETUP:<func>:<stack>:<params>` (WAN) → `CFBL:SETUP:<func>:<stack>:<params>` (LAN)

**WAN MCU Updates:**
- [ ] Add `CONFIG_TYPE_BLE` to `config_type_t` enum
- [ ] Implement `config_parse_ble_json()` function
  - Parse `BL:JSON:<len>:<json>` command
  - Extract JSON length and data
  - Forward to LAN MCU via `mcu_lan_send_config()`
  - Handle response (ACK/NACK)
  
- [ ] Implement `config_parse_ble_discovery()` function
  - Parse `BL:DISC:<timeout>:<stack>` command
  - Validate timeout (1000-60000 ms) and stack_id (0-1)
  - Forward to LAN MCU
  - Wait for discovery results
  - Return results to app
  
- [ ] Implement `config_parse_ble_setup()` function
  - Parse `BL:SETUP:<func>:<stack>:<params>` command
  - Validate function_id (0-19) and stack_id (0-1)
  - Forward to LAN MCU
  - Wait for execution result
  - Return result to app
  
- [ ] Update `config_handler_task()` to route BLE commands
  - Detect "BL:" prefix
  - Route to appropriate parse function

**LAN MCU Updates:**
- [ ] Add `CONFIG_TYPE_BLE` to `config_type_t` enum
- [ ] Implement `config_parse_ble_json()` function
  - Parse `CFBL:JSON:<len>:<json>` command
  - Extract JSON string
  - Call `module_monitor_load_config(stack_id, json, len)`
  - Return ACK to WAN MCU
  
- [ ] Implement `config_parse_ble_discovery()` function
  - Parse `CFBL:DISC:<timeout>:<stack>` command
  - Call `ble_handler_task_start_discovery(stack_id, timeout)`
  - Collect discovered devices (MAC + RSSI)
  - Format response: `BL:DISC:RESULT:<count>:<mac1>:<rssi1>,...`
  - Send result to WAN MCU
  
- [ ] Implement `config_parse_ble_setup()` function
  - Parse `CFBL:SETUP:<func>:<stack>:<params>` command
  - Extract function_id, stack_id, parameters
  - Call `ble_execute_function(function_id, params)`
  - Format response: `BL:SETUP:RESULT:<func>:<status>:<data>`
  - Send result to WAN MCU
  
- [ ] Update `config_handler_task()` to route BLE commands
  - Detect "CFBL:" prefix
  - Route to appropriate parse function

**Implementation Code (see UPDATE_CONFIG_HANDLER.md sections 4-6):**
- Section 4: JSON Config implementation (~120 lines)
- Section 5: Discovery implementation (~180 lines)
- Section 6: Setup Commands implementation (~150 lines)

**Testing:**
- [ ] Test JSON config loading (valid + invalid JSON)
- [ ] Test discovery with timeout variations
- [ ] Test setup commands với 20 function IDs
- [ ] Test error handling (invalid params, timeout, etc.)

**Estimated Hours:** 15-20 hours (based on UPDATE_CONFIG_HANDLER.md section 10.2)

**Reference:** See `/home/trieunguyen/DATN_Workspace/UPDATE_CONFIG_HANDLER.md` for detailed implementation code

**Dependency:** module_monitor_task (Task 1.0), ble_handler (Task 1.1), ble_handler_task (Task 1.2)

---

#### Task 1.3: Config Handler Integration (LAN MCU) - DEPRECATED
**File:** `Application/Config_Handler/src/config_handler.c` (DA2_esp_LAN)
**Status:** ⚠️ SUPERSEDED by Task 1.5 (Config Handler Updates for 3 Flows)

**Original Scope (now merged into Task 1.5):**
- Add `config_parse_ble_json()` function → Moved to Task 1.5
- Update `config_handler_task()` → Moved to Task 1.5
- Add JSON config persistence → Moved to Task 1.0 (Module Monitor)

**Note:** Không cần làm task này riêng, tất cả đã được merge vào Task 1.5

**Estimated Hours:** 0 (merged into other tasks)

**Dependency:** N/A

---

#### Task 1.4: MCU_LAN_Handler Update (Simplified)
**File:** `Application/MCU_LAN_Handler/src/mcu_lan_handler.c` (DA2_esp_LAN)
**Công việc:**
- [ ] Update packet receiver để recognize BLE command packets
  - Detect command prefix "CFBL:" (BLE commands from WAN MCU)
  - Route all BLE packets tới config_handler queue
  
- [ ] Simple forwarding logic:
  - No need to parse command details here
  - Config Handler (Task 1.5) will handle routing to appropriate functions
  - Just detect packet type and forward

**Note:** Task này giờ đơn giản hơn vì Task 1.5 đã handle tất cả command parsing và routing logic.

**Estimated Hours:** 2-3 hours (reduced from 4-6 hours)

**Dependency:** config_handler (Task 1.5)

---

### 🟡 **PRIORITY 2: HIGH - Công việc chính**

#### Task 2.1: MCU_WAN_Handler Update (WAN MCU)
**File:** `Application/MCU_WAN_Handler/` (DA2_esp)
**Công việc:**
- [ ] Update UART handler để receive/transmit JSON config
  - Receive config từ app (via UART/USB)
  - Route config qua SPI tới LAN MCU via `mcu_wan_enqueue_uplink()` hoặc similar
  
- [ ] Update SPI packet router
  - Route JSON config packets tới LAN MCU
  - Route sensor data packets từ LAN MCU tới app
  - Route control commands từ app tới LAN MCU

**Dependency:** uart_handler, mcu_wan_handler core

---

#### Task 2.2: Config Tool Update (App - Optional for now)
**File:** `config_app/` (DA2_esp)
**Công việc:**
- [ ] Add JSON config generator
  - Provide UI để upload BLE JSON config
  - Generate JSON string từ user input
  - Send JSON config qua UART tới WAN MCU
  
- [ ] Add device discovery UI
  - Send discovery command tới LAN MCU
  - Display discovered devices (MAC + RSSI)
  
- [ ] Add module control UI
  - Send control commands tới devices
  - Display response results

**Note:** Có thể skip phase 1, focus vào firmware trước

---

#### Task 2.3: Error Handling & Validation
**Files:** Tất cả files liên quan
**Công việc:**
- [ ] Add validation cho JSON parsing
  - Check required fields
  - Validate function names
  - Validate communication parameters
  
- [ ] Add error logging
  - Log parse errors
  - Log execution errors
  - Log timeout errors
  
- [ ] Add recovery mechanisms
  - Retry failed commands (với exponential backoff)
  - Fallback to SW reset nếu HW reset fail
  - Cleanup stale devices

- [ ] **[APPROVED]** Fix critical bug in module_uart_comm.c:88
  - `uart_driver_delete(config->uart_port)` calls wrong field (uart_port not in config)
  - Must save uart_port in module_uart_comm_s struct during init

- [ ] **[APPROVED]** Fix GPIO mapping comments in stack_handler.c
  - Current comments show wrong GPIO pin assignments
  - Update to match actual code: Stack 1: P02-P07, P10-P12 (fix comments)

- [ ] **[APPROVED]** Add proper flow control configuration in module_uart_comm.c
  - Currently hardcoded disabled, allow configuration

- [ ] **[APPROVED]** Add I2C timeout configuration in module_i2c_comm.c
  - Currently hardcoded, allow configuration from config struct

- [ ] **[APPROVED]** Validate GPIO pin ID format in json_ble_config_parser.c
  - Currently only stores string, should validate format during parsing

- [ ] **[APPROVED]** Add USB event callbacks support in module_usb_comm
  - Support connection/disconnection events

- [ ] **[APPROVED]** Add recv_bytes function in module_uart_comm
  - Only has receive with timeout, may need separate recv_bytes for fixed byte count

- [ ] **[APPROVED]** Handle SPI transaction queue for multiple commands
  - Currently only handles single spi_device_transmit

- [ ] **[APPROVED]** Support I2C multi-device and 10-bit addressing
  - Add support for multiple devices on same I2C bus
  - Add 10-bit addressing mode

---

### 🟢 **PRIORITY 3: MEDIUM - Optimization & Polish**

#### Task 3.1: Performance Optimization
**Công việc:**
- [ ] Batch uplink packets (max 8 hoặc 50ms flush)
- [ ] Reduce latency trong command execution
- [ ] Add DMA support cho UART/SPI nếu applicable
- [ ] Implement async command handling

#### Task 3.2: Testing & Documentation
**Công việc:**
- [ ] Unit test cho JSON parser
- [ ] Integration test cho config loading
- [ ] Integration test cho command execution
- [ ] API documentation
- [ ] Data flow diagrams (Mermaid)

#### Task 3.3: Module-Specific Support
**Công việc:**
- [ ] Add support cho other modules (Zigbee, LoRa, CAN)
  - Create `json_zigbee_config_parser.h/c`
  - Create `json_lora_config_parser.h/c`
  - Reuse chung infrastructure

---

## 5. TECHNICAL CHECKLIST

### 5.1 Communication Interfaces (BSP Layer) ✅

- [x] UART Communication (module_uart_comm.h/c)
  - [x] Init/deinit
  - [x] Send/receive
  - [x] Timeout handling
  - [ ] **[APPROVED]** Add recv_bytes function (receive fixed number of bytes)
  - [ ] **[APPROVED]** Flow control configuration support
  
- [x] SPI Communication (module_spi_comm.h/c)
  - [x] Init/deinit
  - [x] Send/receive
  - [ ] **[APPROVED]** Support SPI transaction queue for multiple commands
  
- [x] I2C Communication (module_i2c_comm.h/c)
  - [x] Init/deinit
  - [x] Read/write
  - [x] Address support
  - [ ] **[APPROVED]** Support 10-bit I2C addressing
  - [ ] **[APPROVED]** Support multi-device on same I2C bus
  - [ ] **[APPROVED]** I2C timeout configuration (currently hardcoded)

- [ ] USB Communication (module_usb_comm.h/c)
  - [x] Send/receive functions implemented (not placeholder)
  - [ ] **[APPROVED]** Validate line_coding values
  - [ ] **[APPROVED]** USB event callbacks (connection/disconnection)
  - [ ] **[APPROVED]** Support VID/PID configuration

### 5.2 GPIO Control (BSP Layer) ✅

- [x] Stack handler GPIO mapping
  - [x] 8 GPIO pins per stack
  - [x] Set direction (input/output)
  - [x] Read/write level
  - [ ] **[APPROVED]** Implement stack_handler_get_stack_id() (currently commented as TODO)
  
- [ ] **[APPROVED]** Fix GPIO mapping comments (update to match actual implementation)

### 5.3 JSON Parsing (Middleware Layer) ✅

- [x] Common metadata parser (json_config_parser.h/c)
- [x] BLE-specific parser (json_ble_config_parser.h/c)
- [ ] **[APPROVED]** Add GPIO pin ID validation (parse "GPIO12" format correctly)
- [ ] **[APPROVED]** Add detailed error handling (not generic ESP_ERR_INVALID_ARG)
- [ ] **[APPROVED]** Support command format validation (ASCII "AT+..." vs binary "0xC0 0xC0")
- [ ] Extend for other modules (Zigbee, LoRa)

### 5.4 Module Config Controller (Middleware Layer) ⚠️

- [x] UART init routing
- [x] SPI init routing
- [x] I2C init routing
- [ ] USB init routing (complete USB send/receive first)
- [ ] **[APPROVED]** Error recovery nếu comm port initialization fails (add retry logic)
- [ ] Config persistence (NVS save/load)

### 5.5 BLE Handler (Middleware Layer) ⚠️

- [ ] Config loading from JSON
- [ ] Function execution (command + GPIO control)
- [ ] UART frame parsing
- [ ] Device management
- [ ] Error handling
- [ ] **[APPROVED]** Command string validation (sanity check, buffer overflow protection)
- [ ] **[APPROVED]** Automatic connection recovery mechanism
- [ ] **[APPROVED]** Support both ASCII (AT+...) and binary (0xC0...) command formats

### 5.6 BLE Handler Task (Application Layer) ❌

- [ ] Uplink task (data collection + batching)
- [ ] Downlink task (command execution)
- [ ] Discovery handling
- [ ] Connection status tracking
- [ ] Queue management
- [ ] **[APPROVED]** Fix idle device cleanup timeout calculation (TickCount → milliseconds)
- [ ] **[APPROVED]** Add error handling when enqueue_uplink fails (retry/logging)
- [ ] **[APPROVED]** Support multiple BLE modules on different stacks

### 5.7 Config Handler Integration (Application Layer) ❌

- [ ] JSON config command parsing
- [ ] BLE config loading
- [ ] Module control command routing
- [ ] Discovery command handling

### 5.7 Config Handler Integration (Application Layer) ❌

- [ ] JSON config command parsing
- [ ] BLE config loading
- [ ] Module control command routing
- [ ] Discovery command handling

---

## 6. QUICK REFERENCE - Essential APIs & Patterns

### BLE Handler API
```c
// Config loading
esp_err_t ble_handler_load_config(uint8_t stack_id, const char *json_str, uint16_t len);

// Function execution
esp_err_t ble_execute_function(uint8_t stack_id, uint8_t function_id, const char *params, char *response, uint16_t resp_len);

// Frame parsing
esp_err_t ble_parse_frame(const uint8_t *data, uint16_t len, uint8_t *mac, uint8_t *payload, uint16_t *payload_len);

// Device management
esp_err_t ble_add_device(const uint8_t *mac_address);
esp_err_t ble_remove_device(const uint8_t *mac_address);
uint8_t ble_get_device_count(void);
```

### Config Handler Commands (WAN MCU)
```
BL:JSON:<len>:<json>          - Load JSON config
BL:DISC:<timeout>:<stack>     - Start discovery
BL:SETUP:<func>:<stack>:<params> - Execute setup
```

### Config Handler Commands (LAN MCU)
```
CFBL:JSON:<len>:<json>        - Parse & load config
CFBL:DISC:<timeout>:<stack>   - Execute discovery
CFBL:SETUP:<func>:<stack>:<params> - Execute setup
```

### Module Monitor API
```c
esp_err_t module_monitor_task_start(void);
esp_err_t module_monitor_load_config(uint8_t stack_id, const char *json_str, uint16_t len);
module_type_t module_monitor_get_stack_type(uint8_t stack_id);
esp_err_t module_monitor_start_handler(uint8_t stack_id);
esp_err_t module_monitor_stop_handler(uint8_t stack_id);
```

---

## 7. ESTIMATED EFFORT & TIMELINE (Coding Only)

### Hotfix Phase (Day 1 - Critical Bugs)
| Task | Complexity | Estimated Hours | Priority |
|------|------------|-----------------|----------|
| Hotfix 0.1: module_uart_comm UART port bug | Low | 0.5 | 🔴 CRITICAL |
| Hotfix 0.2: stack_handler GPIO comments | Low | 0.5 | 🔴 CRITICAL |
| Hotfix 0.3: ble_handler_task timeout calc | Medium | 1 | 🔴 CRITICAL |
| **Subtotal** | | **2 hours** | |

### Priority 1 Phase (Week 1-2)
| Task | Complexity | Estimated Hours | Notes |
|------|------------|-----------------|-------|
| **1.0: Module Monitor Task (NEW)** | **High** | **10-12** | **New component - lifecycle management** |
| 1.1: BLE Handler Middleware | High | 14-18 | +validation, +recovery, +binary support |
| 1.2: BLE Handler Task | High | 14-16 | +timeout fix, +error handling, +multi-stack |
| ~~1.3: Config Handler Integration~~ | ~~Medium~~ | ~~0~~ | **DEPRECATED - merged to 1.5** |
| 1.4: MCU_LAN_Handler Update | Low | 2-3 | Simplified - just forwarding |
| **1.5: Config Handler 3 Flows (NEW)** | **High** | **15-20** | **JSON/Discovery/Setup commands** |
| **Subtotal** | | **55-69 hours** | |

### Priority 2 Phase (Week 3)
| Task | Complexity | Estimated Hours | Notes |
|------|------------|-----------------|-------|
| 2.1: MCU_WAN_Handler Update | Medium | 6-8 | Standard scope |
| 2.2: Config Tool Update | Low | 4-6 | Optional - can defer |
| 2.3: Error Handling & Validation | Medium | 10-14 | +10 approved items to fix |
| **Subtotal** | | **20-28 hours** | |

**Total Estimated (Coding Only):** 75-97 hours  
**Reason:**
- +Task 1.0 (Module Monitor): +10-12 hours
- +Task 1.5 (Config Handler 3 Flows): +15-20 hours
- -Task 1.3 (deprecated): -6-8 hours
- -Task 1.4 (simplified): -2-3 hours

**Realistic Timeline:** 2.5-3 weeks (11-14 days working, 8-9 hours/day) for Priority 1 + 2

**Note:** Testing, documentation, and optimization are handled separately by user

---

## 8. DEPENDENCIES & QUICK START

**Execution Order:**
```
✅ BSP Layer (done)
   ↓
❌ Task 1.0: Module Monitor (foundation - start here)
   ↓
❌ Task 1.1: BLE Handler Middleware
   ↓
❌ Task 1.2: BLE Handler Task
   ↓
❌ Task 1.5: Config Handler 3 Flows
   ↓
❌ Task 1.4: MCU_LAN_Handler (simplified)
   ↓
❌ Task 2.1 & 2.3: Integration
```

**Hotfixes to do first (2h):**
- module_uart_comm.c:88 - Save uart_port in struct
- stack_handler.c - Fix GPIO comments
- ble_handler_task.c - Fix timeout calculation

---

## 9. COMMON PITFALLS & SOLUTIONS (Quick Reference)

```
✅ BSP Layer (UART/SPI/I2C/USB Communication)
   ↓
✅ BSP Layer (Stack Handler - GPIO)
   ↓
✅ Middleware Layer (JSON Config Parser)
   ↓
⚠️ Middleware Layer (Module Config Controller)
   ↓
❌ Application Layer (Module Monitor Task) ← TASK 1.0 (NEW - Foundation)
   ↓
❌ Middleware Layer (BLE Handler) ← TASK 1.1
   ↓
❌ Application Layer (BLE Handler Task) ← TASK 1.2
   ↓
❌ Application Layer (Config Handler - 3 Flows) ← TASK 1.5 (NEW - replaces 1.3)
   ↓
❌ Application Layer (MCU_LAN_Handler) ← TASK 1.4 (Simplified)
   ↓
❌ WAN MCU Layer (MCU_WAN_Handler) ← TASK 2.1
   ↓
✅ System Integration Complete
```

---

## 9. COMMON PITFALLS & SOLUTIONS (Quick Reference)

| Pitfall | Impact | Approved Solution |
|---------|--------|-------------------|
| GPIO pin confusion | Wrong GPIO control | Add clear mapping table in stack_handler.h + FIX comments |
| UART frame parsing failure | Lost device data | Implement robust frame validation (CRC/length) |
| Config persistence missing | Config lost on reboot | Implement NVS save/load in module_config_controller |
| Timeout handling error | Hanging tasks | Use portMAX_DELAY with timeout parameter |
| Device list memory leak | Memory exhaustion | Implement proper cleanup for idle devices |
| Missing error validation | Crashes/undefined behavior | Add ESP_RETURN_ON_ERROR checks everywhere |
| **[NEW]** UART port variable missing | Compile error | Save uart_port in module_uart_comm_s during init |
| **[NEW]** Timeout calculation wrong | Wrong idle device cleanup | Convert TickCount properly: `elapsed = (now - last_activity) / portTICK_PERIOD_MS` |
| **[NEW]** Command buffer overflow | Memory corruption | Add command string validation (length, format check) |
| **[NEW]** No connection recovery | Manual intervention needed | Implement retry logic + fallback to SW reset |
| **[NEW]** Data loss on queue full | Lost uplink data | Add error handling when enqueue_uplink fails (retry or logging) |
| **[NEW]** I2C hardcoded timeout | Timing issues | Make I2C timeout configurable from config struct |
| **[NEW]** SPI single transaction only | Latency issues | Add transaction queue support for multiple commands |
| **[NEW]** GPIO validation missing | Invalid GPIO writes | Validate pin ID format during JSON parsing |

---

## 10. QUICK START CHECKLIST

**Priority 1 (55-69 hours):**
- [ ] Hotfixes (2h)
- [ ] **Task 1.0: Module Monitor (10-12h)** ← Start here
- [ ] Task 1.1: BLE Handler Middleware (14-18h)
- [ ] Task 1.2: BLE Handler Task (14-16h)
- [ ] Task 1.5: Config Handler 3 Flows (15-20h) - See UPDATE_CONFIG_HANDLER.md
- [ ] Task 1.4: MCU_LAN_Handler (2-3h)

**Priority 2 (20-28 hours):**
- [ ] Task 2.1: MCU_WAN_Handler (6-8h)
- [ ] Task 2.3: Error Handling & Validation (10-14h)
- [ ] Task 2.2: Config Tool (4-6h, optional)

**Total:** 75-97 hours, 2.5-3 weeks

---

**End of Document**

_Created: 2026-02-08 | Purpose: Vibe coding reference (Priority 1+2 only)_
