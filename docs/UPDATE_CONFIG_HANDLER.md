# UPDATE CONFIG HANDLER - Module Base Setting Commands

**Ngày tạo:** 2026-02-08  
**Scope:** Command format và updates cho 3 luồng qua config_handler  
**Target MCUs:** DA2_esp (WAN MCU) và DA2_esp_LAN (LAN MCU)

---

## 1. TỔNG QUAN 3 LUỒNG CẦN CẬP NHẬT

Theo TODO.md, có 3 luồng dữ liệu cần đi qua config_handler:

| Luồng | Tên | Hướng | Vai trò Config Handler |
|-------|-----|-------|------------------------|
| **1** | Config Module | App → WAN → LAN | Parse JSON config string và route tới BLE handler |
| **4** | Discovery Device | App ⇄ WAN ⇄ LAN | Parse discovery command, forward tới BLE task, trả kết quả |
| **5** | Setup Commands | App ⇄ WAN ⇄ LAN | Parse setup command (reset/name/RF), execute via BLE handler |

**Luồng 2 & 3 KHÔNG qua config_handler:**
- Luồng 2 (Sensor Data): Trực tiếp từ BLE task → MCU_WAN → Server
- Luồng 3 (Module Control): Trực tiếp từ Server → MCU_WAN → BLE task

---

## 2. KIẾN TRÚC HIỆN TẠI

### 2.1 WAN MCU Config Handler

**File:** `/DA2_esp/Application/Config_Handler/src/config_handler.c`

**Command Types hiện tại:**
```c
typedef enum {
    CONFIG_TYPE_WIFI = 0,       // "WF" - WiFi configuration
    CONFIG_TYPE_MQTT = 1,       // "MQ" - MQTT configuration
    CONFIG_TYPE_LTE = 2,        // "LT" - LTE configuration
    CONFIG_TYPE_INTERNET = 3,   // "IN" - Internet configuration
    CONFIG_UPDATE_FIRMWARE = 4, // "FW" - Firmware update
    CONFIG_TYPE_MCU_LAN = 5,    // "ML" - MCU LAN configuration
    CONFIG_TYPE_SERVER = 6,     // "SV" - Server configuration
    CONFIG_TYPE_UNKNOWN = 0xFF
} config_type_t;
```

**Command Format Pattern:**
```
<2-char-prefix>:<parameters>
```

**Existing Parse Functions:**
- `config_parse_wifi()` - Format: `WF:SSID:PASSWORD:AUTH_MODE`
- `config_parse_mqtt()` - Format: `MQ:BROKER:TOKEN:TOPIC`
- `config_parse_lte()` - Format: `LT:COMM_TYPE:APN:USERNAME:PASSWORD`

### 2.2 LAN MCU Config Handler

**File:** `/DA2_esp_LAN/Application/Config_Handler/src/config_handler.c`

**Command Types hiện tại:**
```c
typedef enum {
  CONFIG_UPDATE_FIRMWARE = 0,  // "CFFW" - Firmware update
  CONFIG_UPDATE_LORA = 1,      // "CFLR" - LoRa config
  CONFIG_UPDATE_CAN = 2,       // "CFCB"/"CFCM"/"CFCW" - CAN config
  CONFIG_UPDATE_SCAN = 3,      // "CFSC" - Config query
  CONFIG_UPDATE_STACK = 4,     // "CFST" - Stack config
  CONFIG_UPDATE_RS485 = 5,     // "CFRS" - RS485 config
  CONFIG_TYPE_UNKNOWN = 0xFF
} config_type_t;
```

**Command Format Pattern:**
```
CF<2-char-type>:<parameters>
```

**Existing Parse Functions:**
- `config_parse_lora()` - Format: `CFLR:MODEM:<6-bytes>` hoặc `CFLR:HDLCF:<11-bytes>`
- `config_parse_can()` - Format: `CFCB:<baud>:<mode>:...`
- `config_parse_stack_type()` - Format: `CFST:<stack_id>:<type>`

---

## 3. THIẾT KẾ COMMAND MỚI CHO MODULE BASE SETTING

### 3.1 Prefix Convention

**Quy ước:**
- WAN MCU: 2-character prefix (existing pattern)
- LAN MCU: `CF` + 2-character type (existing pattern)
- Module-specific: `BL` (BLE), `ZB` (Zigbee), `LR` (LoRa - existing)

**New Command Prefixes:**

| Prefix | MCU | Purpose | Format |
|--------|-----|---------|--------|
| `BL` | WAN | BLE module commands (forward to LAN) | `BL:<subcommand>` |
| `CFBL` | LAN | BLE config/control (local processing) | `CFBL:<subcommand>` |

---

## 4. LUỒNG 1: CONFIG MODULE (JSON Config)

### 4.1 Yêu Cầu

**Mô tả:** Load JSON config cho BLE module từ App → WAN → LAN → BLE Handler

**Data Flow:**
```
App (PC/Mobile)
   ↓ (UART/USB)
WAN MCU - config_handler
   ↓ (parse "BL:JSON:<len>:<json_data>")
   ↓ (route qua SPI to LAN via mcu_lan_handler)
LAN MCU - mcu_wan_handler
   ↓ (detect JSON config packet)
LAN MCU - config_handler
   ↓ (parse "CFBL:JSON:<len>:<json_data>")
   ↓ (call ble_handler_load_config())
BLE Handler Middleware
   ↓ (parse JSON via json_ble_config_parse())
   ↓ (initialize communication port)
   ✓ Config loaded, ready to operate
```

### 4.2 Command Format

#### 4.2.1 WAN MCU Command (từ App)

**Format:**
```
BL:JSON:<json_length>:<json_data>
```

**Components:**
- `BL` - BLE module command prefix
- `JSON` - Subcommand for JSON config
- `<json_length>` - Length of JSON string (decimal, e.g., "0512" for 512 bytes)
- `<json_data>` - Raw JSON string (ASCII)

**Example:**
```
BL:JSON:0245:{"module_id":"001","module_type":"BLE",...}
```

**Length Constraint:**
- Max JSON length: 2048 bytes (CONFIG_CMD_MAX_LEN can be extended)
- Min JSON length: 50 bytes (basic config)

#### 4.2.2 LAN MCU Command (forwarded từ WAN)

**Format:**
```
CFBL:JSON:<json_length>:<json_data>
```

**Components:** Same as WAN, with `CFBL` prefix

**Processing:**
1. Parse command type → `CONFIG_UPDATE_BLE_JSON`
2. Extract JSON length and data
3. Call `json_ble_config_parse(json_data, &parsed_config)`
4. Call `ble_handler_load_config(stack_id, json_data, json_length)`
5. Store to NVS (optional, for persistence)
6. Send ACK back to WAN

### 4.3 Implementation Tasks

#### WAN MCU Updates

**File:** `DA2_esp/Application/Config_Handler/include/config_handler.h`

Add new enum:
```c
typedef enum {
    CONFIG_TYPE_WIFI = 0,
    // ... existing ...
    CONFIG_TYPE_BLE = 7,        // "BL" - BLE module command (NEW)
    CONFIG_TYPE_UNKNOWN = 0xFF
} config_type_t;
```

**File:** `DA2_esp/Application/Config_Handler/src/config_handler.c`

Add parse function:
```c
/**
 * @brief Parse BLE command and forward to LAN MCU
 * Format: "BL:JSON:<len>:<json_data>"
 */
static esp_err_t config_parse_ble(const char *data, uint16_t len) {
    if (!data || len < 10) { // "BL:JSON:XX:"
        ESP_LOGE(TAG, "BLE config: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check prefix "BL:JSON:"
    if (strncmp(data, "BL:JSON:", 8) != 0) {
        ESP_LOGE(TAG, "BLE config: invalid prefix");
        return ESP_FAIL;
    }
    
    // Extract JSON length
    const char *len_str = data + 8;
    const char *colon = strchr(len_str, ':');
    if (!colon) {
        ESP_LOGE(TAG, "BLE config: missing length separator");
        return ESP_FAIL;
    }
    
    char len_buf[8] = {0};
    int len_digits = colon - len_str;
    if (len_digits <= 0 || len_digits >= sizeof(len_buf)) {
        ESP_LOGE(TAG, "BLE config: invalid length format");
        return ESP_FAIL;
    }
    memcpy(len_buf, len_str, len_digits);
    uint16_t json_len = atoi(len_buf);
    
    // Extract JSON data
    const char *json_data = colon + 1;
    if (json_len <= 0 || json_len > 2048) {
        ESP_LOGE(TAG, "BLE config: JSON length out of range: %u", json_len);
        return ESP_FAIL;
    }
    
    // Verify actual data length
    uint16_t actual_len = len - (json_data - data);
    if (actual_len < json_len) {
        ESP_LOGE(TAG, "BLE config: incomplete JSON data (expected %u, got %u)", 
                 json_len, actual_len);
        return ESP_FAIL;
    }
    
    // Forward to LAN MCU via mcu_lan_handler
    // Format: "CFBL:JSON:<len>:<json_data>"
    uint8_t forward_buf[2048 + 20]; // Header + JSON
    int forward_len = snprintf((char*)forward_buf, sizeof(forward_buf),
                               "CFBL:JSON:%04u:%.*s", 
                               json_len, json_len, json_data);
    
    if (forward_len < 0 || forward_len >= sizeof(forward_buf)) {
        ESP_LOGE(TAG, "BLE config: forward buffer overflow");
        return ESP_FAIL;
    }
    
    // Send to LAN MCU
    esp_err_t ret = mcu_lan_send_config(forward_buf, forward_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to forward BLE config to LAN MCU");
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE JSON config forwarded to LAN MCU (%u bytes)", json_len);
    return ESP_OK;
}
```

Update `config_parse_type()`:
```c
config_type_t config_parse_type(const char *cmd, uint16_t len) {
    if (len < CONFIG_CMD_PREFIX_LEN) {
        return CONFIG_TYPE_UNKNOWN;
    }
    
    // ... existing checks ...
    
    // NEW: BLE command
    if (cmd[0] == 'B' && cmd[1] == 'L') {
        return CONFIG_TYPE_BLE;
    }
    
    return CONFIG_TYPE_UNKNOWN;
}
```

Update `config_handler_task()`:
```c
case CONFIG_TYPE_BLE: {
    if (config_parse_ble(cmd.raw_data, cmd.data_len) == ESP_OK) {
        ESP_LOGI(TAG, "BLE config forwarded successfully");
    } else {
        ESP_LOGE(TAG, "Failed to parse BLE config");
    }
    break;
}
```

#### LAN MCU Updates

**File:** `DA2_esp_LAN/Application/Config_Handler/include/config_handler.h`

Add new enum:
```c
typedef enum {
  CONFIG_UPDATE_FIRMWARE = 0,
  // ... existing ...
  CONFIG_UPDATE_BLE_JSON = 6,    // "CFBL:JSON" - BLE JSON config (NEW)
  CONFIG_UPDATE_BLE_SETUP = 7,   // "CFBL:SETUP" - BLE setup command (NEW)
  CONFIG_UPDATE_BLE_DISCOVERY = 8, // "CFBL:DISC" - BLE discovery (NEW)
  CONFIG_TYPE_UNKNOWN = 0xFF
} config_type_t;
```

**File:** `DA2_esp_LAN/Application/Config_Handler/src/config_handler.c`

Add parse function:
```c
/**
 * @brief Parse BLE JSON config
 * Format: "CFBL:JSON:<len>:<json_data>"
 */
static esp_err_t config_parse_ble_json(const uint8_t *data, uint16_t len) {
    if (!data || len < 14) { // "CFBL:JSON:XXXX:"
        ESP_LOGE(TAG, "BLE JSON: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check prefix
    if (strncmp((const char*)data, "CFBL:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "BLE JSON: invalid prefix");
        return ESP_FAIL;
    }
    
    // Extract JSON length
    const char *len_str = (const char*)(data + 10);
    const char *colon = strchr(len_str, ':');
    if (!colon) {
        ESP_LOGE(TAG, "BLE JSON: missing separator");
        return ESP_FAIL;
    }
    
    char len_buf[8] = {0};
    int len_digits = colon - len_str;
    if (len_digits <= 0 || len_digits >= sizeof(len_buf)) {
        ESP_LOGE(TAG, "BLE JSON: invalid length format");
        return ESP_FAIL;
    }
    memcpy(len_buf, len_str, len_digits);
    uint16_t json_len = atoi(len_buf);
    
    // Extract JSON data
    const char *json_data = colon + 1;
    if (json_len <= 0 || json_len > 2048) {
        ESP_LOGE(TAG, "BLE JSON: length out of range: %u", json_len);
        return ESP_FAIL;
    }
    
    // Verify actual length
    uint16_t actual_len = len - (json_data - (const char*)data);
    if (actual_len < json_len) {
        ESP_LOGE(TAG, "BLE JSON: incomplete data (expected %u, got %u)",
                 json_len, actual_len);
        return ESP_FAIL;
    }
    
    // Parse JSON and load to BLE handler
    // Assume stack_id = 0 (can be extended to support multiple stacks)
    uint8_t stack_id = 0;
    
    esp_err_t ret = ble_handler_load_config(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load BLE config to handler: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE JSON config loaded successfully (%u bytes)", json_len);
    
    // Optional: Save to NVS for persistence
    // esp_err_t nvs_ret = config_save_ble_json_to_nvs(stack_id, json_data, json_len);
    
    return ESP_OK;
}
```

Update `config_parse_type()`:
```c
config_type_t config_parse_type(const char *cmd, uint16_t len) {
  if (len < 4 || cmd[0] != 'C' || cmd[1] != 'F') {
    return CONFIG_TYPE_UNKNOWN;
  }

  // ... existing checks ...
  
  // NEW: BLE commands
  if (cmd[2] == 'B' && cmd[3] == 'L') {
    // Check subcommand
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_BLE_JSON;
    } else if (len >= 11 && strncmp(cmd + 5, "SETUP:", 6) == 0) {
      return CONFIG_UPDATE_BLE_SETUP;
    } else if (len >= 10 && strncmp(cmd + 5, "DISC:", 5) == 0) {
      return CONFIG_UPDATE_BLE_DISCOVERY;
    }
  }
  
  return CONFIG_TYPE_UNKNOWN;
}
```

Update `config_handler_task()`:
```c
case CONFIG_UPDATE_BLE_JSON: {
  if (config_parse_ble_json((const uint8_t*)cmd.raw_data, cmd.data_len) == ESP_OK) {
    ESP_LOGI(TAG, "BLE JSON config updated from WAN MCU");
  } else {
    ESP_LOGE(TAG, "Failed to parse BLE JSON config");
  }
  break;
}
```

---

## 5. LUỒNG 4: DISCOVERY DEVICE (Scan/Discovery)

### 5.1 Yêu Cầu

**Mô tả:** Scan và discover BLE devices từ App → WAN → LAN → BLE Module → trả kết quả về

**Data Flow:**
```
App (Request)
   ↓ "BL:DISC:<timeout>"
WAN MCU - config_handler
   ↓ (forward "CFBL:DISC:<timeout>")
LAN MCU - config_handler
   ↓ (call ble_handler_task_start_discovery(timeout))
BLE Handler Task
   ↓ (execute MODULE_START_DISCOVERY)
   ↓ (collect devices: MAC + RSSI)
   ↓ (return via uplink queue)
LAN MCU - mcu_wan_handler
   ↓ (send discovery result via SPI)
WAN MCU - uart_handler
   ↓ (forward to App)
App (Display Results)
```

### 5.2 Command Format

#### 5.2.1 Discovery Request (App → WAN → LAN)

**Format:**
```
BL:DISC:<timeout_ms>:<stack_id>
```

**Components:**
- `BL:DISC` - Discovery command
- `<timeout_ms>` - Discovery timeout in milliseconds (e.g., "5000" for 5 seconds)
- `<stack_id>` - Target stack (0 or 1, optional, default 0)

**Example:**
```
BL:DISC:5000:0    # Scan for 5 seconds on stack 0
BL:DISC:10000:1   # Scan for 10 seconds on stack 1
```

**LAN MCU Format (forwarded):**
```
CFBL:DISC:<timeout_ms>:<stack_id>
```

#### 5.2.2 Discovery Response (LAN → WAN → App)

**Format:**
```
BL:DISC:RESULT:<device_count>:<device_list>
```

**Device List Format:**
```
<MAC1>:<RSSI1>,<MAC2>:<RSSI2>,...
```

**MAC Format:** 12 hex characters (e.g., `AABBCCDDEEFF`)
**RSSI Format:** Signed decimal (e.g., `-45`)

**Example:**
```
BL:DISC:RESULT:3:AABBCCDDEEFF:-45,112233445566:-67,778899AABBCC:-52
```

**Max Devices:** 20 (MAX_DISCOVERED_DEVICES)

### 5.3 Implementation Tasks

#### WAN MCU - Discovery Request Handler

Add to `config_handler.c`:
```c
/**
 * @brief Parse BLE discovery command
 * Format: "BL:DISC:<timeout>:<stack_id>"
 */
static esp_err_t config_parse_ble_discovery(const char *data, uint16_t len) {
    if (!data || len < 11) { // "BL:DISC:XXX"
        ESP_LOGE(TAG, "BLE discovery: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check prefix
    if (strncmp(data, "BL:DISC:", 8) != 0) {
        ESP_LOGE(TAG, "BLE discovery: invalid prefix");
        return ESP_FAIL;
    }
    
    // Parse timeout
    const char *timeout_str = data + 8;
    const char *colon = strchr(timeout_str, ':');
    
    uint16_t timeout_ms = 5000; // Default 5 seconds
    uint8_t stack_id = 0;       // Default stack 0
    
    if (colon) {
        // Has stack_id
        char timeout_buf[8] = {0};
        int len_digits = colon - timeout_str;
        if (len_digits > 0 && len_digits < sizeof(timeout_buf)) {
            memcpy(timeout_buf, timeout_str, len_digits);
            timeout_ms = atoi(timeout_buf);
        }
        
        // Parse stack_id
        stack_id = atoi(colon + 1);
    } else {
        // No stack_id, parse timeout only
        timeout_ms = atoi(timeout_str);
    }
    
    // Validate
    if (timeout_ms < 1000 || timeout_ms > 60000) {
        ESP_LOGW(TAG, "BLE discovery: timeout out of range, using default");
        timeout_ms = 5000;
    }
    
    if (stack_id > 1) {
        ESP_LOGE(TAG, "BLE discovery: invalid stack_id: %u", stack_id);
        return ESP_FAIL;
    }
    
    // Forward to LAN MCU
    uint8_t forward_buf[32];
    int forward_len = snprintf((char*)forward_buf, sizeof(forward_buf),
                               "CFBL:DISC:%u:%u", timeout_ms, stack_id);
    
    esp_err_t ret = mcu_lan_send_config(forward_buf, forward_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to forward discovery command to LAN MCU");
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE discovery command forwarded (timeout=%u, stack=%u)",
             timeout_ms, stack_id);
    return ESP_OK;
}
```

#### LAN MCU - Discovery Execution

Add to `config_handler.c`:
```c
/**
 * @brief Parse BLE discovery command and execute
 * Format: "CFBL:DISC:<timeout>:<stack_id>"
 */
static esp_err_t config_parse_ble_discovery(const uint8_t *data, uint16_t len) {
    if (!data || len < 13) { // "CFBL:DISC:XXX"
        ESP_LOGE(TAG, "BLE discovery: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check prefix
    if (strncmp((const char*)data, "CFBL:DISC:", 10) != 0) {
        ESP_LOGE(TAG, "BLE discovery: invalid prefix");
        return ESP_FAIL;
    }
    
    // Parse timeout and stack_id
    const char *timeout_str = (const char*)(data + 10);
    const char *colon = strchr(timeout_str, ':');
    
    uint16_t timeout_ms = 5000;
    uint8_t stack_id = 0;
    
    if (colon) {
        char timeout_buf[8] = {0};
        int len_digits = colon - timeout_str;
        if (len_digits > 0 && len_digits < sizeof(timeout_buf)) {
            memcpy(timeout_buf, timeout_str, len_digits);
            timeout_ms = atoi(timeout_buf);
        }
        stack_id = atoi(colon + 1);
    } else {
        timeout_ms = atoi(timeout_str);
    }
    
    ESP_LOGI(TAG, "Starting BLE discovery (timeout=%u ms, stack=%u)",
             timeout_ms, stack_id);
    
    // Call BLE handler task to start discovery
    esp_err_t ret = ble_handler_task_start_discovery(stack_id, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE discovery: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for discovery to complete (blocking with timeout)
    vTaskDelay(pdMS_TO_TICKS(timeout_ms + 500)); // Add 500ms margin
    
    // Get discovered devices
    ble_device_info_t devices[20];
    uint8_t device_count = 0;
    ret = ble_handler_task_get_discovered_devices(stack_id, devices, 
                                                   20, &device_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get discovered devices");
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE discovery completed: %u devices found", device_count);
    
    // Format response: "BL:DISC:RESULT:<count>:<device_list>"
    char response[512];
    int pos = snprintf(response, sizeof(response), 
                       "BL:DISC:RESULT:%u:", device_count);
    
    for (uint8_t i = 0; i < device_count && pos < sizeof(response) - 20; i++) {
        pos += snprintf(response + pos, sizeof(response) - pos,
                        "%02X%02X%02X%02X%02X%02X:%d",
                        devices[i].mac_address[0], devices[i].mac_address[1],
                        devices[i].mac_address[2], devices[i].mac_address[3],
                        devices[i].mac_address[4], devices[i].mac_address[5],
                        devices[i].rssi);
        
        if (i < device_count - 1) {
            response[pos++] = ',';
        }
    }
    
    // Send response back to WAN MCU
    ret = mcu_wan_send_uplink((uint8_t*)response, pos);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send discovery result to WAN MCU");
        return ret;
    }
    
    return ESP_OK;
}
```

Update task handler:
```c
case CONFIG_UPDATE_BLE_DISCOVERY: {
  if (config_parse_ble_discovery((const uint8_t*)cmd.raw_data, 
                                  cmd.data_len) == ESP_OK) {
    ESP_LOGI(TAG, "BLE discovery completed");
  } else {
    ESP_LOGE(TAG, "Failed to execute BLE discovery");
  }
  break;
}
```

---

## 6. LUỒNG 5: SETUP COMMANDS (Reset, Set Name, Set RF Params)

### 6.1 Yêu Cầu

**Mô tả:** Execute BLE setup commands (20 functions) từ App

**Data Flow:**
```
App (Setup Command)
   ↓ "BL:SETUP:<function_id>:<params>"
WAN MCU - config_handler
   ↓ (forward "CFBL:SETUP:<function_id>:<params>")
LAN MCU - config_handler
   ↓ (parse and validate)
   ↓ (call ble_handler_execute_function())
BLE Handler Middleware
   ↓ (execute function: GPIO + AT command + wait response)
   ↓ (return result)
LAN MCU - mcu_wan_handler
   ↓ (send result via SPI)
WAN MCU - uart_handler
   ↓ (forward to App)
App (Display Result)
```

### 6.2 Command Format

#### 6.2.1 Setup Command Request

**Format:**
```
BL:SETUP:<function_id>:<stack_id>:<params>
```

**Components:**
- `BL:SETUP` - Setup command prefix
- `<function_id>` - Function ID (0-19, see BLE_FUNC_* enum)
- `<stack_id>` - Target stack (0 or 1)
- `<params>` - Optional parameters (function-specific, can be empty)

**Function ID Mapping:**
```
0  = HW_RESET
1  = SW_RESET
2  = FACTORY_RESET
3  = GET_INFO
4  = SET_NAME
5  = SET_COMM_CONFIG
6  = SET_RF_PARAMS
7  = ENTER_CMD_MODE
8  = ENTER_DATA_MODE
9  = START_BROADCAST
10 = CONNECT
11 = DISCONNECT
12 = GET_CONNECTION_STATUS
13 = ENTER_SLEEP
14 = WAKEUP
15 = START_DISCOVERY
16 = SEND_DATA
17 = GET_DIAGNOSTICS
18 = SET_SECURITY
19 = MANAGE_WHITELIST
```

**Examples:**
```
BL:SETUP:0:0:               # HW Reset on stack 0 (no params)
BL:SETUP:1:0:               # SW Reset on stack 0
BL:SETUP:4:0:MyDevice       # Set Name to "MyDevice" on stack 0
BL:SETUP:6:1:TX_POWER=4     # Set RF Params on stack 1
BL:SETUP:10:0:AABBCCDDEEFF  # Connect to device MAC on stack 0
```

**LAN MCU Format (forwarded):**
```
CFBL:SETUP:<function_id>:<stack_id>:<params>
```

#### 6.2.2 Setup Command Response

**Format:**
```
BL:SETUP:RESULT:<function_id>:<status>:<response_data>
```

**Components:**
- `<status>` - `OK` or `FAIL`
- `<response_data>` - Response from module (can be empty)

**Examples:**
```
BL:SETUP:RESULT:0:OK:                    # HW Reset successful
BL:SETUP:RESULT:3:OK:JDY-23-v2.1         # GET_INFO returned version
BL:SETUP:RESULT:10:FAIL:TIMEOUT          # CONNECT failed with timeout
```

### 6.3 Implementation Tasks

#### WAN MCU - Setup Command Handler

Add to `config_handler.c`:
```c
/**
 * @brief Parse BLE setup command
 * Format: "BL:SETUP:<function_id>:<stack_id>:<params>"
 */
static esp_err_t config_parse_ble_setup(const char *data, uint16_t len) {
    if (!data || len < 12) { // "BL:SETUP:X:Y"
        ESP_LOGE(TAG, "BLE setup: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check prefix
    if (strncmp(data, "BL:SETUP:", 9) != 0) {
        ESP_LOGE(TAG, "BLE setup: invalid prefix");
        return ESP_FAIL;
    }
    
    // Parse function_id
    const char *func_str = data + 9;
    const char *colon1 = strchr(func_str, ':');
    if (!colon1) {
        ESP_LOGE(TAG, "BLE setup: missing function_id separator");
        return ESP_FAIL;
    }
    
    char func_buf[4] = {0};
    int func_len = colon1 - func_str;
    if (func_len <= 0 || func_len >= sizeof(func_buf)) {
        ESP_LOGE(TAG, "BLE setup: invalid function_id format");
        return ESP_FAIL;
    }
    memcpy(func_buf, func_str, func_len);
    uint8_t function_id = atoi(func_buf);
    
    // Parse stack_id
    const char *stack_str = colon1 + 1;
    const char *colon2 = strchr(stack_str, ':');
    if (!colon2) {
        ESP_LOGE(TAG, "BLE setup: missing stack_id separator");
        return ESP_FAIL;
    }
    
    uint8_t stack_id = atoi(stack_str);
    
    // Extract params (can be empty)
    const char *params = colon2 + 1;
    uint16_t params_len = len - (params - data);
    
    // Validate
    if (function_id > 19) {
        ESP_LOGE(TAG, "BLE setup: invalid function_id: %u", function_id);
        return ESP_FAIL;
    }
    
    if (stack_id > 1) {
        ESP_LOGE(TAG, "BLE setup: invalid stack_id: %u", stack_id);
        return ESP_FAIL;
    }
    
    // Forward to LAN MCU
    uint8_t forward_buf[256];
    int forward_len = snprintf((char*)forward_buf, sizeof(forward_buf),
                               "CFBL:SETUP:%u:%u:%.*s",
                               function_id, stack_id, params_len, params);
    
    if (forward_len < 0 || forward_len >= sizeof(forward_buf)) {
        ESP_LOGE(TAG, "BLE setup: forward buffer overflow");
        return ESP_FAIL;
    }
    
    esp_err_t ret = mcu_lan_send_config(forward_buf, forward_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to forward BLE setup command to LAN MCU");
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE setup command forwarded (func=%u, stack=%u)",
             function_id, stack_id);
    return ESP_OK;
}
```

Update parse_type and task handler similar to Discovery section.

#### LAN MCU - Setup Command Execution

Add to `config_handler.c`:
```c
/**
 * @brief Parse BLE setup command and execute
 * Format: "CFBL:SETUP:<function_id>:<stack_id>:<params>"
 */
static esp_err_t config_parse_ble_setup(const uint8_t *data, uint16_t len) {
    if (!data || len < 14) { // "CFBL:SETUP:X:Y"
        ESP_LOGE(TAG, "BLE setup: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check prefix
    if (strncmp((const char*)data, "CFBL:SETUP:", 11) != 0) {
        ESP_LOGE(TAG, "BLE setup: invalid prefix");
        return ESP_FAIL;
    }
    
    // Parse function_id
    const char *func_str = (const char*)(data + 11);
    const char *colon1 = strchr(func_str, ':');
    if (!colon1) {
        ESP_LOGE(TAG, "BLE setup: missing function_id separator");
        return ESP_FAIL;
    }
    
    char func_buf[4] = {0};
    int func_len = colon1 - func_str;
    if (func_len <= 0 || func_len >= sizeof(func_buf)) {
        ESP_LOGE(TAG, "BLE setup: invalid function_id");
        return ESP_FAIL;
    }
    memcpy(func_buf, func_str, func_len);
    uint8_t function_id = atoi(func_buf);
    
    // Parse stack_id
    const char *stack_str = colon1 + 1;
    const char *colon2 = strchr(stack_str, ':');
    if (!colon2) {
        ESP_LOGE(TAG, "BLE setup: missing stack_id separator");
        return ESP_FAIL;
    }
    
    uint8_t stack_id = atoi(stack_str);
    
    // Extract params
    const char *params = colon2 + 1;
    uint16_t params_len = len - (params - (const char*)data);
    
    ESP_LOGI(TAG, "Executing BLE setup (func=%u, stack=%u, params_len=%u)",
             function_id, stack_id, params_len);
    
    // Execute function via BLE handler
    char response[256] = {0};
    esp_err_t ret = ble_handler_execute_function(stack_id, function_id, 
                                                  params, response, 
                                                  sizeof(response));
    
    // Format result
    const char *status = (ret == ESP_OK) ? "OK" : "FAIL";
    char result_buf[512];
    int result_len = snprintf(result_buf, sizeof(result_buf),
                              "BL:SETUP:RESULT:%u:%s:%s",
                              function_id, status, response);
    
    if (result_len < 0 || result_len >= sizeof(result_buf)) {
        ESP_LOGE(TAG, "BLE setup: result buffer overflow");
        return ESP_FAIL;
    }
    
    // Send result back to WAN MCU
    ret = mcu_wan_send_uplink((uint8_t*)result_buf, result_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send BLE setup result to WAN MCU");
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE setup completed (func=%u, status=%s)",
             function_id, status);
    return ESP_OK;
}
```

---

## 7. SUMMARY OF CHANGES

### 7.1 WAN MCU Changes

**File: `/DA2_esp/Application/Config_Handler/include/config_handler.h`**
- Add `CONFIG_TYPE_BLE = 7` to enum

**File: `/DA2_esp/Application/Config_Handler/src/config_handler.c`**
- Add `config_parse_ble()` - Forward JSON config
- Add `config_parse_ble_discovery()` - Forward discovery command
- Add `config_parse_ble_setup()` - Forward setup command
- Update `config_parse_type()` - Recognize "BL" prefix
- Update `config_handler_task()` - Handle `CONFIG_TYPE_BLE` case

### 7.2 LAN MCU Changes

**File: `/DA2_esp_LAN/Application/Config_Handler/include/config_handler.h`**
- Add 3 new enums:
  - `CONFIG_UPDATE_BLE_JSON = 6`
  - `CONFIG_UPDATE_BLE_SETUP = 7`
  - `CONFIG_UPDATE_BLE_DISCOVERY = 8`

**File: `/DA2_esp_LAN/Application/Config_Handler/src/config_handler.c`**
- Add `config_parse_ble_json()` - Parse and load JSON config
- Add `config_parse_ble_discovery()` - Execute discovery and return results
- Add `config_parse_ble_setup()` - Execute setup function and return result
- Update `config_parse_type()` - Recognize "CFBL" prefix with subcommands
- Update `config_handler_task()` - Handle 3 new config types

### 7.3 Integration Points

**Required Interfaces:**
- `ble_handler_load_config(stack_id, json_str, len)` - From BLE Handler Middleware
- `ble_handler_execute_function(stack_id, func_id, params, response, resp_len)` - From BLE Handler
- `ble_handler_task_start_discovery(stack_id, timeout_ms)` - From BLE Handler Task
- `ble_handler_task_get_discovered_devices(stack_id, devices[], max, count)` - From BLE Handler Task
- `mcu_lan_send_config(data, len)` - From MCU_LAN_Handler (WAN MCU)
- `mcu_wan_send_uplink(data, len)` - From MCU_WAN_Handler (LAN MCU)

---

## 8. TESTING PROCEDURES

### 8.1 Test Luồng 1: JSON Config

**Test Case 1.1: Basic JSON Config**
```
Input (App → WAN):
BL:JSON:0245:{"module_id":"001","module_type":"BLE","module_name":"JDY-23","module_communication":{"port_type":"uart","parameters":{"baudrate":9600,"parity":"none","stopbit":1}},"functions":[...]}

Expected:
- WAN MCU logs: "BLE JSON config forwarded to LAN MCU"
- LAN MCU logs: "BLE JSON config loaded successfully (245 bytes)"
- BLE handler logs: "Config loaded for stack 0"
```

**Test Case 1.2: Invalid JSON**
```
Input:
BL:JSON:0050:{invalid json}

Expected:
- LAN MCU logs: "Failed to parse JSON: ESP_ERR_INVALID_ARG"
```

### 8.2 Test Luồng 4: Discovery

**Test Case 4.1: Successful Discovery**
```
Input:
BL:DISC:5000:0

Expected Output:
BL:DISC:RESULT:3:AABBCCDDEEFF:-45,112233445566:-67,778899AABBCC:-52
```

**Test Case 4.2: No Devices Found**
```
Input:
BL:DISC:3000:0

Expected Output:
BL:DISC:RESULT:0:
```

### 8.3 Test Luồng 5: Setup Commands

**Test Case 5.1: HW Reset**
```
Input:
BL:SETUP:0:0:

Expected Output:
BL:SETUP:RESULT:0:OK:
```

**Test Case 5.2: Set Name**
```
Input:
BL:SETUP:4:0:MyGateway

Expected Output:
BL:SETUP:RESULT:4:OK:
```

**Test Case 5.3: Connect to Device**
```
Input:
BL:SETUP:10:0:AABBCCDDEEFF

Expected Output:
BL:SETUP:RESULT:10:OK:CONNECTED
or
BL:SETUP:RESULT:10:FAIL:TIMEOUT
```

---

## 9. ERROR HANDLING

### 9.1 Common Errors

| Error | Cause | Handler Response |
|-------|-------|------------------|
| `ESP_ERR_INVALID_ARG` | Malformed command | Log error, drop packet |
| `ESP_ERR_TIMEOUT` | BLE module timeout | Return `FAIL:TIMEOUT` |
| `ESP_ERR_NOT_FOUND` | Function not configured | Return `FAIL:NOT_CONFIGURED` |
| `ESP_FAIL` | General execution failure | Return `FAIL:ERROR` |

### 9.2 Recovery Mechanisms

- **Retry Logic:** WAN MCU can retry sending command up to 3 times
- **Timeout Handling:** All operations have max timeout (5s for discovery, 2s for setup)
- **Fallback:** If HW reset fails, try SW reset automatically

---

## 10. DEPENDENCIES & TIMELINE

### 10.1 Prerequisites

Before implementing these config_handler updates, ensure:
- ✅ Task 1.1 (BLE Handler Middleware) is complete
- ✅ Task 1.2 (BLE Handler Task) is complete
- ✅ JSON parser working (json_ble_config_parse())
- ✅ MCU_LAN/MCU_WAN communication functional

### 10.2 Implementation Timeline

| Task | Estimated Hours | Priority |
|------|-----------------|----------|
| WAN MCU: Add BLE command types | 1h | 🔴 Critical |
| WAN MCU: Implement 3 parse functions | 3-4h | 🔴 Critical |
| LAN MCU: Add BLE command types | 1h | 🔴 Critical |
| LAN MCU: Implement 3 parse functions | 4-5h | 🔴 Critical |
| Integration testing (3 luồng) | 3-4h | 🔴 Critical |
| Error handling & validation | 2h | 🟡 High |
| Documentation & examples | 1h | 🟢 Medium |
| **Total** | **15-20 hours** | |

---

## 11. APPENDIX

### 11.1 Complete Command Reference

| Command | Format | Direction | Description |
|---------|--------|-----------|-------------|
| JSON Config | `BL:JSON:<len>:<json>` | App→WAN→LAN | Load module config |
| Discovery | `BL:DISC:<timeout>:<stack>` | App→WAN→LAN | Scan devices |
| Setup | `BL:SETUP:<func>:<stack>:<params>` | App→WAN→LAN | Execute function |
| Discovery Result | `BL:DISC:RESULT:<count>:<list>` | LAN→WAN→App | Return scan results |
| Setup Result | `BL:SETUP:RESULT:<func>:<status>:<data>` | LAN→WAN→App | Return execution result |

### 11.2 Function ID to Name Mapping

```c
static const char *BLE_FUNCTION_NAMES[20] = {
    "HW_RESET",              // 0
    "SW_RESET",              // 1
    "FACTORY_RESET",         // 2
    "GET_INFO",              // 3
    "SET_NAME",              // 4
    "SET_COMM_CONFIG",       // 5
    "SET_RF_PARAMS",         // 6
    "ENTER_CMD_MODE",        // 7
    "ENTER_DATA_MODE",       // 8
    "START_BROADCAST",       // 9
    "CONNECT",               // 10
    "DISCONNECT",            // 11
    "GET_CONNECTION_STATUS", // 12
    "ENTER_SLEEP",           // 13
    "WAKEUP",                // 14
    "START_DISCOVERY",       // 15
    "SEND_DATA",             // 16
    "GET_DIAGNOSTICS",       // 17
    "SET_SECURITY",          // 18
    "MANAGE_WHITELIST"       // 19
};
```

---

**End of Document**

_Document created: 2026-02-08_  
_Updates: Config Handler for 3 data flows (JSON Config, Discovery, Setup Commands)_  
_Status: Ready for Implementation_
