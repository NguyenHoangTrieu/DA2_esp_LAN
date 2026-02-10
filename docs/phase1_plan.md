# Phase 1: JSON Parsers & Module Config Controller
## Detailed Implementation Plan

---

## Overview

Phase 1 xây dựng **parser architecture** và **config controller** cho module configuration system:

### Parser Architecture (Modular Design)

1. **Common Parser** (`json_config_parser`) - Parse metadata chung
   - Module ID, type, name
   - Communication config (port type + parameters)
   - Delegate function parsing cho module-specific parsers

2. **BLE-Specific Parser** (`json_ble_config_parser`) - Parse BLE functions
   - **Hardcode 15 function names** để validate
   - Parse commands, GPIO, timing for each function
   - **Chỉ implement trong Phase 1**

3. **Future Parsers** (Phase 3+):
   - `json_zigbee_config_parser` - Hardcode Zigbee function names
   - `json_lora_config_parser` - Hardcode LoRa function names
   - `json_thread_config_parser` - Hardcode Thread function names

4. **Module Config Controller** - Execute functions dựa trên parsed config

**Duration**: 2-3 days  
**Location**: `DA2_esp_LAN/Middleware/`  
**Dependencies**: cJSON library (ESP-IDF), stack_handler (BSP)  

---

## Component 1: Common JSON Config Parser

### 1.1. Directory Structure

```
DA2_esp_LAN/Middleware/JSON_Config_Parser/
├── CMakeLists.txt
├── include/
│   ├── json_config_parser.h          // Common parser
│   └── json_ble_config_parser.h      // BLE-specific
└── src/
    ├── json_config_parser.c           // Common implementation
    └── json_ble_config_parser.c       // BLE implementation
```

### 1.2. Common Parser Header (json_config_parser.h)

**Purpose**: Parse module metadata và communication config

**Constants**:
```c
#define MAX_MODULE_ID_LEN         4
#define MAX_MODULE_TYPE_LEN       32
#define MAX_MODULE_NAME_LEN       64
#define MAX_GPIO_ACTIONS          5
#define MAX_PIN_ID_LEN            4  // "01", "11", etc.
```

**Enums**:
- `comm_port_type_t` - UART, SPI, I2C, USB
- `uart_parity_t` - None, Even, Odd

**Structs**:
- `gpio_control_t` - Pin ID + state (reusable)
- `uart_params_t`, `spi_params_t`, `i2c_params_t` - Communication parameters
- `comm_config_t` - Port type + params union
- `module_metadata_t` - ID, type, name, communication

**APIs**:
```c
// Parse metadata only (không parse functions)
esp_err_t json_config_parse_metadata(const char *json_str, module_metadata_t *metadata);

// Validate communication config
esp_err_t json_config_validate_comm(const comm_config_t *comm);

// Helper: String conversions
comm_port_type_t json_config_string_to_port_type(const char *str);
uart_parity_t json_config_string_to_parity(const char *str);
```

### 1.3. Common Parser Implementation

**Step 1**: Parse module metadata
- Extract `module_id`, `module_type`, `module_name`
- Validate not empty

**Step 2**: Parse communication
- Extract `module_communication.port_type` → enum
- Based on port type, extract parameters:
  - UART: baudrate, parity, stopbit
  - SPI: clock_speed, mode, bit_order
  - I2C: address, clock_speed
- Populate `comm_config_t` union

**Step 3**: Return metadata
- **KHÔNG parse functions** - Để cho module-specific parser
- Return `module_metadata_t`

---

## Component 2: BLE-Specific Parser

### 2.1. Header (json_ble_config_parser.h)

**Purpose**: Parse BLE module configuration với hardcoded function names

**Constants**:
```c
#define BLE_MAX_FUNCTIONS         15  // Hardcoded: exactly 15 core functions
#define BLE_FUNCTION_NAME_LEN     32
#define BLE_COMMAND_LEN           128
#define BLE_RESPONSE_LEN          64
```

**Hardcoded Function Names** (Enum):
```c
typedef enum {
    BLE_FUNC_HW_RESET = 0,
    BLE_FUNC_SW_RESET,
    BLE_FUNC_FACTORY_RESET,
    BLE_FUNC_GET_INFO,
    BLE_FUNC_SET_NAME,
    BLE_FUNC_SET_COMM_CONFIG,
    BLE_FUNC_SET_RF_PARAMS,
    BLE_FUNC_ENTER_CMD_MODE,
    BLE_FUNC_ENTER_DATA_MODE,
    BLE_FUNC_START_BROADCAST,
    BLE_FUNC_CONNECT,
    BLE_FUNC_DISCONNECT,
    BLE_FUNC_GET_CONNECTION_STATUS,
    BLE_FUNC_ENTER_SLEEP,
    BLE_FUNC_WAKEUP,
    BLE_FUNC_MAX
} ble_function_id_t;
```

**Function Name Mapping**:
```c
static const char* BLE_FUNCTION_NAMES[BLE_FUNC_MAX] = {
    "MODULE_HW_RESET",
    "MODULE_SW_RESET",
    "MODULE_FACTORY_RESET",
    "MODULE_GET_INFO",
    "MODULE_SET_NAME",
    "MODULE_SET_COMM_CONFIG",
    "MODULE_SET_RF_PARAMS",
    "MODULE_ENTER_CMD_MODE",
    "MODULE_ENTER_DATA_MODE",
    "MODULE_START_BROADCAST",
    "MODULE_CONNECT",
    "MODULE_DISCONNECT",
    "MODULE_GET_CONNECTION_STATUS",
    "MODULE_ENTER_SLEEP",
    "MODULE_WAKEUP"
};
```

**Structs**:
```c
typedef struct {
    ble_function_id_t function_id;    // Enum ID
    char command[BLE_COMMAND_LEN];
    gpio_control_t gpio_start[MAX_GPIO_ACTIONS];
    uint8_t gpio_start_count;
    uint16_t delay_start_ms;
    char expect_response[BLE_RESPONSE_LEN];
    uint16_t timeout_ms;
    gpio_control_t gpio_end[MAX_GPIO_ACTIONS];
    uint8_t gpio_end_count;
    uint16_t delay_end_ms;
} ble_function_config_t;

typedef struct {
    module_metadata_t metadata;
    ble_function_config_t functions[BLE_MAX_FUNCTIONS];
    uint8_t function_count;
} ble_module_config_t;
```

**APIs**:
```c
// Parse complete BLE config (metadata + functions)
esp_err_t json_ble_config_parse(const char *json_str, ble_module_config_t *config);

// Validate BLE config
esp_err_t json_ble_config_validate(const ble_module_config_t *config);

// Lookup function by name
ble_function_id_t json_ble_get_function_id(const char *function_name);

// Free resources
void json_ble_config_free(ble_module_config_t *config);
```

### 2.2. BLE Parser Implementation

**Step 1**: Parse metadata
- Call `json_config_parse_metadata(json_str, &config->metadata)`
- Verify `metadata.module_type == "BLE"`

**Step 2**: Parse functions array
- Extract `functions[]` từ JSON
- For each function in array:
  - Extract `function_name`
  - **Lookup in BLE_FUNCTION_NAMES** array → `function_id`
  - If not found → ERROR (invalid function name for BLE)
  - Extract `command`, GPIO arrays, delays, timeout, expect_response
  - Store in `functions[function_id]` (indexed by enum)

**Step 3**: Validate
- Check all 15 hardcoded functions present (or allow optional ones)
- Validate GPIO pin format
- Check required fields not empty

**Implementation Note**:
- Functions stored by `function_id` index (0-14), NOT by parse order
- Missing functions → Set `function_id` unused or error
- Duplicate function names → ERROR

---

## Component 3: Module Config Controller (Helper Layer)

### 3.1. Purpose

**Module Config Controller** = Lớp trung gian giữa Handlers (BLE/Zigbee/LoRa) và BSP

**Cung cấp simple wrappers**:
- `module_bus_read/write()` - UART/SPI/I2C communication
- `module_gpio_write()` - GPIO control
- Giúp handlers gọi BSP mà không cần code nhiều

**KHÔNG chịu trách nhiệm**:
- Function execution logic (nằm ở BLE_Handler)
- Sequence control, timing, validation (nằm ở handlers)

### 3.2. Directory Structure

```
DA2_esp_LAN/Middleware/Module_Config_Controller/
├── CMakeLists.txt
├── include/
│   └── module_config_controller.h
└── src/
    └── module_config_controller.c
```

### 3.3. Header File (module_config_controller.h)

**APIs**:
```c
// Initialize (setup BSP if needed)
esp_err_t module_config_controller_init(void);

// Bus communication wrappers
esp_err_t module_bus_write(
    uint8_t stack_id,
    comm_port_type_t port_type,
    const uint8_t *data,
    size_t len
);

esp_err_t module_bus_read(
    uint8_t stack_id,
    comm_port_type_t port_type,
    uint8_t *buffer,
    size_t max_len,
    uint32_t timeout_ms,
    size_t *received_len
);

// GPIO control wrapper
esp_err_t module_gpio_write(
    uint8_t stack_id,
    const char *pin,      // "01", "02", etc.
    bool state            // true=HIGH, false=LOW
);

// Multi-GPIO wrapper
esp_err_t module_gpio_write_multi(
    uint8_t stack_id,
    const gpio_control_t *gpio_actions,
    size_t count
);
```

### 3.4. Implementation (module_config_controller.c)

**Step 1**: Init function
- Verify BSP drivers initialized
- No state needed (stateless wrapper)

**Step 2**: `module_bus_write()`
- Switch on `port_type`:
  - `COMM_PORT_UART` → call `module_uart_send()`
  - `COMM_PORT_SPI` → call `module_spi_send()`
  - `COMM_PORT_I2C` → call `module_i2c_send()`
- Return result

**Step 3**: `module_bus_read()`
- Switch on `port_type`:
  - `COMM_PORT_UART` → call `module_uart_receive()`
  - `COMM_PORT_SPI` → call `module_spi_receive()`
  - `COMM_PORT_I2C` → call `module_i2c_receive()`
- Return result + received length

**Step 4**: `module_gpio_write()`
- Parse pin "XY" → port (X), pin (Y)
- Validate port == stack_id
- Create single `gpio_action_t`
- Call `stack_handler_gpio_write_multi()` with 1 action

**Step 5**: `module_gpio_write_multi()`
- Loop through `gpio_actions[]`
- Parse each pin, convert to `gpio_action_t`
- Call `stack_handler_gpio_write_multi()`

**Implementation Example**:
```c
esp_err_t module_bus_write(uint8_t stack_id, comm_port_type_t port_type,
                           const uint8_t *data, size_t len) {
    switch (port_type) {
        case COMM_PORT_UART:
            return module_uart_send(stack_id, data, len);
        case COMM_PORT_SPI:
            return module_spi_send(stack_id, data, len);
        case COMM_PORT_I2C:
            return module_i2c_send(stack_id, data, len);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t module_gpio_write(uint8_t stack_id, const char *pin, bool state) {
    // Parse "XY" → port X, pin Y
    uint8_t port = pin[0] - '0';
    uint8_t pin_num = pin[1] - '0';
    
    if (port != stack_id) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create action
    gpio_action_t action = {
        .gpio_num = pin_num,
        .level = state ? 1 : 0
    };
    
    // Call stack handler
    return stack_handler_gpio_write_multi(stack_id, &action, 1);
}
```

### 3.5. Usage Example (from BLE_Handler)

```c
// In BLE_Handler - execute MODULE_HW_RESET function
esp_err_t ble_handler_hw_reset(uint8_t stack_id) {
    ble_function_config_t *func = &g_ble_config.functions[BLE_FUNC_HW_RESET];
    
    // GPIO start sequence
    for (int i = 0; i < func->gpio_start_count; i++) {
        module_gpio_write(stack_id, func->gpio_start[i].pin, 
                         func->gpio_start[i].state);
    }
    
    // Delay
    vTaskDelay(pdMS_TO_TICKS(func->delay_start_ms));
    
    // Command (if any)
    if (strlen(func->command) > 0) {
        module_bus_write(stack_id, config.metadata.communication.port_type,
                        (uint8_t*)func->command, strlen(func->command));
    }
    
    // GPIO end sequence
    for (int i = 0; i < func->gpio_end_count; i++) {
        module_gpio_write(stack_id, func->gpio_end[i].pin,
                         func->gpio_end[i].state);
    }
    
    return ESP_OK;
}
```

### 3.6. CMakeLists.txt

```cmake
idf_component_register(
    SRCS "src/module_config_controller.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        json_config_parser
        stack_handler
        module_uart_comm
        module_spi_comm
        module_i2c_comm
)
```

### 3.7. Testing

**Test Cases**:
1. `module_bus_write(UART)` → Calls correct BSP function
2. `module_bus_read(SPI)` → Returns data correctly
3. `module_gpio_write("01", HIGH)` → Toggles GPIO correctly
4. `module_gpio_write_multi()` → Batch GPIO works
5. Invalid port type → Returns error
6. Invalid pin format → Returns error

---

## Integration Points

### BSP Dependencies

**Required APIs** (verify exists):

**Stack Handler**:
- `esp_err_t stack_handler_gpio_write_multi(uint8_t stack_id, const gpio_action_t *actions, size_t count)`
- `esp_err_t stack_handler_lock(uint8_t stack_id, TickType_t timeout)`
- `esp_err_t stack_handler_unlock(uint8_t stack_id)`

**Module UART Comm**:
- `esp_err_t module_uart_init(uint8_t port, uint32_t baud, uint8_t parity, uint8_t stop)`
- `esp_err_t module_uart_send(uint8_t port, const char *data, size_t len)`
- `esp_err_t module_uart_receive(uint8_t port, char *buffer, size_t max_len, uint32_t timeout_ms, size_t *received_len)`

**Module SPI Comm** (similar APIs)  
**Module I2C Comm** (similar APIs)

### cJSON Integration

- Add cJSON dependency trong CMakeLists.txt
- Use `cJSON_Parse()`, `cJSON_GetObjectItem()`, `cJSON_IsString()`, etc.
- Always call `cJSON_Delete()` để avoid memory leak

---

## Implementation Checklist

### Common JSON Parser
- [ ] Create directory structure
- [ ] Create `json_config_parser.h` với common structs (metadata, comm, GPIO)
- [ ] Implement string conversion helpers (port_type, parity)
- [ ] Implement comm params parser (UART/SPI/I2C)
- [ ] Implement metadata parser (id, type, name)
- [ ] Implement `json_config_parse_metadata()` function
- [ ] Implement validation for comm config
- [ ] Add comprehensive logging (INFO/WARN/ERROR)
- [ ] Unit tests cho metadata parsing

### BLE-Specific Parser
- [ ] Create `json_ble_config_parser.h`
- [ ] Define `ble_function_id_t` enum với 15 functions
- [ ] Define `BLE_FUNCTION_NAMES[]` lookup table
- [ ] Define `ble_function_config_t` struct
- [ ] Define `ble_module_config_t` struct
- [ ] Implement `json_ble_get_function_id()` lookup helper
- [ ] Implement GPIO array parser
- [ ] Implement functions array parser (với function name validation)
- [ ] Implement main `json_ble_config_parse()` function
- [ ] Implement `json_ble_config_validate()`
- [ ] Add BLE-specific logging
- [ ] Unit tests với BLE JSON từ TODO.md
- [ ] Memory leak check

### Module Config Controller (Helper Layer)
- [ ] Create directory structure
- [ ] Create `module_config_controller.h` với wrapper APIs
- [ ] Implement `module_config_controller_init()`
- [ ] Implement `module_bus_write()` - switch cho UART/SPI/I2C
- [ ] Implement `module_bus_read()` - switch cho UART/SPI/I2C
- [ ] Implement `module_gpio_write()` - single GPIO wrapper
- [ ] Implement `module_gpio_write_multi()` - batch GPIO wrapper
- [ ] Add error handling
- [ ] Create CMakeLists.txt với BSP dependencies
- [ ] Unit tests cho wrappers
- [ ] Test với actual BSP drivers

### Integration
- [ ] Test complete flow: JSON → Common Parser → BLE Parser → Controller → Execute
- [ ] Verify all 15 BLE functions work
- [ ] Test parameter substitution ({PARAM})
- [ ] Test GPIO sequences
- [ ] Test error scenarios (invalid JSON, missing functions, etc.)

---

## Error Handling Strategy

**JSON Parser**:
- Invalid JSON syntax → ESP_ERR_INVALID_ARG
- Missing required fields → ESP_ERR_INVALID_ARG
- Invalid port_type → ESP_ERR_NOT_SUPPORTED
- Memory allocation failed → ESP_ERR_NO_MEM
- Always log error details với ESP_LOGE()

**Module Controller**:
- Stack not loaded → ESP_ERR_INVALID_STATE
- Function not found → ESP_ERR_NOT_FOUND
- GPIO error → Log warning, continue (don't fail entire execution)
- Command send failed → ESP_FAIL
- Response timeout → ESP_ERR_TIMEOUT
- Response validation failed → ESP_ERR_INVALID_RESPONSE
- Mutex timeout → ESP_ERR_TIMEOUT

---

## Timeline Estimate

| Task | Duration | Cumulative |
|------|----------|------------|
| JSON Parser header design | 2h | 2h |
| JSON Parser implementation | 6h | 8h |
| JSON Parser testing | 4h | 12h |
| Module Controller header design | 2h | 14h |
| Module Controller implementation | 8h | 22h |
| Module Controller testing | 6h | 28h |
| Integration testing | 4h | 32h |
| Documentation & cleanup | 2h | 34h |
| **Total** | **~34h (2-3 days)** | |

---

## Success Criteria

✅ **Functionality**:
- JSON parser handles BLE example JSON correctly
- All fields extracted và validated
- Module controller loads config successfully
- GPIO sequences execute với correct timing
- UART commands send/receive work
- Response validation catches mismatches

✅ **Quality**:
- No memory leaks (verified với heap monitoring)
- Thread-safe với mutex protection
- Graceful error handling for all edge cases
- Comprehensive logging at appropriate levels
- Code follows ESP-IDF style guide

✅ **Performance**:
- JSON parse time < 100ms
- Function execution < 2s (including delays)
- RAM usage < 20KB total for both components
- No stack overflows

✅ **Testing**:
- Unit tests pass 100%
- Integration test passes với hardware
- Tested với invalid inputs (malformed JSON, missing fields)
- Tested với concurrent access (mutex protection works)

---

## Next Steps After Phase 1

1. ✅ Complete JSON Parser và Module Controller
2. ⏭️ Proceed to Phase 2: BSP driver verification
3. ⏭️ Create example JSON configs for BLE module
4. ⏭️ Begin Phase 2.5: Config flow implementation (WAN/LAN handlers)
5. ⏭️ Begin Phase 3: BLE Handler (middleware + application)

---

**Phase 1 Status**: Ready to implement  
**Blockers**: None (cJSON available, stack_handler exists)  
**Risk Level**: Low (well-defined scope, clear dependencies)
