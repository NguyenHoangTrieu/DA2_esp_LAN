# PC App Integration Guide - Phase 3 BLE Handler

**Date:** February 7, 2026  
**Purpose:** Document PC config app (`config_app/main.py`) responsibilities for BLE transportation gateway

---

## Overview

The **PC Config App** works in tandem with the **gateway** to enable BLE device discovery, connection, and data exchange. The app generates JSON configuration files that define how the BLE module operates, then monitors/controls gateway execution.

## Workflow Phases

### Phase 1: JSON Configuration Generation

**Responsibility:** PC app generates complete BLE module configuration

```
PC App generates JSON with:
├── Module metadata (type, model, comm port)
├── Communication config (UART/SPI/I2C/USB parameters)
└── 20 Function Definitions
    ├── 15 Core Functions (mandatory):
    │   ├── MODULE_HW_RESET
    │   ├── MODULE_SW_RESET
    │   ├── MODULE_FACTORY_RESET
    │   ├── MODULE_GET_INFO
    │   ├── MODULE_SET_NAME
    │   ├── MODULE_SET_COMM_CONFIG
    │   ├── MODULE_SET_RF_PARAMS
    │   ├── MODULE_ENTER_CMD_MODE
    │   ├── MODULE_ENTER_DATA_MODE
    │   ├── MODULE_START_BROADCAST
    │   ├── MODULE_CONNECT
    │   ├── MODULE_DISCONNECT
    │   ├── MODULE_GET_CONNECTION_STATUS
    │   ├── MODULE_ENTER_SLEEP
    │   └── MODULE_WAKEUP
    └── 5 Promoted Optional (PC app workflows):
        ├── MODULE_START_DISCOVERY    [Scan for devices]
        ├── MODULE_SEND_DATA          [Transparent send]
        ├── MODULE_GET_DIAGNOSTICS    [Signal strength]
        ├── MODULE_SET_SECURITY       [Pairing/bonding]
        └── MODULE_MANAGE_WHITELIST   [Device filter]
```

**Each function in JSON includes:**

```json
{
  "function_name": "MODULE_CONNECT",
  "gpio_start_control": [
    {"pin": "GPIO12", "state": "HIGH"}
  ],
  "delay_start": 100,
  "command": "AT+CONNECT:${param}",
  "expect_response": "OK",
  "timeout": 5000,
  "gpio_end_control": [],
  "delay_end": 0
}
```

**Implementation Details:**
- GPIO pins must match ESPHome config (GPIO16, GPIO17, etc.)
- Commands use AT syntax or module-specific protocol
- `${param}` placeholder for parameterized functions (e.g., MAC address, device name)
- Empty command means "skip this function if not available"
- Timeout in milliseconds

### Phase 2: Upload to Gateway

**PC App sends JSON via:**
- Serial connection (UART at 115200 baud)
- Ethernet/TCP (if WAN MCU has network interface)
- REST API endpoint (planned for future)

**Expected Response:**
```
Gateway ACK: "CONFIG_RECEIVED"
Gateway Validation: "JSON_PARSED_OK: 20/20 functions"
or
Gateway Error: "PARSE_ERROR: Invalid function MODULE_XYZ"
```

### Phase 3: Device Discovery

**PC App triggers discovery workflow:**

```
Step 1: Reset BLE Module
  PC sends: "execute MODULE_HW_RESET"
  Gateway: Executes GPIO reset sequence
  
Step 2: Enter Command Mode
  PC sends: "execute MODULE_ENTER_CMD_MODE"
  Gateway: Sends "AT+ENTER_CMD_MODE" or similar
  
Step 3: Start Scanning
  PC sends: "execute MODULE_START_DISCOVERY"
  Gateway: Sends AT+SCAN command to BLE module
  
Step 4: Parse Scan Results
  PC receives: [
    {"mac": "AA:BB:CC:DD:EE:01", "name": "Device1", "rssi": -65},
    {"mac": "AA:BB:CC:DD:EE:02", "name": "Device2", "rssi": -72}
  ]
  PC displays to user, user selects device
```

**Response Format (from gateway):**

```c
// After MODULE_START_DISCOVERY execution:
// Gateway parses BLE module response and formats:
{
  "devices": [
    {
      "mac_address": "AA:BB:CC:DD:EE:FF",
      "device_name": "HC-05",
      "rssi": -65,
      "connectable": true
    }
  ],
  "scan_count": 5,
  "timestamp": 1707295200
}
```

### Phase 4: Connection & Data Exchange

**PC App workflow:**

```
Step 1: Connect to Selected Device
  PC sends: "execute MODULE_CONNECT MAC:AA:BB:CC:DD:EE:FF"
  Gateway: Calls ble_handler_connect(0, "AA:BB:CC:DD:EE:FF")
  Gateway executes MODULE_CONNECT function with MAC parameter
  Response: "CONNECTED" or "CONNECTION_TIMEOUT"

Step 2: Enter Data Mode
  PC sends: "execute MODULE_ENTER_DATA_MODE"
  Gateway: Switches to transparent data forwarding
  
Step 3: Send Test Data
  PC sends: "data AA:BB:CC:DD:EE:FF Hello Gateway!"
  Gateway: Calls ble_handler_send_data(0, data, len)
  Gateway: Forwards data to BLE device in transparent mode
  
Step 4: Receive Response (uplink)
  BLE Device responds → Gateway receives data
  Gateway queues uplink via ble_handler_task_enqueue_uplink()
  Gateway forwards to Server via mcu_wan_enqueue_uplink(HANDLER_BLE, ...)
  PC App receives notification from Server:
  "uplink HANDLER_BLE [AA:BB:CC:DD:EE:FF] Response data"
  
Step 5: Validate Data Integrity
  PC App checks:
  - MAC address matches sent command
  - Data payload matches expected format
  - No data corruption during transmission
```

### Phase 5: Graceful Disconnect

```
Step 1: Disconnect Device
  PC sends: "execute MODULE_DISCONNECT"
  Gateway: Calls ble_handler_disconnect(0)
  
Step 2: Optional Diagnostics
  PC sends: "execute MODULE_GET_DIAGNOSTICS"
  Gateway: Queries link quality, RSSI, connection metrics
  Response: {"rssi": -65, "connection_quality": 95}
  
Step 3: Sleep Mode (optional)
  PC sends: "execute MODULE_ENTER_SLEEP"
  Gateway: Puts BLE module in low-power state
  
Step 4: Wakeup (when needed again)
  PC sends: "execute MODULE_WAKEUP"
  Gateway: Wakes BLE module for new discovery
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     PC Config App (main.py)                      │
├─────────────────────────────────────────────────────────────────┤
│ • Generate JSON with 20 functions                                │
│ • Upload to gateway                                              │
│ • Trigger discovery: MODULE_START_DISCOVERY                      │
│ • Trigger connect: MODULE_CONNECT with MAC                       │
│ • Send data: MODULE_SEND_DATA with payload                       │
│ • Get status: MODULE_GET_DIAGNOSTICS                             │
└─────────────────────────────────────────────────────────────────┘
                              ↕
                    Serial / Ethernet
                              ↕
┌─────────────────────────────────────────────────────────────────┐
│                  Gateway (LAN MCU - ESP32S3)                     │
├─────────────────────────────────────────────────────────────────┤
│ JSON Parser (20 functions)                                       │
│     ↓                                                             │
│ Middleware (ble_handler.c - execute functions)                   │
│     ↓                                                             │
│ Application (ble_handler_task.c - queues + forwarding)           │
│     ↓                                                             │
│ Module Controller (UART/SPI/I2C/USB)                             │
│     ↓                                                             │
│ BLE Module (physical hardware)                                   │
└─────────────────────────────────────────────────────────────────┘
                              ↕
                          BLE Radio
                              ↕
┌─────────────────────────────────────────────────────────────────┐
│                    BLE Device (Remote)                           │
├─────────────────────────────────────────────────────────────────┤
│ • HC-05 / HM-10 / NRF52 / etc.                                   │
│ • Advertises services                                             │
│ • Accepts connections                                             │
│ • Sends/receives data                                             │
└─────────────────────────────────────────────────────────────────┘

Uplink Path (BLE Device → Server):
  BLE Device --[data]-→ Gateway --[uplink queue]-→ MCU_WAN_Handler
  --[QSPI]-→ WAN MCU --[IP/Internet]-→ Server

Downlink Path (Server → BLE Device):
  Server --[IP/Internet]-→ WAN MCU --[QSPI]-→ MCU_WAN_Handler
  --[downlink queue]-→ Gateway --[BLE]-→ BLE Device
```

---

## 20 Function Reference

### Core Functions (15) - All Must Be Implemented

| # | Function | Purpose | Required | Example Command |
|---|----------|---------|----------|-----------------|
| 0 | MODULE_HW_RESET | Hardware reset via GPIO | Yes | GPIO12: LOW→HIGH |
| 1 | MODULE_SW_RESET | Software reset | Yes | AT+RST |
| 2 | MODULE_FACTORY_RESET | Full reset to factory | Yes | AT+FACTORY_RESET |
| 3 | MODULE_GET_INFO | Get version/MAC | Yes | AT+VERSION |
| 4 | MODULE_SET_NAME | Set advertising name | Yes | AT+NAME:${param} |
| 5 | MODULE_SET_COMM_CONFIG | Configure UART params | Yes | AT+BAUD:115200 |
| 6 | MODULE_SET_RF_PARAMS | Set TX power/freq | Yes | AT+TXPOWER:0 |
| 7 | MODULE_ENTER_CMD_MODE | Enter AT mode | Yes | AT+ENTER_CMD_MODE |
| 8 | MODULE_ENTER_DATA_MODE | Enter transparent mode | Yes | AT+ENTER_DATA_MODE |
| 9 | MODULE_START_BROADCAST | Start advertising | Yes | AT+ADVERTISE |
| 10 | MODULE_CONNECT | Connect to device | Yes | AT+CONNECT:${param} |
| 11 | MODULE_DISCONNECT | Disconnect | Yes | AT+DISCONNECT |
| 12 | MODULE_GET_CONNECTION_STATUS | Check connection | Yes | AT+CONNECTED? |
| 13 | MODULE_ENTER_SLEEP | Enter sleep mode | Yes | AT+SLEEP |
| 14 | MODULE_WAKEUP | Wake from sleep | Yes | AT+WAKEUP |

### Promoted Optional Functions (5) - For PC App Workflows

| # | Function | Purpose | Required | Example Command |
|---|----------|---------|----------|-----------------|
| 15 | MODULE_START_DISCOVERY | Scan for devices | Optional | AT+SCAN:5000 |
| 16 | MODULE_SEND_DATA | Send data transparent | Optional | Direct UART write |
| 17 | MODULE_GET_DIAGNOSTICS | Get RSSI/quality | Optional | AT+DIAGNOSTICS |
| 18 | MODULE_SET_SECURITY | Configure security | Optional | AT+SECURITY:1 |
| 19 | MODULE_MANAGE_WHITELIST | Manage device list | Optional | AT+WHITELIST:${param} |

**Notes:**
- If optional function not in JSON config, gateway skips it gracefully (returns ESP_OK)
- Core functions failure stops gateway startup
- PC app can query which functions are available after gateway boots

---

## Error Handling

### Common Scenarios

**Scenario 1: Optional Function Not Configured**
```
PC sends: "execute MODULE_START_DISCOVERY"
Gateway: Function not in JSON (skipped in config)
Response: {"status": "SKIPPED", "reason": "Not configured"}
PC App: Falls back to manual device entry or retries
```

**Scenario 2: Connection Timeout**
```
PC sends: "execute MODULE_CONNECT MAC:AA:BB:CC:DD:EE:FF"
Gateway: Sends command, waits 5 seconds, no response
Response: {"status": "ERROR", "error": "CONNECTION_TIMEOUT"}
PC App: Retries discovery, or user selects different device
```

**Scenario 3: Data Transmission Error**
```
PC sends: "execute MODULE_SEND_DATA with payload"
Gateway: BLE module not connected
Response: {"status": "ERROR", "error": "NOT_CONNECTED"}
PC App: Prompts user to connect first via MODULE_CONNECT
```

---

## Testing Checklist

- [ ] PC app generates valid 20-function JSON
- [ ] Gateway parses JSON without errors
- [ ] MODULE_HW_RESET executes GPIO sequence correctly
- [ ] MODULE_ENTER_CMD_MODE switches to command mode
- [ ] MODULE_START_DISCOVERY scans and returns device list
- [ ] MODULE_CONNECT with MAC succeeds
- [ ] MODULE_SEND_DATA forwards payload to BLE device
- [ ] Uplink data reaches PC app via MCU_WAN_Handler
- [ ] Downlink data from server reaches BLE device
- [ ] MODULE_GET_DIAGNOSTICS returns signal metrics
- [ ] MODULE_DISCONNECT cleanly terminates connection
- [ ] Optional functions skip gracefully if not configured
- [ ] Multi-device scenarios work (connect/disconnect cycle)
- [ ] Error messages are clear and actionable

---

## Configuration File Example

**Minimal JSON (core functions only):**
```json
{
  "module_type": "BLE",
  "module_model": "HC-05",
  "metadata": {
    "version": "1.0",
    "communication": {
      "port_type": "UART",
      "params": {
        "uart": {
          "tx_pin": 17,
          "rx_pin": 18,
          "baud_rate": 115200
        }
      }
    }
  },
  "functions": [
    {"function_name": "MODULE_HW_RESET", "gpio_start_control": [{"pin": "GPIO12", "state": "LOW"}, {"pin": "GPIO12", "state": "HIGH"}], "command": "", "expect_response": "", "timeout": 1000},
    {"function_name": "MODULE_SW_RESET", "command": "AT+RST", "expect_response": "OK", "timeout": 2000},
    ...
    {"function_name": "MODULE_START_DISCOVERY", "command": "AT+INQM=1,10,5", "expect_response": "", "timeout": 15000},
    ...
  ]
}
```

---

## Next Steps

1. **Phase 3 Implementation**: Middleware + Application layers
2. **JSON Validation Tool**: Add schema validation in PC app
3. **Command Queueing**: Implement PC app command interface for on-demand function execution
4. **Device Persistence**: Store discovered devices in local cache for fast reconnection
5. **Production Hardening**: Comprehensive error handling, retry logic, watchdog
