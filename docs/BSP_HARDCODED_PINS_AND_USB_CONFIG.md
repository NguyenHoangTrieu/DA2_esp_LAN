# BSP Driver Hardcoded Pins & USB Config Guide

**Date:** February 7, 2026  
**Purpose:** Document hardcoded pin/port configuration và USB parameters cho JSON config  

---

## 📌 **TÓM TẮT THAY ĐỔI**

### **Trước khi update:**
- BSP drivers nhận pins/ports qua config struct
- JSON phải chứa pin numbers → Phức tạp, dễ sai

### **Sau khi update:**
- ✅ Pins/ports được **hardcoded** trong BSP drivers
- ✅ JSON chỉ cần `stack_id` (0 hoặc 1) để xác định hardware
- ✅ Đơn giản hóa JSON config, giảm lỗi cấu hình

---

## 🔌 **HARDCODED PIN DEFINITIONS**

### **1. UART Pins** (`Module_UART_Communication/include/module_uart_comm.h`)

```c
// Stack 0 UART pins
#define STACK0_UART_PORT    UART_NUM_1
#define STACK0_UART_TX_PIN  17
#define STACK0_UART_RX_PIN  18

// Stack 1 UART pins
#define STACK1_UART_PORT    UART_NUM_2
#define STACK1_UART_TX_PIN  15
#define STACK1_UART_RX_PIN  16
```

**Config Struct:**
```c
typedef struct {
  uint8_t stack_id;           // 0 or 1 → determines pins automatically
  uint32_t baudrate;
  uart_parity_t parity;
  uart_stop_bits_t stop_bits;
  size_t rx_buffer_size;
  size_t tx_buffer_size;
} module_uart_config_t;
```

---

### **2. SPI Pins** (`Module_SPI_Communication/include/module_spi_comm.h`)

```c
// Stack 0 SPI pins
#define STACK0_SPI_HOST       SPI2_HOST
#define STACK0_SPI_MOSI_PIN   13
#define STACK0_SPI_MISO_PIN   12
#define STACK0_SPI_SCLK_PIN   14
#define STACK0_SPI_CS_PIN     15

// Stack 1 SPI pins
#define STACK1_SPI_HOST       SPI3_HOST
#define STACK1_SPI_MOSI_PIN   23
#define STACK1_SPI_MISO_PIN   19
#define STACK1_SPI_SCLK_PIN   18
#define STACK1_SPI_CS_PIN     5
```

**Config Struct:**
```c
typedef struct {
  uint8_t stack_id;            // 0 or 1
  uint32_t clock_speed_hz;
  uint8_t mode;                // 0-3
  uint8_t queue_size;
} module_spi_config_t;
```

---

### **3. I2C Pins** (`Module_I2C_Communication/include/module_i2c_comm.h`)

```c
// Stack 0 I2C pins
#define STACK0_I2C_PORT       I2C_NUM_0
#define STACK0_I2C_SDA_PIN    21
#define STACK0_I2C_SCL_PIN    22

// Stack 1 I2C pins
#define STACK1_I2C_PORT       I2C_NUM_1
#define STACK1_I2C_SDA_PIN    26
#define STACK1_I2C_SCL_PIN    27
```

**Config Struct:**
```c
typedef struct {
  uint8_t stack_id;            // 0 or 1
  uint8_t device_address;      // 7-bit I2C address
  uint32_t clock_speed_hz;
  bool pullup_enable;
} module_i2c_config_t;
```

---

### **4. USB Configuration** (`Module_USB_Communication/include/module_usb_comm.h`)

```c
// Stack 0 USB configuration
#define STACK0_USB_VID        0x303A  // Espressif VID
#define STACK0_USB_PID        0x1001  // Custom PID

// Stack 1 USB configuration  
#define STACK1_USB_VID        0x303A
#define STACK1_USB_PID        0x1002
```

**USB CDC Line Coding:**
```c
typedef struct {
  uint32_t bit_rate;      // Bit rate (bps) - e.g., 115200
  uint8_t stop_bits;      // 0=1bit, 1=1.5bits, 2=2bits
  uint8_t parity;         // 0=None, 1=Odd, 2=Even
  uint8_t data_bits;      // 5, 6, 7, 8, or 16
} usb_cdc_line_coding_t;
```

**Config Struct:**
```c
typedef struct {
  uint8_t stack_id;          // 0 or 1
  usb_cdc_line_coding_t line_coding;
  size_t rx_buffer_size;
  size_t tx_buffer_size;
} module_usb_config_t;
```

**Note:** USB D+/D- pins are **hardwired** in ESP32-S3/C3/C6 (không cần config)

---

## 📄 **JSON CONFIG PARAMETERS**

### **Common Fields (Tất cả module types):**

```json
{
  "module_id": "00",              // "00" = Stack 0, "01" = Stack 1
  "module_type": "BLE",           // BLE, Zigbee, LoRa, etc.
  "module_name": "JDY-23",
  "module_communication": {
    "port_type": "uart",          // "uart", "spi", "i2c", "usb"
    "parameters": { ... }          // See below for each type
  },
  "functions": [ ... ]
}
```

---

### **1. UART Parameters:**

```json
"module_communication": {
  "port_type": "uart",
  "parameters": {
    "baudrate": 9600,              // Baud rate (bps)
    "parity": "none",              // "none", "even", "odd"
    "stopbit": 1,                  // 1 or 2
    "rx_buffer_size": 1024,        // RX buffer (bytes) - Optional, default 1024
    "tx_buffer_size": 1024         // TX buffer (bytes) - Optional, default 1024
  }
}
```

**Pins được auto-select từ `module_id`:**
- `module_id "00"` → Stack 0 → UART1, TX=17, RX=18
- `module_id "01"` → Stack 1 → UART2, TX=15, RX=16

---

### **2. SPI Parameters:**

```json
"module_communication": {
  "port_type": "spi",
  "parameters": {
    "clock_speed": 1000000,        // Clock speed (Hz)
    "mode": 0,                     // SPI mode (0-3)
    "bit_order": "msb",            // "msb" or "lsb" - Currently not used in driver
    "queue_size": 1                // Transaction queue size - Optional, default 1
  }
}
```

**Pins được auto-select từ `module_id`:**
- `module_id "00"` → Stack 0 → SPI2, MOSI=13, MISO=12, SCLK=14, CS=15
- `module_id "01"` → Stack 1 → SPI3, MOSI=23, MISO=19, SCLK=18, CS=5

---

### **3. I2C Parameters:**

```json
"module_communication": {
  "port_type": "i2c",
  "parameters": {
    "address": 80,                 // I2C device address (7-bit decimal, e.g., 0x50 = 80)
    "clock_speed": 100000,         // Clock speed (Hz), e.g., 100kHz
    "pullup_enable": true          // Enable internal pull-up resistors - Optional
  }
}
```

**Pins được auto-select từ `module_id`:**
- `module_id "00"` → Stack 0 → I2C0, SDA=21, SCL=22
- `module_id "01"` → Stack 1 → I2C1, SDA=26, SCL=27

---

### **4. USB Parameters (NEW):**

```json
"module_communication": {
  "port_type": "usb",
  "parameters": {
    "bit_rate": 115200,            // Bit rate (bps) - For CDC line coding
    "stop_bits": 0,                // 0=1bit, 1=1.5bits, 2=2bits
    "parity": 0,                   // 0=None, 1=Odd, 2=Even
    "data_bits": 8,                // 5, 6, 7, 8, or 16
    "rx_buffer_size": 2048,        // RX buffer (bytes) - Optional, default 1024
    "tx_buffer_size": 2048         // TX buffer (bytes) - Optional, default 1024
  }
}
```

**USB Pins:**
- **Không cần config pins!** USB D+/D- hardwired trong ESP32-S3/C3/C6
- `module_id "00"` → Stack 0 → VID=0x303A, PID=0x1001
- `module_id "01"` → Stack 1 → VID=0x303A, PID=0x1002

**Driver:** ESP32 USB Serial/JTAG peripheral
- Single USB port shared with JTAG debugging
- CDC (Communication Device Class) compliant
- Automatic enumeration as serial port trên PC

---

## ✅ **EXAMPLE: BLE Module với UART**

```json
{
  "module_id": "00",
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
    }
  ]
}
```

**Result:** Driver sẽ auto-config UART1 (TX=17, RX=18) với baudrate 9600

---

## ✅ **EXAMPLE: Zigbee Module với USB**

```json
{
  "module_id": "01",
  "module_type": "Zigbee",
  "module_name": "CC2652",
  "module_communication": {
    "port_type": "usb",
    "parameters": {
      "bit_rate": 115200,
      "stop_bits": 0,
      "parity": 0,
      "data_bits": 8,
      "rx_buffer_size": 4096,
      "tx_buffer_size": 4096
    }
  },
  "functions": [ ... ]
}
```

**Result:** Driver sẽ use USB Serial/JTAG với buffer sizes 4KB

---

## 🔧 **ĐIỀU CHỈNH PINS THEO HARDWARE**

**Để thay đổi pins theo schematic của bạn:**

1. Mở header file tương ứng:
   - UART: `BSP/Module_UART_Communication/include/module_uart_comm.h`
   - SPI: `BSP/Module_SPI_Communication/include/module_spi_comm.h`
   - I2C: `BSP/Module_I2C_Communication/include/module_i2c_comm.h`
   - USB: `BSP/Module_USB_Communication/include/module_usb_comm.h`

2. Sửa `#define` constants:
   ```c
   // Example: Change UART Stack 0 pins
   #define STACK0_UART_TX_PIN  25  // Was 17
   #define STACK0_UART_RX_PIN  26  // Was 18
   ```

3. Rebuild project - Không cần thay đổi JSON config!

---

## 📝 **NOTES**

### **USB Driver Details:**
- **Peripheral:** ESP32-S3/C3/C6 USB Serial/JTAG
- **API Used:** `driver/usb_serial_jtag.h`
- **Thread-safe:** Yes (uses mutexes)
- **Concurrent debug:** USB Serial + JTAG debugging work simultaneously
- **Auto-enumeration:** Appears as `/dev/ttyACM0` (Linux) hoặc `COMx` (Windows)

### **USB CDC Line Coding:**
- `bit_rate`: Không ảnh hưởng actual speed (USB full-speed = 12 Mbps)
- Parameters chỉ for CDC compatibility với terminal programs
- ESP32 ignores line coding, always runs at full USB speed

### **JSON Parser Integration:**
JSON parser hiện tại (Phase 1) đã hỗ trợ:
- ✅ UART parameters parsing
- ✅ SPI parameters parsing
- ✅ I2C parameters parsing
- ❌ USB parameters parsing - **CẦN THÊM**

**Action Required:** Update `json_config_parser.c` để parse USB parameters!

---

## 🎯 **SUMMARY**

| Feature | Before | After |
|---------|--------|-------|
| **Config Complexity** | High (pins trong JSON) | Low (chỉ stack_id) |
| **Pin Management** | JSON (error-prone) | Hardcoded (reliable) |
| **Hardware Changes** | Update JSON | Update #define |
| **USB Support** | Not implemented | ✅ Fully functional |
| **JSON Size** | Larger | Smaller |

**Benefits:**
- ✅ Simplified JSON configuration
- ✅ Reduced user errors (không thể sai pins)
- ✅ Easier hardware adaptation (#define only)
- ✅ Complete USB CDC support
- ✅ Consistent API across all comm types
