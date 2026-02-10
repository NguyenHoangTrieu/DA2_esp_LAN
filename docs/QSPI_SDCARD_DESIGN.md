# QSPI + SD Card Communication Design Document
## Modular Architecture with Mandatory QSPI Interface

---

## 1. SYSTEM OVERVIEW

Dual ESP32 architecture with **mandatory QSPI (Quad SPI)** for high-speed bidirectional communication and modular task separation.

**Key Features**:
- QSPI: 40 MHz, 4-bit parallel (20 MB/s theoretical, 8-12 MB/s practical)
- LAN MCU (Master): Protocol handlers (CAN/LoRa/ZigBee/RS485) + SD card storage
- WAN MCU (Slave): WiFi/MQTT + RTC (PCF8563)
- Uplink throughput: 800 KB/s sustained, 2 MB/s burst
- Downlink latency: <5 ms (ISR to data received)
- Coordinated FOTA: Version-based synchronization between MCUs

---

## 2. HARDWARE ARCHITECTURE

### 2.1. QSPI Bus (Mandatory - 40 MHz)

| Signal | LAN MCU (Master) | WAN MCU (Slave) | GPIO | Purpose |
|--------|------------------|-----------------|------|---------|
| CLK | Output | Input | GPIO12 | 40 MHz clock |
| CS | Output | Input | GPIO10 | Chip select |
| IO0 (D0) | Bidirectional | Bidirectional | GPIO11 | Data line 0 |
| IO1 (D1) | Bidirectional | Bidirectional | GPIO13 | Data line 1 |
| IO2 (D2) | Bidirectional | Bidirectional | GPIO14 | Data line 2 |
| IO3 (D3) | Bidirectional | Bidirectional | GPIO15 | Data line 3 |
| DR_LAN | Input (ISR) | - | GPIO46 | Data-ready from WAN |
| DR_WAN | - | Output | GPIO8 | Data-ready to LAN |

### 2.2. SD Card Interface (LAN MCU Only)

| Pin | Function | GPIO | Notes |
|-----|----------|------|-------|
| CLK | SDMMC Clock | GPIO7 | Separate from QSPI |
| CMD | Command | GPIO6 | |
| D0-D3 | Data Lines | GPIO8,3,4,5 | 4-bit mode |

**No Pin Conflicts**: QSPI (GPIO10-15) vs SD (GPIO3-8)

---

## 3. MODULAR FILE STRUCTURE

### 3.1. LAN MCU (Master) - 8 Files

```

mcu_wan_handler.h                  \# Public API
mcu_wan_handler_downlink.c         \# Downlink task (Priority 7)
mcu_wan_handler_uplink.c           \# Uplink task (Priority 5)
storage_handler.h/c                \# SD card abstraction
wan_comm.h/c                       \# QSPI master driver
frame_types.h                      \# Protocol definitions

```

### 3.2. WAN MCU (Slave) - 6 Files

```

mcu_lan_handler.h                  \# Public API
mcu_lan_handler_uplink.c           \# Uplink processor (Priority 6)
mcu_lan_handler_downlink.c         \# Downlink manager (no task)
lan_comm.h/c                       \# QSPI slave driver
frame_types.h                      \# Protocol definitions

```

---

## 4. MODULE RESPONSIBILITIES

### 4.1. LAN MCU

**mcu_wan_handler_downlink.c** (Priority 7):
- GPIO46 ISR response (<5ms latency)
- DQ retry (10×50ms via QSPI)
- Frame dispatch to CAN/LoRa/ZigBee/RS485
- Config query response

**mcu_wan_handler_uplink.c** (Priority 5):
- Handshake with WAN MCU (includes firmware version)
- Process uplink queue (50 items)
- QSPI transmission + ACK (200ms timeout)
- RTC request (1s interval)
- SD card retry when online
- FOTA coordination: Send version on handshake

**storage_handler.c**:
- Thread-safe SD operations
- FIFO queue: /sdcard/QUEUE/PKT_NNNNNNNN.dat
- Batch writes (reduced fsync overhead)
- Power-cycle recovery via directory scan

**wan_comm.c** (QSPI Master):
- QIO mode: SPI_TRANS_MODE_QIO
- 40 MHz, 4-bit parallel
- DMA buffers: 16KB TX/RX
- Full-duplex transactions

### 4.2. WAN MCU

**mcu_lan_handler_uplink.c** (Priority 6):
- QSPI slave receive loop
- Frame dispatch: CF/DT/DQ
- Forward to MQTT/HTTP
- RTC response (PCF8563)
- Handshake response (includes firmware version)
- FOTA coordination: Compare versions, trigger update or abort

**mcu_lan_handler_downlink.c** (No Task):
- Downlink queue (20 items)
- GPIO8 data-ready signaling
- Config cache management
- Async config request (semaphore-based)

**lan_comm.c** (QSPI Slave):
- QIO mode reception
- DMA buffers: 8KB TX/RX
- TX buffer flush: disable→write→enable
- Hardware-driven QIO

---

## 5. TASK ARCHITECTURE

### 5.1. LAN MCU - Dual Task

**Downlink Poll Task** (Priority 7):
```

Block on GPIO46 ISR → Acquire g_qspi_mutex → Retry DQ (10×50ms)
→ Parse frame → Dispatch → Release mutex → Block
Total: <5ms ISR response, 500ms max retry

```

**Uplink Handler Task** (Priority 5):
```

Phase 1: Handshake (blocking until success)
- Send: [CF][0x01][fw_version(4)]
- Receive: [ACK][0x10][internet_flag][wan_fw_version(4)]
- Compare versions for FOTA coordination
Phase 2: Loop {
Queue check → Build DT packet → QSPI send (0.5ms)
→ Wait ACK (200ms) → On fail: storage_save()
RTC request (1s interval)
SD retry (if online + has_data)
}
Mutex timeout: 50ms (yields gracefully)

```

### 5.2. WAN MCU - Single Task

**Uplink Processor** (Priority 6):
```

Pre-queue RX → Block on completion (100ms) → Parse frame
→ Dispatch:
CF (handshake + version check/RTC)
DT (forward to server)
DQ (check pending)
→ Re-queue transaction

```

**Downlink** (No Task): MQTT calls mcu_lan_enqueue_downlink() → signal GPIO8

---

## 6. COMMUNICATION PROTOCOL

### 6.1. Frame Types

| Header | Value | Direction | QSPI Time (512B) | Purpose |
|--------|-------|-----------|------------------|---------|
| CF | 0x4346 | LAN → WAN | ~0.1 ms | Command |
| DT | 0x4454 | Bidirectional | ~0.5 ms | Data (vs 2ms standard SPI) |
| DQ | 0x4451 | LAN → WAN | ~0.05 ms | Query slave |
| CQ | 0x4351 | Bidirectional | ~0.2 ms | Config |
| RT | ASCII | WAN → LAN | ~0.05 ms | RTC status |
| ACK | 0x02 | WAN → LAN | ~0.05 ms | Acknowledgment |

### 6.2. Key Packets

**Handshake with Version** (Enhanced for FOTA):
```

Request (LAN → WAN):
[CF][0x01][fw_version(4 bytes)]
fw_version format: uint32_t (e.g., 0x01020304 = v1.2.3.4)

Response (WAN → LAN):
[ACK][0x10][internet_flag][wan_fw_version(4 bytes)]
Total: 6 bytes

Version Comparison Logic (WAN MCU):
if (received_lan_version > cached_lan_version):
cached_lan_version = received_lan_version
trigger_fota = true
proceed with FOTA update
else:
trigger_fota = false
inform_server("FOTA_ABORTED: No new version detected")

```

**FOTA Flow**:
1. LAN MCU completes FOTA → Reboots with new version
2. LAN sends handshake with new version: `[CF][0x01][0x01020305]`
3. WAN receives, compares: `0x01020305 > 0x01020304` → **Proceed with WAN FOTA**
4. WAN MCU performs OTA update
5. If comparison fails: WAN sends MQTT message `{"event":"fota_aborted","reason":"version_mismatch"}`

**Data**: `[DT][handler_type(3)][length(2)][payload]`
- Handler types: CAN, LOR, ZIG, RS4

**RTC**: `[CF][RT]` → Response: `[RT][dd/mm/yyyy-hh:mm:ss][status]`

**Config**: `[CF][CQ]` → Response: `[CQ][length(2)][key=value|key=value|...]`

**ACK Codes**:
- 0x10: Handshake OK
- 0x11: Data received
- 0x12: Internet online
- 0x13: Internet offline

---

## 7. FOTA COORDINATION MECHANISM

### 7.1. FOTA Sequence Diagram

```

Server                WAN MCU              LAN MCU
│                      │                    │
├─FOTA command─────►   │                    │
│  (LAN firmware)      │                    │
│                      ├─FOTA notify────►   │
│                      │  [CF][CFFW][data]  │
│                      │                    │
│                      │                ┌───┴───┐
│                      │                │ FOTA  │
│                      │                │Update │
│                      │                │Reboot │
│                      │                └───┬───┘
│                      │                    │
│                      │  ◄───Handshake─────┤
│                      │  [CF][0x01][v1.2.3.5]
│                      │                    │
│                      ├──Check version──►  │
│                      │  v1.2.3.5 > v1.2.3.4?
│                      │  YES → Continue    │
│                      │                    │
│                      ├─Response────────►  │
│                      │ [ACK][0x10][flag][v1.2.3.4]
│                      │                    │
│                  ┌───┴───┐                │
│                  │ WAN   │                │
│                  │ FOTA  │                │
│                  │Update │                │
│                  │Reboot │                │
│                  └───┬───┘                │
│                      │                    │
│  ◄────FOTA OK────────┤                    │
│   MQTT: {"status":"success","lan_ver":"1.2.3.5","wan_ver":"1.2.3.5"}
│                      │                    │

```

### 7.2. FOTA Abort Scenario

```

Server                WAN MCU              LAN MCU
│                      │                    │
├─FOTA command─────►   │                    │
│  (LAN firmware)      │                    │
│                      ├─FOTA notify────►   │
│                      │                    │
│                      │                ┌───┴───┐
│                      │                │ FOTA  │
│                      │                │Failed │
│                      │                │Reboot │
│                      │                └───┬───┘
│                      │                    │
│                      │  ◄───Handshake─────┤
│                      │  [CF][0x01][v1.2.3.4]
│                      │  (Same version)    │
│                      │                    │
│                      ├──Check version──►  │
│                      │  v1.2.3.4 == v1.2.3.4?
│                      │  NO NEW → Abort    │
│                      │                    │
│  ◄────FOTA FAIL──────┤                    │
│   MQTT: {"status":"aborted","reason":"no_version_change","lan_ver":"1.2.3.4"}
│                      │                    │

```

### 7.3. Version Storage

**LAN MCU**:
```c
// Stored in NVS (non-volatile storage)
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 3
#define FW_VERSION_BUILD 4

uint32_t g_lan_fw_version = (MAJOR << 24) | (MINOR << 16) | (PATCH << 8) | BUILD;
```

**WAN MCU**:

```c
// Cached from last handshake
uint32_t g_cached_lan_version = 0x01020304; // Last known LAN version
uint32_t g_wan_fw_version = 0x01020304;     // Current WAN version

bool check_fota_required(uint32_t received_lan_version) {
    if (received_lan_version > g_cached_lan_version) {
        g_cached_lan_version = received_lan_version;
        ESP_LOGI(TAG, "New LAN version detected: 0x%08X, triggering WAN FOTA", received_lan_version);
        return true;
    } else {
        ESP_LOGW(TAG, "No version change (0x%08X), FOTA aborted", received_lan_version);
        return false;
    }
}
```


### 7.4. FOTA Server Notification

**MQTT Topic**: `device/{device_id}/fota/status`

**Success Payload**:

```json
{
  "event": "fota_complete",
  "timestamp": "2026-01-30T07:42:00Z",
  "lan_version": "1.2.3.5",
  "wan_version": "1.2.3.5",
  "status": "success"
}
```

**Abort Payload**:

```json
{
  "event": "fota_aborted",
  "timestamp": "2026-01-30T07:42:00Z",
  "reason": "no_version_change",
  "lan_version": "1.2.3.4",
  "expected_version": "1.2.3.5",
  "status": "failed"
}
```

**Alternative Protocols**:

- HTTPS POST: `https://api.server.com/devices/{device_id}/fota/status`
- CoAP POST: `coap://server.com/fota/status`

---

## 8. DATA FLOW

### 8.1. Uplink (LAN → WAN → Server)

```
Handler → mcu_wan_enqueue_uplink() → Queue → Uplink task
→ Build DT → QSPI send (0.5ms) → Wait ACK (200ms)
→ If OK: success | If fail: storage_save()
WAN receives → Parse → server_handler_enqueue_uplink() → MQTT
Total latency: 10-20ms (vs 64-155ms standard SPI)
```


### 8.2. Downlink (Server → WAN → LAN)

```
MQTT → mcu_lan_enqueue_downlink() → signal_data_ready()
→ GPIO46 ISR → Downlink task wakes → DQ retry (10×50ms)
→ QSPI read (0.5ms) → Dispatch to handler
Total latency: <5ms (vs 50-100ms standard SPI)
```


### 8.3. SD Backup/Recovery

```
Offline: uplink fails → storage_handler_save() → /sdcard/QUEUE/PKT_*.dat
Online: storage_handler_read_oldest() → QSPI send → ACK OK → delete
QSPI advantage: 50 packets/sec (vs 10 packets/sec standard SPI)
```


---

## 9. PERFORMANCE CHARACTERISTICS

### 9.1. QSPI vs Standard SPI

| Metric | Standard SPI | QSPI | Improvement |
| :-- | :-- | :-- | :-- |
| Clock | 10 MHz | 40 MHz | 4× |
| Data width | 1-bit | 4-bit | 4× |
| Throughput | ~2 MB/s | ~8 MB/s | 4× |
| Uplink latency | 64-155ms | 10-20ms | 6-8× |
| Downlink latency | 50-100ms | <5ms | 10-20× |
| SD recovery rate | 10 pkt/s | 50 pkt/s | 5× |
| Handshake time | 50-100ms | <10ms | 5-10× |

### 9.2. Timing Parameters

| Parameter | Value | Location | Notes |
| :-- | :-- | :-- | :-- |
| QSPI Clock | 40 MHz | wan_comm.c, lan_comm.c | Mandatory |
| GPIO ISR Response | <5 ms | mcu_wan_handler_downlink.c | ISR → data received |
| DQ Retry Interval | 50 ms | mcu_wan_handler_downlink.c | 10 retries max |
| ACK Wait Timeout | 200 ms | mcu_wan_handler_uplink.c | Reduced from 1000ms |
| RTC Update | 1000 ms | mcu_wan_handler_uplink.c | Periodic |
| QSPI Mutex Timeout | 50 ms | mcu_wan_handler_uplink.c | Non-blocking |
| Handshake w/ Version | <10 ms | mcu_wan_handler_uplink.c | Includes version check |


---

## 10. SYNCHRONIZATION

### 10.1. Mutexes

**LAN MCU**:

- `g_qspi_mutex`: Binary semaphore, protects QSPI bus
- `g_storage_mutex`: Protects SD operations
- `g_rtc_mutex`: Protects RTC cache

**WAN MCU**:

- `g_config_mutex`: Protects config cache
- `g_downlink_queue_mutex`: Protects downlink queue
- `g_version_mutex`: Protects cached version variables


### 10.2. Queues

- LAN uplink: 50 items × 532B = 26 KB
- WAN downlink: 20 items × 1030B = 20 KB

**QSPI Benefit**: Faster drain rate (4× throughput), lower overflow risk

---

## 11. ERROR HANDLING

### 11.1. QSPI Errors

**LAN MCU**:

- DQ retry exhaustion (10×50ms): Log, continue (non-fatal)
- Send failure: Fallback to storage_handler_save()
- ACK timeout (200ms): Assume offline, save to SD
- DMA error: Check buffer alignment, retry

**WAN MCU**:

- Parse error: Log, discard, re-queue
- TX load failure: Log, skip transmission
- Slave overrun: Increase buffer size


### 11.2. SD Card Errors

- Mount failure: Log, continue without backup (non-fatal)
- Write failure: Check directory, auto-delete corrupt files
- Read failure: Delete corrupt file, scan next
- Power cycle: scan_and_update_counters() recovers state


### 11.3. FOTA Errors

**Version Mismatch**:

- WAN detects no version change → Abort FOTA
- Send MQTT notification to server
- Log error with expected vs actual version

**Handshake Timeout**:

- LAN MCU not responding after FOTA → WAN continues with old version
- Inform server: `{"status":"lan_unreachable"}`

**FOTA Download Failure** (WAN MCU):

- OTA partition write error → Rollback to previous version
- Inform server: `{"status":"download_failed","reason":"ota_write_error"}`

---

## 12. MEMORY USAGE

**LAN MCU**: ~64 KB heap (QSPI buffers 16KB + uplink queue 26KB + storage 4KB + SD cache 4KB) + 8 KB stack

**WAN MCU**: ~33 KB heap (QSPI buffers 8KB + downlink queue 20KB) + 4 KB stack

---

## 13. KEY DESIGN DECISIONS

1. **QSPI Mandatory**: 40 MHz, 4-bit, QIO mode for 4× throughput
2. **Modular Separation**: Uplink/downlink split for clarity and independent scaling
3. **Storage Abstraction**: storage_handler.c wraps SDCard_comm with thread safety
4. **Priority-Based Preemption**: Downlink (7) > Uplink (5) for <5ms ISR response
5. **Reduced Timeouts**: ACK 200ms (vs 1s), DQ retry 50ms (vs 150ms) due to QSPI speed
6. **SD FIFO Queue**: Sequential files for power-cycle recovery
7. **Async Config Request**: Heap-allocated + semaphore for timeout handling
8. **No ACK on Downlink**: GPIO ISR + DQ polling (stateless, high speed)
9. **FOTA Version Coordination**: Handshake includes firmware versions, WAN compares and decides
10. **FOTA Abort Notification**: Server informed via MQTT/HTTPS/CoAP on version mismatch

---

## 14. MIGRATION FROM STANDARD SPI

**Hardware**: Change SPI pins to GPIO10-15 (QSPI), keep SD on GPIO3-8

**Software**:

- wan_comm.c: Add `SPI_TRANS_MODE_QIO`, increase clock to 40 MHz
- lan_comm.c: Configure slave for QIO reception
- Reduce timeouts: ACK 200ms, DQ retry 50ms
- Increase DMA buffers: 16KB (master), 8KB (slave)
- Add version fields to handshake packet (4 bytes each side)
- Implement version comparison logic in WAN MCU
- Add MQTT/HTTPS/CoAP notification for FOTA status

**Testing**: Verify 4× throughput improvement, <5ms ISR latency, FOTA coordination flow

---
---

## 15. BUFFER MANAGEMENT STRATEGY

### 15.1. SD Card Write Buffer (100 KB)

**Purpose**: Minimize SD write operations via buffering, maximize throughput

**Buffer Parameters**:
- Size: 102400 bytes (100 KB)
- Alignment: 512-byte sectors (SD block size)
- Location: LAN MCU PSRAM or DRAM (static allocation)

**Packet Accumulation Logic**:
```c
typedef struct {
    uint8_t buffer[102400];
    size_t used;
    uint32_t packet_count;
} sd_write_buffer_t;

static sd_write_buffer_t g_sd_buffer = {0};

esp_err_t sd_buffer_add_packet(const uint8_t *data, size_t len) {
    // Check if packet fits in remaining space
    if (g_sd_buffer.used + len <= 102400) {
        memcpy(&g_sd_buffer.buffer[g_sd_buffer.used], data, len);
        g_sd_buffer.used += len;
        g_sd_buffer.packet_count++;
        return ESP_OK;
    } else {
        // Buffer full/near full - pad and flush
        size_t padding = 102400 - g_sd_buffer.used;
        memset(&g_sd_buffer.buffer[g_sd_buffer.used], 0xFF, padding); // Dummy bytes
        g_sd_buffer.used = 102400;

        // Flush to SD card
        storage_handler_write_block(g_sd_buffer.buffer, 102400);

        // Start new buffer with current packet
        memcpy(g_sd_buffer.buffer, data, len);
        g_sd_buffer.used = len;
        g_sd_buffer.packet_count = 1;
        return ESP_OK;
    }
}
```

**Flush Conditions**:
1. Buffer reaches 100 KB (exact or with padding)
2. Timeout: 5 seconds idle (no new packets)
3. Critical event: Power loss, reset command

**Dummy Byte Format**: `0xFF` (SD card erase state)

**Performance**:
- Reduces write operations: ~200 packets/flush (assuming 500B avg packet)
- Write latency: 100 KB @ 20 MB/s = 5ms
- Avoids partial packet reconstruction on read

---

### 15.2. DMA Buffer Constraint (4 KB)

**Hardware Limitation**: ESP32 SPI DMA max descriptor = 4092 bytes (4 KB - 4 bytes)

**Both MCUs** (LAN Master, WAN Slave):
- QSPI TX/RX DMA buffer: 4096 bytes maximum
- Prevents DMA descriptor chain overflow

**Packet Transmission Logic**:
```c
#define DMA_BUFFER_SIZE 4096

typedef struct {
    uint8_t buffer[DMA_BUFFER_SIZE];
    size_t used;
} dma_tx_buffer_t;

static dma_tx_buffer_t g_dma_tx = {0};

esp_err_t dma_buffer_add_frame(const uint8_t *frame, size_t len) {
    // Check if frame fits
    if (g_dma_tx.used + len <= DMA_BUFFER_SIZE) {
        memcpy(&g_dma_tx.buffer[g_dma_tx.used], frame, len);
        g_dma_tx.used += len;
        return ESP_OK;
    } else {
        // DMA buffer full - pad and transmit
        size_t padding = DMA_BUFFER_SIZE - g_dma_tx.used;
        memset(&g_dma_tx.buffer[g_dma_tx.used], 0x00, padding); // Dummy bytes
        g_dma_tx.used = DMA_BUFFER_SIZE;

        // Execute QSPI transaction
        spi_transaction_t t = {
            .length = DMA_BUFFER_SIZE * 8,
            .tx_buffer = g_dma_tx.buffer,
            .flags = 0
        };
        spi_device_transmit(g_qspi_handle, &t);

        // Start new buffer with current frame
        memcpy(g_dma_tx.buffer, frame, len);
        g_dma_tx.used = len;
        return ESP_OK;
    }
}
```

**Key Differences from SD Buffer**:
- Dummy byte: `0x00` (QSPI no-op, ignored by receiver)
- Receiver parser: Skip `0x00` padding until valid frame header
- Flush trigger: DMA buffer full only (no timeout)

**Receiver Frame Parser**:
```c
void parse_dma_buffer(const uint8_t *buf, size_t len) {
    size_t offset = 0;
    while (offset < len) {
        // Skip dummy bytes
        while (offset < len && buf[offset] == 0x00) {
            offset++;
        }
        if (offset >= len) break;

        // Parse frame header
        uint16_t header = (buf[offset] << 8) | buf[offset+1];
        if (header == 0x4454 || header == 0x4346 || header == 0x4451) {
            // Valid frame - extract length and process
            uint16_t frame_len = extract_frame_length(&buf[offset]);
            process_frame(&buf[offset], frame_len);
            offset += frame_len;
        } else {
            offset++; // Skip invalid byte
        }
    }
}
```

**Performance Impact**:
- Padding overhead: < 0.1% (typical frame 512B, padding ~100B per 8 frames)
- No DMA descriptor fragmentation
- Receiver CPU: < 1% for dummy byte filtering

---

### 15.3. Buffer Coordination

**Layering**:
```
Protocol Layer (DT/CF frames) → DMA Buffer (4KB) → QSPI Hardware
                               ↓
                         SD Buffer (100KB) → SD Card (LAN MCU only)
```

**Independence**:
- DMA buffer: Transmission-level, both MCUs
- SD buffer: Storage-level, LAN MCU only
- No interaction between buffers (separate concerns)

**Example Flow** (LAN MCU Uplink):
1. CAN handler → `mcu_wan_enqueue_uplink()` → Queue
2. Uplink task → Build DT frame → `dma_buffer_add_frame()`
3. If DMA full → Pad + QSPI transmit
4. If ACK timeout → `sd_buffer_add_packet()` (full frame)
5. If SD buffer full → Pad + Write to SD

**Memory Allocation**:
```c
// LAN MCU
static sd_write_buffer_t g_sd_buffer __attribute__((section(".psram")));  // 100 KB PSRAM
static dma_tx_buffer_t g_dma_tx;  // 4 KB DRAM (DMA requirement)

// WAN MCU
static dma_tx_buffer_t g_dma_tx;  // 4 KB DRAM
// No SD buffer
```

---
