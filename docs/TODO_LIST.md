# 📋 TODO LIST - Complete Code Fixes & Upgrades

**Ngày tạo:** 26/01/2026  
**Last Updated:** 26/01/2026 14:26  
**Dựa trên:**
- [CODE_REVIEW_REPORT.md](CODE_REVIEW_REPORT.md) - 25 issues
- Phase 1 Implementation (COMPLETED)

**Tiến độ tổng thể:** 9/26 tasks (35%) ✅

---

## 📊 PROGRESS OVERVIEW

```
┌──────────────────────────────────────────────────┐
│  CATEGORY          │  DONE  │  TODO  │  TOTAL   │
├────────────────────┼────────┼────────┼──────────┤
│ 🔴 CRITICAL        │  9/11  │   2    │    11    │
│ 🟠 HIGH PRIORITY   │  0/8   │   8    │     8    │
│ 🟡 MEDIUM          │  0/5   │   5    │     5    │
│ 🟢 LOW             │  0/2   │   2    │     2    │
├────────────────────┼────────┼────────┼──────────┤
│ TOTAL              │  9/26  │  17    │    26    │
└──────────────────────────────────────────────────┘

Timeline: Phase 1: ✅ DONE | Phase 3-6: 8-10 tuần
```

---

## 🔥 PHASE 0: IMMEDIATE FIXES ✅ COMPLETE

> ✅ **Status:** COMPLETE (2026-01-26)  
> **Time Taken:** 30 minutes  

### ✅ **Checkpoint 0.1: Security Critical**  
**Completed:** 2026-01-26

- [x] **T0.1** ✅ Remove hardcoded WiFi credentials
  - File: `wifi_connect.c`
  - **Completed:** 2026-01-26

- [x] **T0.2** ✅ Fix buffer bounds checking
  - File: `config_handler.c`
  - **Completed:** 2026-01-26

- [x] **T0.3** ✅ Review and test current safety checks
  - **Completed:** 2026-01-26

---

## 🚀 PHASE 1: THREAD-SAFETY & MEMORY ✅ COMPLETE

> ✅ **Status:** COMPLETE (2026-01-26 14:05)  
> **Time Taken:** ~2 hours  
> **Report:** [walkthrough.md](../.gemini/antigravity/brain/b49047cb-5a8c-42a0-9ea7-f2dfe82da0d7/walkthrough.md)

### ✅ **Checkpoint 1.1: Config Handler Thread-Safety**
**Completed:** 2026-01-26

- [x] **T1.1** ✅ Implement config context mutex
  - File: `config_handler.c`
  - Added: `g_config_context_mutex`
  - **Completed:** 2026-01-26

- [x] **T1.2** ✅ Implement thread-safe config access functions
  - Added 6 safe functions (WiFi/LTE/MQTT get/update)
  - **Completed:** 2026-01-26

- [x] **T1.3** ✅ Update UART/USB handlers to use safe functions
  - Files: `uart_handler.c`, `usb_handler.c`
  - **Completed:** 2026-01-26

- [x] **T1.4** ✅ Add mutex for WiFi reconnect
  - File: `wifi_connect.c`
  - Added: `g_wifi_reconfig_mutex`
  - **Completed:** 2026-01-26

### ✅ **Checkpoint 1.2: Memory Management**
**Completed:** 2026-01-26

- [x] **T1.5** ✅ Implement buffer pool for CAN handler
  - File: `can_handler.c` (DA2_esp_LAN)
  - Static pool: 10 x 256 bytes
  - **Completed:** 2026-01-26

- [x] **T1.6** ✅ Fix memory cleanup in error paths
  - File: `lan_comm.c`
  - Added cleanup macro
  - **Completed:** 2026-01-26

- [x] **T1.7** ✅ Fix semaphore leaks
  - File: `mcu_lan_handler.c`
  - Cleanup label pattern
  - **Completed:** 2026-01-26

### ✅ **Checkpoint 1.3: ISR Safety**
**Completed:** 2026-01-26

- [x] **T1.8** ✅ Atomic operations for ISR counters
  - File: `mcu_wan_handler.c` (DA2_esp_LAN)
  - Used `__atomic_fetch_add()`
  - **Completed:** 2026-01-26

- [x] **T1.9** ✅ Remove logging from ISR
  - Moved logging to task context
  - **Completed:** 2026-01-26

---

## 🧹 PHASE 3: CODE QUALITY (2-3 ngày)

> Refactoring, cleanup, and documentation improvements

### **Checkpoint 3.1: Refactoring**
**Deadline:** Tuần 3-4  
**Estimated Time:** 2 ngày

- [ ] **T3.1** Extract common CFSC handling
  - Create: `common/config_response_formatter.c`
  - Extract shared code from uart_handler.c and usb_handler.c
  - Implement callback-based formatter
  - **Priority:** 🟡 MEDIUM
  - **Time:** 1 ngày
  - **Issue:** CODE_REVIEW_REPORT.md #11

- [ ] **T3.2** Task cleanup on stop
  - File: `config_handler.c`
  - Delete queues in `config_handler_task_stop()`
  - Delete mutexes properly
  - **Priority:** 🟡 MEDIUM
  - **Time:** 2 giờ
  - **Issue:** CODE_REVIEW_REPORT.md #15

- [ ] **T3.3** Add timeout handling
  - File: `lan_comm.c`
  - Replace `portMAX_DELAY` with `pdMS_TO_TICKS(5000)`
  - Add timeout error handling
  - **Priority:** 🟡 MEDIUM
  - **Time:** 3 giờ
  - **Issue:** CODE_REVIEW_REPORT.md #17

### **Checkpoint 3.2: Documentation**
**Deadline:** Tuần 4  
**Estimated Time:** 1 ngày

- [ ] **T3.4** Add inline comments for complex logic
  - Document SPI communication protocol
  - Document state machines
  - **Priority:** 🟢 LOW
  - **Time:** 4 giờ
  - **Issue:** CODE_REVIEW_REPORT.md #25

- [ ] **T3.5** Update documentation files
  - Reflect actual implementation status
  - Clean up outdated claims
  - **Priority:** 🟢 LOW
  - **Time:** 1 giờ

---

## 🚀 PHASE 4: QSPI REFACTORING (3-4 tuần)

> Major architecture upgrade - QSPI communication with DMA

### **Checkpoint 4.1: QSPI Driver Development**
**Deadline:** Tuần 5-6  
**Estimated Time:** 2 tuần

- [ ] **T4.1** Create QSPI BSP driver structure
  - Create: `BSP/QSPI_Communication/include/qspi_comm.h`
  - Create: `BSP/QSPI_Communication/src/qspi_comm.c`
  - Define API structs and enums
  - **Priority:** 🔴 CRITICAL (Phase 2.1)
  - **Time:** 1 ngày
  - **Design:** QSPI_SDCARD_DESIGN.md

- [ ] **T4.2** Implement QSPI initialization
  - GPIO configuration (CS, CLK, IO0-3, DR signals)
  - SPI bus config (Quad mode, 40MHz)
  - DMA setup
  - **Priority:** 🔴 CRITICAL
  - **Time:** 3 ngày

- [ ] **T4.3** Implement DMA transfer functions
  - `qspi_transmit_async()`
  - `qspi_receive_async()`
  - DMA callbacks
  - **Priority:** 🔴 CRITICAL
  - **Time:** 2 ngày

- [ ] **T4.4** Implement GPIO interrupt handling
  - DR signal ISR
  - Task notification
  - **Priority:** 🟠 HIGH
  - **Time:** 1 ngày

- [ ] **T4.5** Frame encoder/decoder
  - Create: `qspi_frame.c`
  - CRC calculation
  - Frame validation
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

### **Checkpoint 4.2: Application Layer Integration**
**Deadline:** Tuần 7-8  
**Estimated Time:** 1 tuần

- [ ] **T4.6** Create Uplink/Downlink tasks
  - Implement dual-task model
  - Queue management
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T4.7** Replace old SPI communication
  - Migrate từ `lan_comm.c` / `wan_comm.c`
  - Update handlers
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T4.8** Testing and optimization
  - Bandwidth testing
  - Latency measurements
  - DMA efficiency
  - **Priority:** 🟠 HIGH
  - **Time:** 3 ngày

---

## 💾 PHASE 5: SD CARD STORAGE HANDLER (2-3 tuần)

> Buffered, batch SD card writes for offline data storage

### **Checkpoint 5.1: Storage Handler Implementation**
**Deadline:** Tuần 9-10  
**Estimated Time:** 2 tuần

- [ ] **T5.1** Create Storage Handler structure
  - Create: `DA2_esp_LAN/Application/Storage_Handler/`
  - Define API in `storage_handler.h`
  - **Priority:** 🟠 HIGH
  - **Time:** 1 ngày
  - **Design:** QSPI_SDCARD_DESIGN.md

- [ ] **T5.2** Implement RAM buffer
  - 100KB ring buffer
  - Thread-safe append/read
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T5.3** Implement batch flush logic
  - Threshold detection (100KB)
  - FATFS wrapper
  - File rotation (daily)
  - **Priority:** 🟠 HIGH
  - **Time:** 3 ngày

- [ ] **T5.4** Implement Storage Task
  - Main loop
  - Queue processing
  - Error handling
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T5.5** SD Card BSP driver
  - Create: `DA2_esp_LAN/BSP/SDCard_Communication/`
  - SDMMC configuration (4-line, 40MHz)
  - Mount/unmount helpers
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

### **Checkpoint 5.2: Integration & Testing**
**Deadline:** Tuần 11  
**Estimated Time:** 1 tuần

- [ ] **T5.6** Integrate với MQTT handler
  - Clone data to storage khi online
  - Fallback to storage khi offline
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T5.7** Recovery mode implementation
  - Replay từ SD card khi internet restored
  - **Priority:** 🟡 MEDIUM
  - **Time:** 2 ngày

- [ ] **T5.8** Testing
  - Online/offline scenarios
  - Buffer full handling
  - Write performance (100KB batches)
  - **Priority:** 🟠 HIGH
  - **Time:** 3 ngày

---

## 🧪 PHASE 6: TESTING & VERIFICATION (1-2 tuần)

> Comprehensive testing before production deployment

### **Checkpoint 6.1: Unit Testing**
**Deadline:** Tuần 12  
**Estimated Time:** 1 tuần

- [ ] **T6.1** Thread-safety tests
  - Concurrent config access stress test
  - Mutex deadlock scenarios
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T6.2** Memory stability tests
  - Heap monitoring (24+ giờ)
  - Buffer pool stress test
  - Memory leak detection
  - **Priority:** 🟠 HIGH
  - **Time:** 2 ngày

- [ ] **T6.3** QSPI performance tests
  - Bandwidth measurements
  - Latency testing
  - DMA efficiency
  - **Priority:** 🟡 MEDIUM
  - **Time:** 1 ngày

- [ ] **T6.4** Storage handler tests
  - Write performance (batch vs individual)
  - Offline mode testing
  - Recovery mode testing
  - **Priority:** 🟡 MEDIUM
  - **Time:** 2 ngày

### **Checkpoint 6.2: Integration Testing**
**Deadline:** Tuần 13  
**Estimated Time:** 1 tuần

- [ ] **T6.5** Full system integration test
  - End-to-end data flow
  - Error recovery scenarios
  - **Priority:** 🟠 HIGH
  - **Time:** 3 ngày

- [ ] **T6.6** Update documentation
  - Update all docs to reflect actual implementation
  - Create deployment guide
  - **Priority:** 🟡 MEDIUM
  - **Time:** 2 ngày

---

## 📅 TIMELINE SUMMARY

```
Week 0:    [████████████████████] ✅ PHASE 0: Emergency Fixes (DONE)
Week 1-2:  [████████████████████] ✅ PHASE 1: Thread-Safety (DONE)
Week 3-4:  [                    ] ⏳ PHASE 3: Code Quality
Week 5-8:  [                    ] ⏳ PHASE 4: QSPI Refactoring
Week 9-11: [                    ] ⏳ PHASE 5: Storage Handler
Week 12-13:[                    ] ⏳ PHASE 6: Testing & Verification

Total: ~13 weeks estimated for 100% completion
Current: Week 1-2 complete (Phase 0 + Phase 1)
```

---

## 🎯 MILESTONES

| Milestone | Date | Tasks | Status |
|-----------|------|-------|--------|
| **M0: Emergency Fixes** | Week 0 | T0.1-T0.3 | ✅ DONE |
| **M1: Thread-Safe Core** | Week 2 | T1.1-T1.9 | ✅ DONE |
| **M3: Code Quality** | Week 4 | T3.1-T3.5 | ⏳ TODO |
| **M4: QSPI Complete** | Week 8 | T4.1-T4.8 | ⏳ TODO |
| **M5: Storage Complete** | Week 11 | T5.1-T5.8 | ⏳ TODO |
| **M6: Production Ready** | Week 13 | T6.1-T6.6 | ⏳ TODO |

---

## 📊 TRACKING

### Completion Status Legend:
- ⏳ TODO - Chưa bắt đầu
- 🚧 IN PROGRESS - Đang làm
- ✅ DONE - Hoàn thành
- ⚠️ BLOCKED - Bị chặn
- ❌ CANCELLED - Hủy bỏ

### Update Log:
```
2026-01-26 00:00: Created initial TODO list (0% complete)
2026-01-26 14:05: ✅ Phase 0 complete (3 tasks)
2026-01-26 14:05: ✅ Phase 1 complete (9 tasks) - 35% total progress
2026-01-26 14:26: Updated TODO list (removed Phase 2 per user request)
```

---

## 🎓 GUIDELINES

### Khi bắt đầu task:
1. Update status từ ⏳ → 🚧
2. Add start date vào comment
3. Create branch nếu cần: `feature/T3.1-extract-cfsc`

### Khi complete task:
1. Update status từ 🚧 → ✅
2. Add completion date
3. Update progress counter ở đầu file
4. Test thoroughly
5. Create PR cho review

### Priority xử lý:
1. 🔴 CRITICAL - Làm trước tiên
2. 🟠 HIGH - Quan trọng, làm sau CRITICAL
3. 🟡 MEDIUM - Có thể đợi
4. 🟢 LOW - Nice to have

---

## 📞 NOTES

### Dependencies:
- ✅ PHASE 1 complete - Thread-safety foundation established
- PHASE 4 (QSPI) requires PHASE 1 (already done)
- PHASE 5 (Storage) can run parallel with PHASE 4
- PHASE 3 can be done anytime

### Risks:
- QSPI refactoring là breaking change lớn - cần testing kỹ
- Storage handler yêu cầu SD card hardware - verify availability
- Memory constraints trên ESP32-S3 - monitor heap usage

### What's Next:
Recommend starting with **Phase 3** (Code Quality) as quick wins:
- T3.2: Task cleanup (2 giờ)
- T3.3: Timeout handling (3 giờ)
- T3.1: CFSC refactor (1 ngày)

Or jump to **Phase 4** (QSPI) if hardware-ready for major upgrade.

---

**Created:** 2026-01-26  
**Last Updated:** 2026-01-26 14:26  
**Version:** 2.0 (Phase 2 removed, Phase 1 completed)  
**Owner:** Development Team
