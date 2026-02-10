# DA2_esp_LAN - Code Analysis Report

Ngày báo cáo: 08/02/2026

---

## 1. Middleware/BLE_Handler/include/ble_handler.h

### Tóm tắt
File header định nghĩa BLE Handler Middleware - lớp vận chuyển (transportation layer) cho gateway BLE. Cung cấp giao diện transparent cho dòng dữ liệu hai chiều giữa các thiết bị BLE và server qua WAN MCU. Tổng cộng 20 hàm BLE được định nghĩa (15 cơ bản + 5 tùy chọn) để xử lý reset, cấu hình, kết nối, và truyền dữ liệu.

### Các hàm/struct chính
1. **ble_function_id_t** - Enum 20 hàm BLE (HW_RESET, SW_RESET, GET_INFO, SET_NAME, START_BROADCAST, CONNECT, DISCONNECT, GET_CONNECTION_STATUS, ENTER_SLEEP, WAKEUP, START_DISCOVERY, SEND_DATA, GET_DIAGNOSTICS, SET_SECURITY, MANAGE_WHITELIST)
2. **ble_device_t** - Struct lưu trữ trạng thái thiết bị BLE (MAC, tên, RSSI, thời gian hoạt động)
3. **ble_function_config_t** - Struct cấu hình AT command với GPIO control trước/sau
4. **ble_module_config_t** - Cấu hình module BLE toàn phần (metadata + 20 function configs)
5. **ble_handler_init()** - Khởi tạo middleware
6. **ble_handler_load_config()** - Nạp cấu hình JSON cho module BLE
7. **ble_handler_execute_function()** - Thực thi hàm BLE nội bộ (dùng cho task layer)
8. **ble_handler_hw_reset()** - Reset cứng qua GPIO
9. **ble_handler_connect()** - Kết nối đến thiết bị BLE
10. **ble_handler_send_data()** - Gửi dữ liệu trong transparent mode

### Missing/Incomplete code
- Không có cơ chế quản lý kết nối tự động (automatic connection recovery) (duyệt)
- Không xử lý timeout cho từng hàm riêng lẻ (chỉ có timeout chung cho response) (loại)

---

## 2. Middleware/BLE_Handler/src/ble_handler.c

### Tóm tắt
Triển khai BLE Handler Middleware với đầy đủ 20 hàm BLE. Thực thi AT commands và GPIO sequences được định nghĩa trong JSON config. Sử dụng Module_Config_Controller để gửi command qua UART/SPI/I2C, kiểm tra response và kiểm soát GPIO. Hỗ trợ 2 stack BLE độc lập với cấu hình riêng.

### Các hàm/struct chính
1. **g_ble_handler** - Global state lưu cấu hình cho 2 stack
2. **ble_is_valid_stack_id()** - Kiểm tra stack ID (0 hoặc 1)
3. **ble_get_function_config()** - Lấy config hàm theo ID
4. **ble_execute_function_internal()** - Thực thi hàm bên trong (7 bước: GPIO start, delay, send command, recv response, verify, GPIO end, delay)
5. **ble_handler_load_config()** - Parse JSON config từ json_ble_config_parser và lưu
6. **ble_handler_hw_reset()** đến **ble_handler_manage_whitelist()** - Implementations cho 20 hàm
7. **Wrapper layer** - Gọi module_gpio_write, module_bus_write, module_bus_read từ Module_Config_Controller

### Missing/Incomplete code
- Không kiểm tra sanity check cho command string (có thể buffer overflow nếu command quá dài) (duyệt)
- Thiếu cơ chế retry tự động khi command timeout (chỉ trả về lỗi ngay lập tức) (loại)
- Không log chi tiết về GPIO states trước/sau execution (loại)

---

## 3. Middleware/JSON_Config_Parser/include/json_ble_config_parser.h

### Tóm tắt
Header cho BLE-specific JSON parser - xử lý cấu hình JSON với tên hàm hardcoded cho 20 hàm BLE (15 core + 5 optional). Sử dụng common parser cho metadata, sau đó parse các hàm BLE cụ thể với validation tên hàm.

### Các hàm/struct chính
1. **json_ble_function_id_t** - Enum 20 hàm BLE (HW_RESET=0 đến MANAGE_WHITELIST=19)
2. **json_ble_function_config_t** - Struct cấu hình hàm BLE (command, GPIO arrays, delay, timeout, expect_response)
3. **json_ble_module_config_t** - Cấu hình module hoàn chỉnh (metadata + array 20 functions)
4. **json_ble_config_parse()** - Parser JSON thành json_ble_module_config_t
5. **BLE_FUNCTION_NAMES[]** - Array hardcoded tên hàm ("MODULE_HW_RESET", "MODULE_CONNECT", v.v.)
6. **BLE_MAX_FUNCTIONS** = 20

### Missing/Incomplete code
- Không validate giá trị command (có thể chứa ký tự không hợp lệ) (loại)
- Không kiểm tra xung đột tên hàm duplicate (trong JSON) (loại)
- Thiếu kiểm tra GPIO pin ID validity (chỉ lưu string, không validate format) (duyệt)

---

## 4. Middleware/JSON_Config_Parser/src/json_ble_config_parser.c

### Tóm tắt
Triển khai BLE JSON parser - phân tích chuỗi JSON để lấy metadata (module type, communication port, baudrate) và 20 function definitions. Ánh xạ tên hàm string sang enum ID, parse GPIO control arrays và timeout values. Sử dụng cJSON library để xử lý JSON.

### Các hàm/struct chính
1. **BLE_FUNCTION_NAMES[]** - Mapping tên hàm (static const)
2. **get_function_id()** - Lookup enum từ tên string
3. **parse_gpio_array()** - Parse GPIO start/end control arrays
4. **parse_function()** - Parse một function entry JSON
5. **json_ble_config_parse()** - Main parser function
6. **cJSON** - Library cJSON cho parsing

### Missing/Incomplete code
- Không handle JSON parsing errors chi tiết (generic ESP_ERR_INVALID_ARG) (duyệt)
- Thiếu validation cho values (GPIO pins, timeouts, baudrates) (loại)
- Không log parsed values cho debugging (loại)
- Không validate command format (ASCII vs binary) (duyệt, thêm vào giả sử command non ASCII( thì ở dạng 0x... ví dụ 0xC0 0xC0 0xC0 cho một command chẳng hạn), command ở dạng string như AT và non AT nhưng ở dạng ASCII)

---

## 5. Middleware/Module_Config_Controller/include/module_config_controller.h

### Tóm tắt
Module Config Controller - Helper wrapper layer cung cấp API đơn giản để truy cập BSP. Cấp hai hàm wrapper cho: (1) Bus communication (UART/SPI/I2C/USB write/read), (2) GPIO control (single/multiple pins). BLE_Handler và các middleware khác gọi các hàm này thay vì gọi BSP trực tiếp.

### Các hàm/struct chính
1. **module_config_controller_init()** - Khởi tạo controller
2. **module_config_controller_init_uart/spi/i2c/usb()** - Init comm port cho stack
3. **module_config_controller_deinit_uart/spi/i2c/usb()** - Deinit comm port
4. **module_bus_write()** - Ghi dữ liệu qua bus được cấu hình
5. **module_bus_read()** - Đọc dữ liệu qua bus với timeout
6. **module_gpio_write()** - Ghi 1 pin GPIO
7. **module_gpio_write_multi()** - Ghi nhiều pins GPIO (batched)

### Missing/Incomplete code
- Không có error recovery nếu comm port initialization fails (duyệt)
- Thiếu API để query port type hiện tại của stack (loại)
- Không support dynamic port switching (phải reinitialize) (loại)

---

## 6. Middleware/Module_Config_Controller/src/module_config_controller.c

### Tóm tắt
Triển khai Module Config Controller - cấp lớp wrapper giữa Middleware (BLE_Handler, etc.) và BSP (Module_UART_Communication, Module_SPI_Communication, v.v.). Quản lý handles cho 4 loại comm port (UART, SPI, I2C, USB) trên 2 stacks, cung cấp API điều khiển GPIO thông qua stack_handler. Hỗ trợ batched GPIO writes để tối ưu I2C transactions.

### Các hàm/struct chính
1. **stack_handles_t** - Struct lưu 4 comm handles cho 1 stack
2. **g_stack_handles[2]** - Global array handles cho 2 stacks
3. **module_bus_write()** - Dispatcher ghi dữ liệu (UART: uart_send, SPI: spi_transfer, I2C: i2c_write, USB: usb_send)
4. **module_bus_read()** - Dispatcher đọc dữ liệu với timeout
5. **module_gpio_write()** - Ghi 1 GPIO bằng cách parse "XY" → stack X, pin Y, gọi stack_handler_gpio_write
6. **module_gpio_write_multi()** - Convert GPIO_control_t array → gpio_action_t, gọi stack_handler_gpio_write_multi
7. **Initialization functions** - Gọi BSP init functions với parameters từ json_config_parser

### Missing/Incomplete code
- Không validate stack_id cho tất cả input (loại)
- Thiếu check xem comm port đã initialize chưa trước khi dùng (loại do hàm init luôn phải được gọi trong init) (loại)
- Không có timeout cho I2C transactions (hardcoded timeout) (duyệt)
- Thiếu event handler cho port event như usb, uart event (duyệt)

---

## 7. Application/BLE_Handler/include/ble_handler_task.h

### Tóm tắt
Header cho BLE Handler Task - FreeRTOS task quản lý kết nối thiết bị BLE và định tuyến dữ liệu hai chiều giữa BLE devices và server qua WAN MCU. Task xử lý device discovery, connection management, uplink data (sensor → server), downlink data (server → device), nạp cấu hình JSON từ PC App.

### Các hàm/struct chính
1. **ble_uplink_packet_t** - Dữ liệu từ BLE device (MAC, timestamp, payload)
2. **ble_downlink_packet_t** - Dữ liệu đến BLE device (target MAC, timeout, payload)
3. **ble_handler_task_start()** - Tạo FreeRTOS task cho BLE management
4. **ble_handler_task_stop()** - Dừng task
5. **ble_handler_task_load_config()** - Nạp JSON config từ PC App
6. **ble_handler_task_enqueue_uplink()** - Queue dữ liệu từ BLE lên server
7. **ble_handler_task_enqueue_downlink()** - Queue dữ liệu từ server xuống BLE
8. **ble_handler_task_get_connected_devices()** - Lấy danh sách devices kết nối
9. **ble_handler_task_start_discovery()** - Kích hoạt device scan
10. **ble_handler_task_get_discovered_devices()** - Lấy kết quả scan

### Missing/Incomplete code
- Không có API để query connection status của device cụ thể (từ task layer) (loại)
- Thiếu mechanism để disconnect device từ ngoài (chỉ timeout tự động) (loại)
- Không support multiple BLE modules trên stack khác nhau (duyệt)

---

## 8. Application/BLE_Handler/src/ble_handler_task.c

### Tóm tắt
Triển khai BLE Handler Task - tạo 2 FreeRTOS tasks độc lập (uplink + downlink) để xử lý dòng dữ liệu. Task uplink đọc từ queue, batch dữ liệu, gửi lên WAN MCU. Task downlink đọc lệnh từ server, gửi đến BLE device. Quản lý danh sách devices kết nối, tự động cleanup idle devices, gọi middleware functions qua ble_handler_execute_function().

### Các hàm/struct chính
1. **ble_uplink_task()** - Task xử lý dữ liệu uplink (batch + flush logic)
2. **ble_downlink_task()** - Task xử lý dữ liệu downlink
3. **g_connected_devices[]** - Array lưu devices kết nối (6 MAC, RSSI, timestamps)
4. **g_discovered_devices[]** - Array kết quả scan (6 MAC, RSSI)
5. **ble_find_connected_device()** - Tìm device theo MAC
6. **ble_add_connected_device()** - Thêm device vào danh sách
7. **ble_remove_connected_device()** - Xóa device khỏi danh sách
8. **ble_cleanup_idle_devices()** - Ngắt kết nối devices không hoạt động > 60s
9. **BLE_UPLINK_BATCH_MAX** = 8, **BLE_UPLINK_BATCH_FLUSH_MS** = 50ms
10. **MAX_CONNECTED_DEVICES** = 5, **MAX_DISCOVERED_DEVICES** = 20

### Missing/Incomplete code
- **PLACEHOLDER**: `ble_handler_disconnect(0)` được gọi nhưng không có target device parameter (chỉ hardcoded stack 0)
- **BUG**: Hàm cleanup idle devices không tìm được idle devices vì logic timeout calculation sai (timestamps là TickCount chứ không phải ms) (duyệt)
- Thiếu error handling khi enqueue_uplink fails (data bị mất) (duyệt)
- Không validate MAC address format trong enqueue functions (loại)

---

## 9. BSP/Module_UART_Communication/include/module_uart_comm.h

### Tóm tắt
Header cho Generic UART Communication Driver - cung cấp unified interface để giao tiếp UART với modules. Wraps ESP-IDF UART driver với API đơn giản, thread-safety (mutex), và proper error handling. Hỗ trợ 2 stacks với pin/port hardcoded dựa trên stack_id.

### Các hàm/struct chính
1. **module_uart_comm_handle_t** - Opaque handle pointer
2. **module_uart_config_t** - Struct cấu hình (stack_id, baudrate, parity, stop_bits, buffer sizes)
3. **STACK0_UART_PORT/TX_PIN/RX_PIN** - Hardcoded pins (UART1, TX=17, RX=18)
4. **STACK1_UART_PORT/TX_PIN/RX_PIN** - Hardcoded pins (UART2, TX=15, RX=16)
5. **module_uart_comm_init()** - Khởi tạo UART driver
6. **module_uart_comm_send()** - Gửi dữ liệu với timeout
7. **module_uart_comm_receive()** - Nhận dữ liệu với timeout
8. **module_uart_comm_flush()** - Clear RX buffer
9. **module_uart_comm_available()** - Kiểm tra dữ liệu sẵn có
10. **module_uart_comm_deinit()** - Đóng driver

### Missing/Incomplete code
- Không có recv_bytes function (chỉ có receive với timeout) (duyệt)
- Thiếu clear TX buffer functionality (loại)
- Không support UART event callbacks (custom event handling) (duyệt)

---

## 10. BSP/Module_UART_Communication/src/module_uart_comm.c

### Tóm tắt
Triển khai Generic UART Communication Driver - wraps ESP-IDF uart_driver_install, uart_param_config, uart_set_pin với unified handle-based API. Cung cấp mutex cho thread-safety, hỗ trợ send/receive với timeout. Lưu state (stack_id, baudrate, buffer sizes) trong opaque handle struct.

### Các hàm/struct chính
1. **module_uart_comm_s** - Internal handle struct (stack_id, port, baudrate, mutex, rx/tx_buffer_size, initialized flag)
2. **is_valid_handle()** - Check handle validity
3. **module_uart_comm_init()** - Allocate handle, install driver, create mutex
4. **module_uart_comm_send()** - Lock mutex, gọi uart_write_bytes, unlock
5. **module_uart_comm_receive()** - Lock mutex, gọi uart_read_bytes, unlock
6. **module_uart_comm_deinit()** - Delete mutex, uninstall driver, free handle

### Missing/Incomplete code
- **BUG**: `uart_driver_delete(config->uart_port)` nhưng config không có field uart_port (compile error) (duyệt)
- Không validate baudrate (có thể set invalid value) (loại)
- Thiếu flow control configuration (hardcoded disabled) (duyệt)

---

## 11. BSP/Module_SPI_Communication/include/module_spi_comm.h

### Tóm tắt
Header cho Generic SPI Communication Driver - unified interface cho SPI master communication với modules. Supports SPI1/SPI2/SPI3 với pin/mode configurability. Hardcodes pins cho Stack 0 (SPI2_HOST) và Stack 1 (SPI3_HOST). Supports full-duplex transfers.

### Các hàm/struct chính
1. **module_spi_comm_handle_t** - Opaque handle pointer
2. **module_spi_config_t** - Struct cấu hình (stack_id, clock_speed, mode 0-3, queue_size)
3. **STACK0_SPI_HOST/MOSI/MISO/SCLK/CS_PIN** - SPI2, MOSI=13, MISO=12, SCLK=14, CS=15
4. **STACK1_SPI_HOST/MOSI/MISO/SCLK/CS_PIN** - SPI3, MOSI=23, MISO=19, SCLK=18, CS=5
5. **module_spi_comm_init()** - Khởi tạo SPI bus + device
6. **module_spi_comm_transfer()** - Full-duplex transfer (tx_data/rx_data can be NULL)
7. **module_spi_comm_deinit()** - Đóng SPI bus

### Missing/Incomplete code
- Không có separate send/receive functions (chỉ full-duplex transfer) (loại) 
- Thiếu support cho half-duplex mode (loại)
- Không allow dynamic pin reconfiguration (loại)

---

## 12. BSP/Module_SPI_Communication/src/module_spi_comm.c

### Tóm tắt
Triển khai Generic SPI Communication Driver - wraps ESP-IDF spi_bus_initialize, spi_bus_add_device, spi_device_transmit. Manages SPI bus + device handle. Supports full-duplex transfers với configurable clock speed và SPI mode.

### Các hàm/struct chính
1. **module_spi_comm_s** - Internal struct (stack_id, host, device handle, clock_speed, mode, initialized)
2. **is_valid_handle()** - Handle validation
3. **module_spi_comm_init()** - Initialize SPI bus (with duplex mode support), add device, store config
4. **module_spi_comm_transfer()** - Prepare spi_transaction_t, gọi spi_device_transmit
5. **module_spi_comm_deinit()** - Remove device, free bus

### Missing/Incomplete code
- Không handle SPI bus already initialized case (returns error) (loại)
- Thiếu DMA configuration (loại, kg dùng dma)
- Không support transactions queue (chỉ gọi spi_device_transmit, không queue multiple) (duyệt)

---

## 13. BSP/Module_I2C_Communication/include/module_i2c_comm.h

### Tóm tắt
Header cho Generic I2C Communication Driver - unified interface cho I2C master communication với modules. Hỗ trợ I2C_NUM_0 (Stack 0) và I2C_NUM_1 (Stack 1) với hardcoded pins. Cung cấp write/read/write_read operations với timeout support.

### Các hàm/struct chính
1. **module_i2c_comm_handle_t** - Opaque handle pointer
2. **module_i2c_config_t** - Struct cấu hình (stack_id, device_address, clock_speed, pullup_enable)
3. **STACK0_I2C_PORT/SDA/SCL_PIN** - I2C0, SDA=21, SCL=22
4. **STACK1_I2C_PORT/SDA/SCL_PIN** - I2C1, SDA=26, SCL=27
5. **module_i2c_comm_init()** - Khởi tạo I2C port
6. **module_i2c_comm_write()** - Ghi dữ liệu đến device
7. **module_i2c_comm_read()** - Đọc dữ liệu từ device
8. **module_i2c_comm_write_read()** - Write register address, rồi read dữ liệu (repeated start)
9. **module_i2c_comm_deinit()** - Đóng I2C driver

### Missing/Incomplete code
- Không support 10-bit I2C addressing (chỉ 7-bit) (duyệt)
- Thiếu multi-device support trên cùng I2C bus (duyệt)
- Không có burst write function (viết nhiều bytes liên tiếp) (loại)

---

## 14. BSP/Module_I2C_Communication/src/module_i2c_comm.c

### Tóm tắt
Triển khai Generic I2C Communication Driver - wraps ESP-IDF i2c_param_config, i2c_driver_install, i2c_cmd_link_create/i2c_master_*. Cung cấp write/read/write_read với proper I2C protocol (START, address, data, ACK/NACK, STOP). Hỗ trợ timeout qua pdMS_TO_TICKS.

### Các hàm/struct chính
1. **module_i2c_comm_s** - Internal struct (stack_id, port, device_address, clock_speed, initialized)
2. **module_i2c_comm_write()** - Create cmd link, START, address+WRITE, data, STOP
3. **module_i2c_comm_read()** - START, address+READ, data (ACK/NACK last byte), STOP
4. **module_i2c_comm_write_read()** - START, address+WRITE, reg_addr, REPEATED_START, address+READ, data, STOP
5. **i2c_cmd_link_create/delete** - Manage I2C command chains
6. **i2c_master_cmd_begin()** - Execute I2C transaction

### Missing/Incomplete code
- Không handle I2C driver already installed case (returns error) (loại)
- Thiếu clock stretching timeout (loại)
- Không support 16-bit register addressing (loại)

---

## 15. BSP/Module_USB_Communication/include/module_usb_comm.h

### Tóm tắt
Header cho Generic USB CDC Communication Driver - USB Serial/JTAG interface cho modules. Sử dụng ESP32 built-in USB peripheral (không cần pin configuration). Hỗ trợ USB CDC line coding parameters (bit rate, parity, stop bits). API tương tự UART (send/receive dengan timeout).

### Các hàm/struct chính
1. **module_usb_comm_handle_t** - Opaque handle pointer
2. **usb_cdc_line_coding_t** - Struct CDC line coding (bit_rate, stop_bits, parity, data_bits)
3. **module_usb_config_t** - Struct cấu hình (stack_id, line_coding, rx/tx_buffer_size)
4. **STACK0_USB_VID/PID** - 0x303A/0x1001
5. **STACK1_USB_VID/PID** - 0x303A/0x1002
6. **module_usb_comm_init()** - Khởi tạo USB Serial/JTAG driver
7. **module_usb_comm_send()** - Gửi dữ liệu (NOT_YET_IMPLEMENTED)
8. **module_usb_comm_receive()** - Nhận dữ liệu (NOT_YET_IMPLEMENTED)
9. **module_usb_comm_flush()** - Flush RX buffer
10. **module_usb_comm_deinit()** - Đóng driver

### Missing/Incomplete code
- **PLACEHOLDER**: send/receive functions return ESP_ERR_NOT_SUPPORTED (chưa implement) (duyệt)
- Không support VID/PID configuration (duyệt)
- Không có multiple interface support (trong trường hợp có nhiều USB devices) (loại)

---

## 16. BSP/Module_USB_Communication/src/module_usb_comm.c

### Tóm tắt
Triển khai Generic USB CDC Communication Driver - wraps ESP-IDF usb_serial_jtag_driver_install. Cung cấp basic init/flush/deinit. Send/receive functions chưa implement (placeholder). Mutex cho thread-safety.

### Các hàm/struct chính
1. **module_usb_comm_s** - Internal struct (stack_id, line_coding, buffer sizes, mutex, initialized)
2. **module_usb_comm_init()** - Create handle, install USB driver, create mutex
3. **module_usb_comm_send()** - Lock mutex, gọi usb_serial_jtag_write_bytes (IMPLEMENTED)
4. **module_usb_comm_receive()** - Lock mutex, gọi usb_serial_jtag_read_bytes (IMPLEMENTED)
5. **module_usb_comm_flush()** - No-op cho USB Serial/JTAG
6. **module_usb_comm_deinit()** - Delete mutex, uninstall driver

### Missing/Incomplete code
- Không validate line_coding values (baudrate, parity, stop bits) (duyệt)
- Thiếu USB event callbacks (connection/disconnection events) (duyệt)
- Không support serial number configuration (loại)

---

## 17. BSP/stack_handler/include/stack_handler.h

### Tóm tắt
Header cho Communication Stack Manager - quản lý 2 communication stacks với dedicated GPIO ports từ TCA6424A expander. Mỗi stack có access đến 9 GPIO pins với mapping cụ thể. Cung cấp API để read/write GPIO, set direction, batch operations. Hỗ trợ lock/unlock stacks để exclusive access.

### Các hàm/struct chính
1. **stack_port_t** - Enum port (STACK_PORT_1=TCA_PORT_0, STACK_PORT_2=TCA_PORT_1)
2. **stack_gpio_pin_num_t** - Enum 9 pins (PIN_1 đến PIN_9)
3. **stack_comm_type_t** - Enum comm types (LORA, RS485, ZIGBEE, CAN)
4. **stack_config_t** - Struct cấu hình (comm_type, gpio_port, uart_port, tx/rx_pin, enabled)
5. **gpio_action_t** - Struct batch GPIO action (pin, level)
6. **stack_handler_init()** - Khởi tạo handler + TCA ports + mutexes
7. **stack_handler_gpio_write()** - Ghi 1 pin
8. **stack_handler_gpio_read()** - Đọc 1 pin
9. **stack_handler_gpio_write_multi()** - Ghi nhiều pins (batched by port)
10. **stack_handler_lock/unlock()** - Mutex lock/unlock

### Missing/Incomplete code
- **PLACEHOLDER/TODO**: `stack_handler_get_stack_id()` - commented out, chưa implement ( để TODO)
- Không validate stack communication type sau configuration (loại)
- Thiếu API để query current GPIO state toàn bộ (loại)

---

## 18. BSP/stack_handler/src/stack_handler.c

### Tóm tắt
Triển khai Communication Stack Manager - quản lý 2 stacks với GPIO mapping từ TCA6424A (9 pins mỗi stack, phân bố trên 3 TCA ports). Hỗ trợ read/write/direction operations qua I2C. Batch GPIO writes optimize I2C transactions bằng cách group actions by TCA port. Thread-safe với mutexes.

### Các hàm/struct chính
1. **stack1_gpio_map[9]** - Mapping GPIO → {TCA_PORT, pin} cho stack 1
2. **stack2_gpio_map[9]** - Mapping GPIO → {TCA_PORT, pin} cho stack 2
3. **g_stack_configs[2]** - Global cấu hình cho 2 stacks
4. **g_stack_mutex[2]** - Mutexes cho 2 stacks
5. **get_tca_mapping()** - Lookup TCA port/pin từ stack_id + gpio_pin
6. **stack_handler_gpio_write_multi()** - Build port_masks, batch write by port
7. **stack_handler_gpio_set_direction()** - Read config register, modify bits, write back
8. **stack_handler_lock/unlock()** - Mutex operations

### Missing/Incomplete code
- **BUG**: GPIO mapping comments bị sai (comment says "Stack 1: P02-P07, P10-P12" nhưng code có P06-P07, P10-P12, v.v.) (duyệt)
- Không validate comm_type value trong set_config (loại)
- Thiếu API để get current comm_type (loại)
- Không log GPIO state changes chi tiết (chỉ log BEFORE/AFTER cho set_direction) (loại)

---

## Summary - Key Issues & Todos

### Critical Bugs
1. **module_uart_comm.c:88** - `uart_driver_delete(config->uart_port)` sai, config không có field uart_port
2. **ble_handler_task.c** - Hàm cleanup idle devices có logic timeout sai
3. **stack_handler.c** - GPIO mapping comments bị nhầm lẫn

### Missing Implementations (Placeholders)
1. **module_usb_comm.h/c** - `module_usb_comm_send/receive()` returns ESP_ERR_NOT_SUPPORTED
2. **stack_handler.h** - `stack_handler_get_stack_id()` commented out (TODO)

### Design Issues
1. Thiếu automatic connection recovery cho BLE
2. Không có API để disconnect device cụ thể từ task layer
3. Hardcoded baudrates/pins không flexible cho custom hardware
4. Chưa validate command strings (buffer overflow risk)
5. Không có retry mechanism cho timeout commands

### Recommendations
1. Implement missing USB send/receive functions
2. Fix module_uart_comm.c malloc deinit bug
3. Complete stack_handler_get_stack_id() cho runtime detection
4. Thêm command string validation
5. Thêm connection recovery mechanism

