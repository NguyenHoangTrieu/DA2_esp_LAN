#!/bin/bash

# 1. Export ESP-IDF environment variables (ensure correct version and toolchain)
source ~/esp-idf/export.sh

# 2. (Optional) remove old build directory for clean build
# idf.py fullclean

# 3. Build project (includes configure, CMake, Ninja, dependencies...)
idf.py build

#4. copy firmware to flash directory
cp -r build/DA2_esp_LAN.bin ../DA2_esp/config_app/dist/bin
cp -r build/bootloader/bootloader.bin ../DA2_esp/config_app/dist/bin/bootloader_LAN.bin
cp -r build/partition_table/partition-table.bin ../DA2_esp/config_app/dist/bin/partition-table_LAN.bin

# 4. (Optional) flash firmware to device
# idf.py -p /dev/ttyUSB0 flash
