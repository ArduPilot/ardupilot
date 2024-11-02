#!/bin/bash
source ./modules/esp_idf/export.sh
xtensa-esp32s3-elf-gdb -x ./Tools/debug/gdbinit.esp32s3 build/esp32s3devkit/esp-idf_build/ardupilot.elf

# gdb tips:
#'... normal breakpoints only work with functions in IRAM. For functions in flash, please use hardware breakpoints (use 'hb' instead of 'b').''

# hb HAL_ESP32_Class.cpp:108

# mon reset halt

