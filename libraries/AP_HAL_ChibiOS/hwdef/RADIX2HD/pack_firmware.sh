#!/usr/bin/env bash

# Example script to pack firmware for RADIX 2 HD
# Get the firmware packer tool from here: https://github.com/BrainFPV/brainfpv_fw_packer

# Get version information
GIT_VER=$(git describe HEAD 2>&1)
GIT_SHA1=$(git rev-parse HEAD)

TARGET_BUILD_DIR=./build/RADIX2HD/bin
FW_ELF=${TARGET_BUILD_DIR}/ardu$1
FW_HEX=${TARGET_BUILD_DIR}/ardu$1.hex
FW_PACKED=${TARGET_BUILD_DIR}/ardu$1_${GIT_VER}_brainfpv.bin

# Create hex file
arm-none-eabi-objcopy -O ihex ${FW_ELF} ${FW_HEX}

# Create binary for BrainFPV bootloader
brainfpv_fw_packer.py \
    --name ardu$1 \
    --version ${GIT_VER} \
    --sha1 ${GIT_SHA1} \
    --in ${FW_HEX} --out ${FW_PACKED} \
    --dev radix2hd -t firmware -b 0x90400000 -z --noheader
