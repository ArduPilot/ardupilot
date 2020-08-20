/*
  application -> bootloader communication structure This is put into
  the start of RAM by AP_Periph to facilitate firmware upload with
  UAVCAN
 */

#pragma once

#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a

struct app_bootloader_comms {
    uint32_t magic;
    uint32_t reserved[4];
    uint8_t server_node_id;
    uint8_t my_node_id;
    uint8_t path[201];
};

#ifndef FW_MAJOR
#define FW_MAJOR 0
#define FW_MINOR 0
#endif

/*
 the app_descriptor stored in flash in the main firmware and is used
 by the bootloader to confirm that the firmware is not corrupt and is
 suitable for this board. The build dependent values in this structure
 are filled in by set_app_descriptor() in the waf build
 */
struct app_descriptor {
    uint8_t sig[8] = { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 };
    // crc1 is the crc32 from firmware start to start of image_crc1
    uint32_t image_crc1 = 0;
    // crc2 is the crc32 from the start of version_major to the end of the firmware
    uint32_t image_crc2 = 0;
    // total size of firmware image in bytes
    uint32_t image_size = 0;
    uint32_t git_hash = 0;
    // software version number
    uint8_t  version_major = FW_MAJOR;
    uint8_t version_minor = FW_MINOR;
    // APJ_BOARD_ID (hardware version). This is also used in CAN NodeInfo
    // with high byte in HardwareVersion.major and low byte in HardwareVersion.minor
    uint16_t  board_id = APJ_BOARD_ID;
    uint8_t reserved[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
};

#define APP_DESCRIPTOR_TOTAL_LENGTH 36
static_assert(sizeof(app_descriptor) == APP_DESCRIPTOR_TOTAL_LENGTH, "app_descriptor incorrect length");
