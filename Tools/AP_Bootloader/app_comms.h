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

/*
 the app_descriptor stored in flash in the main firmware and is used
 by the bootloader to confirm that the firmware is not corrupt and is
 suitable for this board. The build dependent values in this structure
 are filled in by set_app_descriptor() in the waf build
 */
struct app_descriptor {
    uint8_t sig[8];
    // crc1 is the crc32 from firmware start to start of image_crc1
    uint32_t image_crc1;
    // crc2 is the crc32 from the start of version_major to the end of the firmware
    uint32_t image_crc2;
    // total size of firmware image in bytes
    uint32_t image_size;
    uint32_t git_hash;
    // software version number
    uint8_t  version_major;
    uint8_t version_minor;
    // APJ_BOARD_ID (hardware version). This is also used in CAN NodeInfo
    // with high byte in HardwareVersion.major and low byte in HardwareVersion.minor
    uint16_t  board_id;
    uint8_t reserved[8];
};

#define APP_DESCRIPTOR_TOTAL_LENGTH 36
static_assert(sizeof(app_descriptor) == APP_DESCRIPTOR_TOTAL_LENGTH, "app_descriptor incorrect length");
