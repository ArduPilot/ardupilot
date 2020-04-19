/*
  application -> bootloader communication structure This is put into
  the start of RAM by AP_Periph to facilitate firmware upload with
  UAVCAN
 */

#pragma once

#include "AP_HAL_ChibiOS.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>


#define BOOTLOADER_APP_COMMS_MAGIC 0xa9da445c

struct bootloader_app_comms {
    uint32_t magic;
    uint32_t git_hash;
    uint32_t chibios_git_hash;
};

#ifndef FW_MAJOR
#define FW_MAJOR 0
#define FW_MINOR 0
#endif

#ifndef APM_BUILD_DIRECTORY
  #define APM_BUILD_DIRECTORY APM_BUILD_UNKNOWN
#endif

#ifndef TARGET_FEATURE_MASK
  #define TARGET_FEATURE_MASK 0x0
#endif

#define APP_DESCRIPTOR_VERSION 0x01

/*
 the app_descriptor stored in flash in the main firmware and is used
 by the bootloader to confirm that the firmware is not corrupt and is
 suitable for this board. The build dependent values in this structure
 are filled in by set_app_descriptor() in the waf build
 */
#define HAL_APP_DESCRIPTOR_SIG { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 }
#define HAL_OLD_APP_DESCRIPTOR_SIG { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0xFF }
struct app_descriptor {
    uint8_t sig[8] = HAL_APP_DESCRIPTOR_SIG;
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

    uint8_t app_descriptor_version = APP_DESCRIPTOR_VERSION;
    uint8_t app_device_type_id = APM_BUILD_DIRECTORY;
    uint8_t app_feature_mask = TARGET_FEATURE_MASK;
    uint8_t reserved = 0xFF;
    uint8_t reserved2[32] = {0xFF};
};

enum app_feature_bits {
    FEATURE_BIT_STORAGE_BACKUP = 0,
};

#define APP_DESCRIPTOR_TOTAL_LENGTH 64
static_assert(sizeof(app_descriptor) == APP_DESCRIPTOR_TOTAL_LENGTH, "app_descriptor incorrect length");
