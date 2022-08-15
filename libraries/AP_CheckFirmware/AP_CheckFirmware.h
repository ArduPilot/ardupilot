/*
  support checking board ID and firmware CRC in the bootloader
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_OpenDroneID/AP_OpenDroneID_config.h>
#include <AP_HAL/AP_HAL.h>

#ifndef AP_CHECK_FIRMWARE_ENABLED
#define AP_CHECK_FIRMWARE_ENABLED AP_OPENDRONEID_ENABLED
#endif

#if AP_CHECK_FIRMWARE_ENABLED

enum class check_fw_result_t : uint8_t {
    CHECK_FW_OK = 0,
    FAIL_REASON_NO_APP_SIG = 10,
    FAIL_REASON_BAD_LENGTH_APP = 11,
    FAIL_REASON_BAD_BOARD_ID = 12,
    FAIL_REASON_BAD_CRC = 13,
    FAIL_REASON_IN_UPDATE = 14,
    FAIL_REASON_WATCHDOG = 15,
    FAIL_REASON_BAD_LENGTH_DESCRIPTOR = 16,
    FAIL_REASON_BAD_FIRMWARE_SIGNATURE = 17,
    FAIL_REASON_VERIFICATION = 18,
};

#ifndef FW_MAJOR
#define APP_FW_MAJOR 0
#define APP_FW_MINOR 0
#else
#define APP_FW_MAJOR FW_MAJOR
#define APP_FW_MINOR FW_MINOR
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(APJ_BOARD_ID)
// this allows for sitl_periph_gps to build
#define APJ_BOARD_ID 0
#endif

/*
 the app_descriptor stored in flash in the main firmware and is used
 by the bootloader to confirm that the firmware is not corrupt and is
 suitable for this board. The build dependent values in this structure
 are filled in by set_app_descriptor() in the waf build
 */
struct app_descriptor {
#if AP_SIGNED_FIRMWARE
    uint8_t sig[8] = { 0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07 };
#else
    uint8_t sig[8] = { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 };
#endif
    // crc1 is the crc32 from firmware start to start of image_crc1
    uint32_t image_crc1 = 0;
    // crc2 is the crc32 from the start of version_major to the end of the firmware
    uint32_t image_crc2 = 0;
    // total size of firmware image in bytes
    uint32_t image_size = 0;
    uint32_t git_hash = 0;

#if AP_SIGNED_FIRMWARE
    // firmware signature
    uint32_t signature_length = 0;
    uint8_t signature[72] = {};
#endif
    // software version number
    uint8_t  version_major = APP_FW_MAJOR;
    uint8_t version_minor = APP_FW_MINOR;
    // APJ_BOARD_ID (hardware version). This is also used in CAN NodeInfo
    // with high byte in HardwareVersion.major and low byte in HardwareVersion.minor
    uint16_t  board_id = APJ_BOARD_ID;
    uint8_t reserved[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

};

#if AP_SIGNED_FIRMWARE
#define APP_DESCRIPTOR_TOTAL_LENGTH (36+72+4)
#else
#define APP_DESCRIPTOR_TOTAL_LENGTH 36
#endif

static_assert(sizeof(app_descriptor) == APP_DESCRIPTOR_TOTAL_LENGTH, "app_descriptor incorrect length");

#ifdef HAL_BOOTLOADER_BUILD
check_fw_result_t check_good_firmware(void);
#endif
void check_firmware_print(void);

#endif // AP_CHECK_FIRMWARE_ENABLED
