/*
  support checking board ID and firmware CRC in the bootloader
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_OpenDroneID/AP_OpenDroneID_config.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_config.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif

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
// this allows for sitl_periph to build
#define APJ_BOARD_ID 0
#endif

/*
 the app_descriptor stored in flash in the main firmware and is used
 by the bootloader to confirm that the firmware is not corrupt and is
 suitable for this board. The build dependent values in this structure
 are filled in by set_app_descriptor() in the waf build

 Note that we need to define both structures to make it possible to
 boot a signed firmware using a bootloader setup for unsigned
 */

#define AP_APP_DESCRIPTOR_SIGNATURE_SIGNED   { 0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07 }
#define AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 }

struct app_descriptor_unsigned {
    uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
    // crc1 is the crc32 from firmware start to start of image_crc1
    uint32_t image_crc1 = 0;
    // crc2 is the crc32 from the start of version_major to the end of the firmware
    uint32_t image_crc2 = 0;
    // total size of firmware image in bytes
    uint32_t image_size = 0;
    uint32_t git_hash = 0;

    // software version number
    uint8_t  version_major = APP_FW_MAJOR;
    uint8_t version_minor = APP_FW_MINOR;
    // APJ_BOARD_ID (hardware version). This is also used in CAN NodeInfo
    // with high byte in HardwareVersion.major and low byte in HardwareVersion.minor
    uint16_t  board_id = APJ_BOARD_ID;
    uint8_t reserved[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
};

struct app_descriptor_signed {
    uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
    // crc1 is the crc32 from firmware start to start of image_crc1
    uint32_t image_crc1 = 0;
    // crc2 is the crc32 from the start of version_major to the end of the firmware
    uint32_t image_crc2 = 0;
    // total size of firmware image in bytes
    uint32_t image_size = 0;
    uint32_t git_hash = 0;

    // firmware signature
    uint32_t signature_length = 0;
    uint8_t signature[72] = {};

    // software version number
    uint8_t  version_major = APP_FW_MAJOR;
    uint8_t version_minor = APP_FW_MINOR;
    // APJ_BOARD_ID (hardware version). This is also used in CAN NodeInfo
    // with high byte in HardwareVersion.major and low byte in HardwareVersion.minor
    uint16_t  board_id = APJ_BOARD_ID;
    uint8_t reserved[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
};

#if AP_SIGNED_FIRMWARE
typedef struct app_descriptor_signed app_descriptor_t;
#else
typedef struct app_descriptor_unsigned app_descriptor_t;
#endif

#define APP_DESCRIPTOR_UNSIGNED_TOTAL_LENGTH 36
#define APP_DESCRIPTOR_SIGNED_TOTAL_LENGTH (APP_DESCRIPTOR_UNSIGNED_TOTAL_LENGTH+72+4)

static_assert(sizeof(app_descriptor_unsigned) == APP_DESCRIPTOR_UNSIGNED_TOTAL_LENGTH, "app_descriptor_unsigned incorrect length");
static_assert(sizeof(app_descriptor_signed) == APP_DESCRIPTOR_SIGNED_TOTAL_LENGTH, "app_descriptor_signed incorrect length");

#if AP_SIGNED_FIRMWARE

#define AP_PUBLIC_KEY_LEN 32
#define AP_PUBLIC_KEY_MAX_KEYS 10
#define AP_PUBLIC_KEY_SIGNATURE {0x4e, 0xcf, 0x4e, 0xa5, 0xa6, 0xb6, 0xf7, 0x29}

struct PACKED ap_secure_data {
    uint8_t sig[8] = AP_PUBLIC_KEY_SIGNATURE;
    struct PACKED {
        uint8_t key[AP_PUBLIC_KEY_LEN] = {};
    } public_key[AP_PUBLIC_KEY_MAX_KEYS];
};
#endif

#ifdef HAL_BOOTLOADER_BUILD
check_fw_result_t check_good_firmware(void);
#else
void check_firmware_print(void);

class AP_CheckFirmware {
public:
#if HAL_GCS_ENABLED
    // handle a message from the GCS. This is static as we don't have an AP_CheckFirmware object
    static void handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg);
    static void handle_secure_command(mavlink_channel_t chan, const mavlink_secure_command_t &pkt);
    static bool check_signature(const mavlink_secure_command_t &pkt);
#endif
    static const struct ap_secure_data *find_public_keys(void);

    /*
      in memory structure representing the current bootloader. It has two
      data regions to cope with persistent data at the end of the
      bootloader sector
    */
    struct bl_data {
        uint32_t length1;
        uint8_t *data1;
        uint32_t offset2;
        uint32_t length2;
        uint8_t *data2;

        // destructor
        ~bl_data(void) {
            delete[] data1;
            delete[] data2;
        }
    };
    static struct bl_data *read_bootloader(void);
    static bool write_bootloader(const struct bl_data *bld);
    static bool set_public_keys(uint8_t key_idx, uint8_t num_keys, const uint8_t *key_data);
    static bool all_zero_keys(const struct ap_secure_data *sec_data);
    static bool check_signed_bootloader(const uint8_t *fw, uint32_t fw_size);

private:
#if HAL_GCS_ENABLED
    static uint8_t session_key[8];
#endif
};

#endif // HAL_BOOTLOADER_BUILD

#endif // AP_CHECK_FIRMWARE_ENABLED
