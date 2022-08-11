/*
  support checking board ID and firmware CRC in the bootloader
 */
#include "AP_CheckFirmware.h"
#include <AP_HAL/HAL.h>
#include <AP_Math/crc.h>

#if AP_CHECK_FIRMWARE_ENABLED

#if defined(HAL_BOOTLOADER_BUILD)
/*
  check firmware CRC and board ID to see if it matches
 */
check_fw_result_t check_good_firmware(void)
{
    const uint8_t sig[8] = { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 };
    const uint8_t *flash = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor *ad = (const app_descriptor *)memmem(flash, flash_size-sizeof(app_descriptor), sig, sizeof(sig));
    if (ad == nullptr) {
        // no application signature
        return check_fw_result_t::FAIL_REASON_NO_APP_SIG;
    }
    // check length
    if (ad->image_size > flash_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_APP;
    }

    bool id_ok = (ad->board_id == APJ_BOARD_ID);
#ifdef ALT_BOARD_ID
    id_ok |= (ad->board_id == ALT_BOARD_ID);
#endif

    if (!id_ok) {
        return check_fw_result_t::FAIL_REASON_BAD_BOARD_ID;
    }

    const uint8_t desc_len = offsetof(app_descriptor, version_major) - offsetof(app_descriptor, image_crc1);
    uint32_t len1 = ((const uint8_t *)&ad->image_crc1) - flash;
    if ((len1 + desc_len) > ad->image_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_DESCRIPTOR;
    }

    uint32_t len2 = ad->image_size - (len1 + desc_len);
    uint32_t crc1 = crc32_small(0, flash, len1);
    uint32_t crc2 = crc32_small(0, (const uint8_t *)&ad->version_major, len2);
    if (crc1 != ad->image_crc1 || crc2 != ad->image_crc2) {
        return check_fw_result_t::FAIL_REASON_BAD_CRC;
    }
    return check_fw_result_t::CHECK_FW_OK;
}
#endif // HAL_BOOTLOADER_BUILD

#if !defined(HAL_BOOTLOADER_BUILD)
extern const AP_HAL::HAL &hal;

/*
  declare constant app_descriptor in flash
 */
extern const struct app_descriptor app_descriptor;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
const struct app_descriptor app_descriptor __attribute__((section(".app_descriptor")));
#else
const struct app_descriptor app_descriptor;
#endif

/*
  this is needed to ensure we don't elide the app_descriptor
 */
void check_firmware_print(void)
{
    hal.console->printf("Booting %u/%u\n",
                        app_descriptor.version_major,
                        app_descriptor.version_minor);
}
#endif

#endif // AP_CHECK_FIRMWARE_ENABLED
