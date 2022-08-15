/*
  support checking board ID and firmware CRC in the bootloader
 */
#include "AP_CheckFirmware.h"
#include <AP_HAL/HAL.h>
#include <AP_Math/crc.h>

#if AP_CHECK_FIRMWARE_ENABLED

#if defined(HAL_BOOTLOADER_BUILD)

#if AP_SIGNED_FIRMWARE
#include "../../Tools/AP_Bootloader/support.h"
#include <string.h>
#include "monocypher.h"

struct PACKED secure_data {
    uint8_t sig[8] = {0x4e, 0xcf, 0x4e, 0xa5, 0xa6, 0xb6, 0xf7, 0x29};
    struct PACKED {
        uint8_t key[32] = {};
    } public_key[10];
};

const struct secure_data public_keys __attribute__((section(".ecc_raw")));

/*
  check a signature against bootloader keys
 */
static check_fw_result_t check_firmware_signature(const app_descriptor *ad,
                                                  const uint8_t *flash1, uint32_t len1,
                                                  const uint8_t *flash2, uint32_t len2)
{
    // 8 byte signature version
    static const uint64_t sig_version = 30437LLU;

    if (ad->signature_length != 72) {
        return check_fw_result_t::FAIL_REASON_BAD_FIRMWARE_SIGNATURE;
    }
    if (memcmp((const uint8_t*)&sig_version, ad->signature, sizeof(sig_version)) != 0) {
        return check_fw_result_t::FAIL_REASON_BAD_FIRMWARE_SIGNATURE;
    }

    /*
      look over all public keys, if one matches then we are OK
     */
    for (const auto &public_key : public_keys.public_key) {
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract*)&ctx;
        crypto_check_init(actx, &ad->signature[sizeof(sig_version)], public_key.key);

        crypto_check_update(actx, flash1, len1);
        crypto_check_update(actx, flash2, len2);
        if (crypto_check_final(actx) == 0) {
            // good signature
            return check_fw_result_t::CHECK_FW_OK;
        }
    }
    // none of the public keys matched
    return check_fw_result_t::FAIL_REASON_VERIFICATION;
}
#endif // AP_SIGNED_FIRMWARE

/*
  check firmware CRC and board ID to see if it matches
 */
check_fw_result_t check_good_firmware(void)
{
#if AP_SIGNED_FIRMWARE
    const uint8_t sig[8] = { 0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07 };
#else
    const uint8_t sig[8] = { 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 };
#endif
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor *ad = (const app_descriptor *)memmem(flash1, flash_size-sizeof(app_descriptor), sig, sizeof(sig));
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

    const uint8_t *flash2 = (const uint8_t *)&ad->version_major;
    const uint8_t desc_len = offsetof(app_descriptor, version_major) - offsetof(app_descriptor, image_crc1);
    const uint32_t len1 = ((const uint8_t *)&ad->image_crc1) - flash1;

    if ((len1 + desc_len) > ad->image_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_DESCRIPTOR;
    }

    const uint32_t len2 = ad->image_size - (len1 + desc_len);
    uint32_t crc1 = crc32_small(0, flash1, len1);
    uint32_t crc2 = crc32_small(0, flash2, len2);
    if (crc1 != ad->image_crc1 || crc2 != ad->image_crc2) {
        return check_fw_result_t::FAIL_REASON_BAD_CRC;
    }

    check_fw_result_t ret = check_fw_result_t::CHECK_FW_OK;

#if AP_SIGNED_FIRMWARE
    ret = check_firmware_signature(ad, flash1, len1, flash2, len2);
#endif

    return ret;
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
