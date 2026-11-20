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

#ifndef BOOT_FROM_EXT_FLASH
#define BOOT_FROM_EXT_FLASH 0
#endif

const struct ap_secure_data public_keys __attribute__((section(".apsec_data")));

/*
  return true if all public keys are zero. We allow boot of an
  unsigned firmware in that case
 */
static bool all_zero_public_keys(void)
{
    return ap_public_keys_all_zero(public_keys);
}

/*
  check a signature against bootloader keys
 */
static check_fw_result_t check_firmware_signature(const app_descriptor_signed *ad,
                                                  const uint8_t *flash1, uint32_t len1,
                                                  const uint8_t *flash2, uint32_t len2,
                                                  const uint8_t *extf_signature)
{
    if (all_zero_public_keys()) {
        return check_fw_result_t::CHECK_FW_OK;
    }

    // 8 byte signature version
    static const uint64_t sig_version_legacy = 30437LLU;
    uint64_t sig_version;
    if (ad->signature_length != 72) {
        return check_fw_result_t::FAIL_REASON_BAD_FIRMWARE_SIGNATURE;
    }
    memcpy(&sig_version, ad->signature, sizeof(sig_version));

    if (sig_version != sig_version_legacy) {
        return check_fw_result_t::FAIL_REASON_BAD_FIRMWARE_SIGNATURE;
    }

    /*
      look over all public keys, if one matches then we are OK
     */
    bool signature_ok = false;
    for (const auto &public_key : public_keys.public_key) {
        if (ap_public_key_is_zero(public_key.key)) {
            continue;
        }
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract*)&ctx;
        crypto_check_init(actx, &ad->signature[sizeof(sig_version)], public_key.key);

        crypto_check_update(actx, flash1, len1);
        crypto_check_update(actx, flash2, len2);
        if (crypto_check_final(actx) == 0) {
            signature_ok = true;
            break;
        }
    }
    if (!signature_ok) {
        return check_fw_result_t::FAIL_REASON_VERIFICATION;
    }

#if EXT_FLASH_SIZE_MB && !BOOT_FROM_EXT_FLASH
    static const uint64_t sig_version_extf = 30438LLU;
    uint32_t extf_signature_length;
    uint64_t extf_signature_version;
    memcpy(&extf_signature_length, extf_signature, sizeof(extf_signature_length));
    memcpy(&extf_signature_version, extf_signature+sizeof(extf_signature_length), sizeof(extf_signature_version));
    if (ad->extf_image_size == 0 ||
        ad->extf_image_size > board_info.extf_size ||
        extf_signature_length != 72 ||
        extf_signature_version != sig_version_extf) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_EXTF;
    }

    for (const auto &public_key : public_keys.public_key) {
        if (ap_public_key_is_zero(public_key.key)) {
            continue;
        }
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract*)&ctx;
        ap_extf_signature_check_init(actx,
                                     extf_signature+sizeof(extf_signature_length)+sizeof(extf_signature_version),
                                     public_key.key, len1, len2, ad->extf_image_size);
        crypto_check_update(actx, flash1, len1);
        crypto_check_update(actx, flash2, len2);

        uint8_t extf_buffer[256];
        for (uint32_t offset = 0; offset < ad->extf_image_size; offset += sizeof(extf_buffer)) {
            const uint32_t remaining = ad->extf_image_size - offset;
            const uint32_t read_size = remaining < sizeof(extf_buffer) ? remaining : sizeof(extf_buffer);
            if (!ext_flash_read(offset, extf_buffer, read_size)) {
                return check_fw_result_t::FAIL_REASON_EXTF_READ;
            }
            crypto_check_update(actx, extf_buffer, read_size);
        }
        if (crypto_check_final(actx) == 0) {
            return check_fw_result_t::CHECK_FW_OK;
        }
    }
#else
    return check_fw_result_t::CHECK_FW_OK;
#endif

    // none of the public keys matched
    return check_fw_result_t::FAIL_REASON_VERIFICATION;
}
#endif // AP_SIGNED_FIRMWARE

/*
  check firmware CRC and board ID to see if it matches
 */
static check_fw_result_t check_good_firmware_signed(void)
{
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor_signed *ad = (const app_descriptor_signed *)memmem(flash1, flash_size-sizeof(app_descriptor_signed), sig, sizeof(sig));
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
    const uint8_t desc_len = offsetof(app_descriptor_signed, version_major) - offsetof(app_descriptor_signed, image_crc1);
    const uint32_t len1 = ((const uint8_t *)&ad->image_crc1) - flash1;
    const uint8_t descriptor_tail_len = sizeof(app_descriptor_signed) - offsetof(app_descriptor_signed, version_major);

    if ((len1 + desc_len + descriptor_tail_len) > ad->image_size) {
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
    const uint8_t *extf_signature = nullptr;
#if EXT_FLASH_SIZE_MB && !BOOT_FROM_EXT_FLASH
    static const uint8_t extf_signature_size = 76;
    if (ad->image_size > flash_size-extf_signature_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_EXTF;
    }
    extf_signature = flash1+ad->image_size;
#endif
    ret = check_firmware_signature(ad, flash1, len1, flash2, len2, extf_signature);
#endif

    return ret;
}

/*
  check firmware CRC and board ID to see if it matches, using unsigned
  signature
 */
static check_fw_result_t check_good_firmware_unsigned(void)
{
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor_unsigned *ad = (const app_descriptor_unsigned *)memmem(flash1, flash_size-sizeof(app_descriptor_unsigned), sig, sizeof(sig));
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
    const uint8_t desc_len = offsetof(app_descriptor_unsigned, version_major) - offsetof(app_descriptor_unsigned, image_crc1);
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

    return check_fw_result_t::CHECK_FW_OK;
}

check_fw_result_t check_good_firmware(void)
{
#if AP_SIGNED_FIRMWARE
    // allow unsigned format if we have no public keys. This allows
    // for use of SECURE_COMMAND to remove all public keys and then
    // load of unsigned firmware
    const auto ret = check_good_firmware_signed();
    if (ret != check_fw_result_t::CHECK_FW_OK &&
        all_zero_public_keys() &&
        check_good_firmware_unsigned() == check_fw_result_t::CHECK_FW_OK) {
        return check_fw_result_t::CHECK_FW_OK;
    }
    return ret;
#else
    const auto ret = check_good_firmware_unsigned();
    if (ret != check_fw_result_t::CHECK_FW_OK) {
        // allow for signed format, not checking public keys. This
        // allows for booting of a signed firmware with an unsigned
        // bootloader, which allows for bootstrapping a system up from
        // unsigned to signed
        const auto ret2 = check_good_firmware_signed();
        if (ret2 == check_fw_result_t::CHECK_FW_OK) {
            return check_fw_result_t::CHECK_FW_OK;
        }
    }
    return ret;
#endif
}

const app_descriptor_t *get_app_descriptor(void)
{
#if AP_SIGNED_FIRMWARE
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
#else
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
#endif
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor_t *ad = (const app_descriptor_t *)memmem(flash1, flash_size-sizeof(app_descriptor_t), sig, sizeof(sig));
    return ad;
}

#endif // HAL_BOOTLOADER_BUILD

#if !defined(HAL_BOOTLOADER_BUILD)
extern const AP_HAL::HAL &hal;
extern const app_descriptor_t app_descriptor;

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
