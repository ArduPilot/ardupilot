#ifndef FORCE_VERSION_H_INCLUDE
#error AP_CheckFirmwareDefines.h should never be included directly. You probably want to include AP_CheckFirmware/AP_CheckFirmware.h
#endif
#include "AP_CheckFirmware.h"

#if AP_CHECK_FIRMWARE_ENABLED
/*
  declare constant app_descriptor in flash
 */
extern const app_descriptor_t app_descriptor;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
const app_descriptor_t app_descriptor __attribute__((section(".app_descriptor"))) = {
#else
const app_descriptor_t app_descriptor = {
#endif
#if AP_SIGNED_FIRMWARE
    .sig = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED,
#else
    .sig = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED,
#endif
    .image_crc1 = 0,
    .image_crc2 = 0,
    .image_size = 0,
    .git_hash = 0,
#if AP_SIGNED_FIRMWARE
    .signature_length = 0,
    .signature = {},
#endif
    .version_major = APP_FW_MAJOR,
    .version_minor = APP_FW_MINOR,
    .board_id = APJ_BOARD_ID,
    .reserved = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};
#endif // AP_CHECK_FIRMWARE_ENABLED
