#include "AP_AirframeID.h"

#if AP_AIRFRAME_ID_ENABLED && !defined(HAL_BOOTLOADER_BUILD)

#include <AP_HAL/AP_HAL.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

bool AP_AirframeID::find(const uint8_t *image, uint32_t image_len, uint16_t &airframe_id)
{
    const uint8_t sig[] = AP_AIRFRAME_ID_SIGNATURE;
    const uint8_t *p = (const uint8_t *)memmem(image, image_len, sig, sizeof(sig));
    if (p == nullptr || p + sizeof(ap_airframe_data) > image + image_len) {
        return false;
    }
    airframe_id = ((const ap_airframe_data *)p)->airframe_id;
    return true;
}

/*
  the bootloader links its airframe data near the start of its image,
  so like the public keys it is in the first flash sector. Only the
  part of that sector below the firmware is searched, as on a board
  without a bootloader the sector holds this firmware, which contains
  the signature as a search key
 */
bool AP_AirframeID::installed(uint16_t &airframe_id)
{
    const uint32_t flash_addr = hal.flash->getpageaddr(0);
    uint32_t len = hal.flash->getpagesize(0);
    if (FLASH_LOAD_ADDRESS <= flash_addr) {
        return false;
    }
    if (FLASH_LOAD_ADDRESS - flash_addr < len) {
        len = FLASH_LOAD_ADDRESS - flash_addr;
    }
    return find((const uint8_t *)flash_addr, len, airframe_id);
}

#endif // AP_AIRFRAME_ID_ENABLED && !defined(HAL_BOOTLOADER_BUILD)
