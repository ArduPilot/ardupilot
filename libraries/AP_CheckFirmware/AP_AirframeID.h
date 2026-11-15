/*
  airframe IDs allow several airframes built around the same autopilot
  to share one APJ_BOARD_ID while still being told apart.

  An airframe is given an ID with "define AP_AIRFRAME_ID n" in both its
  hwdef.dat and hwdef-bl.dat. The bootloader embeds the ID in its
  image, and firmware built with an airframe ID refuses to arm unless
  the installed bootloader carries the same ID. No firmware will
  replace a bootloader which carries an airframe ID with one which
  carries a different ID, or none, so the airframe a board belongs to
  survives firmware loads.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>

#ifndef AP_AIRFRAME_ID_ENABLED
#define AP_AIRFRAME_ID_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS)
#endif

// firmware built for a specific airframe checks the bootloader's airframe ID
#if AP_AIRFRAME_ID_ENABLED && defined(AP_AIRFRAME_ID)
#define AP_AIRFRAME_ID_CHECK_ENABLED 1
#else
#define AP_AIRFRAME_ID_CHECK_ENABLED 0
#endif

#if AP_AIRFRAME_ID_ENABLED

#ifdef AP_AIRFRAME_ID
static_assert(AP_AIRFRAME_ID > 0 && AP_AIRFRAME_ID <= UINT16_MAX, "AP_AIRFRAME_ID must be between 1 and 65535");
#endif

#define AP_AIRFRAME_ID_SIGNATURE { 0xec, 0x80, 0xd8, 0x62, 0x58, 0x4e, 0x1a, 0xff }

// placed in the bootloader image, found by searching for the signature
struct PACKED ap_airframe_data {
    uint8_t sig[8];
    uint16_t airframe_id;
};

#ifndef HAL_BOOTLOADER_BUILD
class AP_AirframeID {
public:
    // find the airframe ID in a bootloader image, false if it has none
    static bool find(const uint8_t *image, uint32_t image_len, uint16_t &airframe_id);

    // find the airframe ID in the installed bootloader, false if it has none
    static bool installed(uint16_t &airframe_id);
};
#endif // HAL_BOOTLOADER_BUILD

#endif // AP_AIRFRAME_ID_ENABLED
