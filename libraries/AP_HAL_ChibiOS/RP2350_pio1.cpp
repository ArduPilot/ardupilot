/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RP2350_pio1.h"

/*
  The broker arbitrates between three things a bootloader has none of - the OSD
  scan-out, the LED driver and the PIO UARTs - and it asks AP_Param which of
  them the user wants, so it drags AP_Param and GCS in with it. Nothing in a
  bootloader build calls it: its hwdef is standalone and defines neither
  HAL_HAVE_PIO_UARTS nor the OSD.
 */
#if defined(RP2350) && !defined(HAL_BOOTLOADER_BUILD)

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

namespace ChibiOS {

/*
  OSD_TYPE selecting the RP2350 PIO scan-out. AP_OSD's enum runs 0-5 and gains
  this value with the backend; until then the number is only meaningful here,
  and an OSD_TYPE AP_OSD does not recognise simply leaves it without a
  backend, which is harmless.
 */
#define OSD_TYPE_PICO 6.0f

static PIO1Owner intended = PIO1Owner::NONE;
static PIO1Owner current = PIO1Owner::NONE;
static bool resolved;

PIO1Owner pio1_intended_owner(void)
{
    if (resolved) {
        return intended;
    }

    /*
      A board that wires UARTs to PIO1 settles this at build time - PIOUART2
      and PIOUART3 are fixed to that block - so neither the overlay nor the
      LED driver can have it whatever OSD_TYPE says, and there is nothing to
      wait for parameters for.
     */
#if defined(HAL_HAVE_PIO_UARTS) && HAL_HAVE_PIO_UARTS > 2
    intended = PIO1Owner::PIOUART;
    resolved = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PIO1: PIO UARTs");
    return intended;
#endif

    /*
      Reading OSD_TYPE before load_parameters() gives the compiled-in default
      rather than what the user set, so wait for AP_Param.

      Not is_system_initialized(): that only goes true at the very end of
      AP_Vehicle::setup(), long after init_ardupilot() has set up the output
      groups and asked. NeoPixel therefore always got NONE, RCOutput took the
      refusal as permanent and disabled the group, and the LED output failed
      whatever OSD_TYPE said. load_parameters() is AP_Vehicle.cpp:342 and
      init_ardupilot() is :449, so by the time anyone asks, this is true.
     */
    if (!AP_Param::initialised()) {
        return PIO1Owner::NONE;
    }

    float osd_type = 0;
    const bool got = AP_Param::get("OSD_TYPE", osd_type);
    /*
      A name that does not exist returns false, which would read here as "no
      OSD" and silently hand the block to the LED driver. OSD_TYPE is defined
      by AP_OSD's group as "TYPE", so it is absent only when AP_OSD is
      compiled out - in which case the LED driver is the right answer anyway.
     */
    if (got && is_equal(osd_type, OSD_TYPE_PICO)) {
        intended = PIO1Owner::OSD;
    } else {
        intended = PIO1Owner::NEOPIXEL;
    }
    resolved = true;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PIO1: %s",
                  intended == PIO1Owner::OSD ? "OSD" : "NeoPixel");
    return intended;
}

bool pio1_claim(PIO1Owner who)
{
    if (who == PIO1Owner::NONE) {
        return false;
    }
    if (current == who) {
        return true;
    }
    if (current != PIO1Owner::NONE) {
        // already taken, and the block cannot be shared
        return false;
    }
    if (pio1_intended_owner() != who) {
        return false;
    }
    current = who;
    return true;
}

PIO1Owner pio1_current_owner(void)
{
    return current;
}

}  // namespace ChibiOS

#endif  // RP2350 && !HAL_BOOTLOADER_BUILD
