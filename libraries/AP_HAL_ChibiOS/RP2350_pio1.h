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
/*
 * Ownership of PIO1 on RP2350.
 *
 * PIO2 belongs to DShot with all four state machines in use, so PIO1 is the
 * block with anything free - and the only one left at GPIOBASE 0, which pins
 * below GPIO32 require. Three drivers want it and none can share it: the
 * analog OSD scan-out is 31 of the block's 32 instruction slots, WS2812 needs
 * four, and PIOUART2 and PIOUART3 are fixed to PIO1 in PIOUART's config
 * table, so a board with more than two PIO UARTs puts them here too.
 *
 * Every claim is a blind write to INSTR_MEM, and neopixel_init() is lazy, so
 * without arbitration the later caller silently overwrites the earlier one.
 * That is not hypothetical: it overwrote a running video output during OSD
 * bring-up and was only visible by reading CLKDIV back over SWD.
 *
 * The owner is decided by the board and by parameter, not by who asks first,
 * so the outcome does not depend on driver init order. Asking before
 * parameters are loaded returns NONE and the caller is expected to try again.
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#if defined(RP2350)

namespace ChibiOS {

enum class PIO1Owner : uint8_t {
    NONE = 0,
    OSD,
    NEOPIXEL,
    PIOUART,
};

/*
  Resolved once and cached. PIO UARTs on this block are a build time fact and
  win outright; otherwise it comes from OSD_TYPE, and is NONE until parameters
  are up.
 */
PIO1Owner pio1_intended_owner(void);

// True only for the intended owner, and only once. A caller that loses is
// expected to do nothing rather than proceed - a partial claim of this block
// is worse than none, because the hardware gives no way to detect it.
bool pio1_claim(PIO1Owner who);

PIO1Owner pio1_current_owner(void);

}  // namespace ChibiOS

#endif  // RP2350
