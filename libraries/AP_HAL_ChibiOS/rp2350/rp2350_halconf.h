/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   RP2350 HAL driver selection, included by the common halconf.h ahead of
   its defaults.
 */
/*
  The HAL drivers every RP2350 board's flight firmware uses, so a board's
  hwdef.dat does not repeat them; a board can still set any of them. The
  bootloader's hwdef-bl.dat chooses its own.
 */
#pragma once

#if !defined(HAL_BOOTLOADER_BUILD)

// parameter storage in the boot flash
#ifndef HAL_USE_EFL
#define HAL_USE_EFL                         TRUE
#endif

// the hardware UARTs are SIO drivers; there is no serial driver to fall back on
#ifndef HAL_USE_SIO
#define HAL_USE_SIO                         TRUE
#endif
#ifndef HAL_USE_SERIAL
#define HAL_USE_SERIAL                      FALSE
#endif

#ifndef HAL_USE_WDG
#define HAL_USE_WDG                         TRUE
#endif

#ifndef PAL_USE_WAIT
#define PAL_USE_WAIT                        TRUE
#endif

#endif  // !defined(HAL_BOOTLOADER_BUILD)
