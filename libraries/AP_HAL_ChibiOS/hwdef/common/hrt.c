/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  High Resolution Timer code. This provides support for 32 and 64 bit
  returns for micros and millis functions
*/

#include "ch.h"
#include "hal.h"
#include "hrt.h"
#include <stdint.h>

#pragma GCC optimize("O2")

/*
  we have 4 possible configurations of boards, made up of boards that
  have the following properties:

    CH_CFG_ST_RESOLUTION = 16 or 32
    CH_CFG_ST_FREQUENCY  = 1000 or 1000000

  To keep as much code in common as possible we create a function
  system_time_u32_us() for all boards which gives system time since
  boot in microseconds, and which wraps at 0xFFFFFFFF.

  On top of this base function we build get_systime_us32() which has
  the same property, but which also maintains timer_base_us64 to allow
  for micros64()
*/

#if CH_CFG_ST_RESOLUTION == 16
static uint32_t system_time_u32_us(void)
{
    systime_t now = chVTGetSystemTimeX();
#if CH_CFG_ST_FREQUENCY != 1000000U
    now *= 1000000U/CH_CFG_ST_FREQUENCY;
#endif
    static systime_t last_systime;
    static uint32_t timer_base_us32;
    uint16_t dt = now - last_systime;
    last_systime = now;
    timer_base_us32 += dt;
    return timer_base_us32;
}
#elif CH_CFG_ST_RESOLUTION == 32
static uint32_t system_time_u32_us(void)
{
    systime_t now = chVTGetSystemTimeX();
#if CH_CFG_ST_FREQUENCY != 1000000U
    now *= 1000000U/CH_CFG_ST_FREQUENCY;
#endif
    return now;
}
#else
#error "unsupported timer resolution"
#endif

// offset for micros64()
static uint64_t timer_base_us64;

static uint32_t get_systime_us32(void)
{
    static uint32_t last_us32;
    uint32_t now = system_time_u32_us();
    if (now < last_us32) {
        const uint64_t dt_us = 0x100000000ULL;
        timer_base_us64 += dt_us;
    }
    last_us32 = now;
    return now;
}

/*
  for the exposed functions we use chSysGetStatusAndLockX() to prevent
  an interrupt changing the globals while allowing this call from any
  context
*/

uint64_t hrt_micros64()
{
    syssts_t sts = chSysGetStatusAndLockX();
    uint32_t now = get_systime_us32();
    uint64_t ret = timer_base_us64 + now;
    chSysRestoreStatusX(sts);
    return ret;
}

uint32_t hrt_micros32()
{
    syssts_t sts = chSysGetStatusAndLockX();
    uint32_t ret = get_systime_us32();
    chSysRestoreStatusX(sts);
    return ret;
}

uint32_t hrt_millis32()
{
    return (uint32_t)(hrt_micros64() / 1000U);
}
