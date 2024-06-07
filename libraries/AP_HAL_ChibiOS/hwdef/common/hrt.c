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

#include "../../../AP_Math/div1000.h"

/*
  we have 4 possible configurations of boards, made up of boards that
  have the following properties:

    CH_CFG_ST_RESOLUTION = 16 or 32
    CH_CFG_ST_FREQUENCY  = 1000 or 1000000

  To keep as much code in common as possible we create a function
  system_time_u32_us() for all boards which gives system time since
  boot in microseconds, and which wraps at 0xFFFFFFFF.

  On top of this base function we build get_systime_us32() which has
  the same property, but which allows for a startup offset
  for micros64()
*/

#if CH_CFG_ST_RESOLUTION == 16
static uint32_t system_time_u32_us(void)
{
    systime_t now = chVTGetSystemTimeX();
#if CH_CFG_ST_FREQUENCY != 1000000U
    #error "must use 32 bit timer if system clock not 1MHz"
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

static uint32_t get_systime_us32(void)
{
    uint32_t now = system_time_u32_us();
#ifdef AP_BOARD_START_TIME
    now += AP_BOARD_START_TIME;
#endif
    return now;
}

/*
  for the exposed functions we use chVTGetTimeStampI which handles
  wrap and directly gives a uint64_t (aka systimestamp_t)
*/

static uint64_t hrt_micros64I(void)
{
    uint64_t ret = chVTGetTimeStampI();
#if CH_CFG_ST_FREQUENCY != 1000000U
    ret *= 1000000U/CH_CFG_ST_FREQUENCY;
#endif
#ifdef AP_BOARD_START_TIME
    ret += AP_BOARD_START_TIME;
#endif
    return ret;
}

static inline bool is_locked(void) {
    return !port_irq_enabled(port_get_irq_status());
}

uint64_t hrt_micros64()
{
    if (is_locked()) {
        return hrt_micros64I();
    } else if (port_is_isr_context()) {
        uint64_t ret;
        chSysLockFromISR();
        ret = hrt_micros64I();
        chSysUnlockFromISR();
        return ret;
    } else {
        uint64_t ret;
        chSysLock();
        ret = hrt_micros64I();
        chSysUnlock();
        return ret;
    }
}

uint32_t hrt_micros32()
{
#if CH_CFG_ST_RESOLUTION == 16
    // boards with 16 bit timers need to call get_systime_us32() in a
    // lock state because on those boards we have local static
    // variables that need protection
    if (is_locked()) {
        return get_systime_us32();
    } else if (port_is_isr_context()) {
        uint32_t ret;
        chSysLockFromISR();
        ret = get_systime_us32();
        chSysUnlockFromISR();
        return ret;
    } else {
        uint32_t ret;
        chSysLock();
        ret = get_systime_us32();
        chSysUnlock();
        return ret;
    }
#else
    return get_systime_us32();
#endif
}

uint64_t hrt_millis64()
{
    return uint64_div1000(hrt_micros64());
}
        
uint32_t hrt_millis32()
{
    return (uint32_t)(hrt_millis64());
}

