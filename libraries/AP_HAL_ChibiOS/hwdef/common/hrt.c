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
// High Resolution Timer

#include "ch.h"
#include "hal.h"
#include "hrt.h"
#include <stdint.h>

static uint64_t timer_base_us64;
#if CH_CFG_ST_RESOLUTION == 16
static uint32_t timer_base_us32;
#endif
static uint32_t timer_base_ms;
static volatile systime_t last_systime;

#if CH_CFG_ST_RESOLUTION == 16
static uint32_t get_systime_us32(void)
{
    systime_t now = chVTGetSystemTimeX();
#if CH_CFG_ST_FREQUENCY != 1000000
    now *= (1000000UL / CH_CFG_ST_FREQUENCY);
#endif
    if (now < last_systime) {
        uint32_t last_u32 = timer_base_us32;
        timer_base_us32 += (uint32_t)TIME_MAX_SYSTIME;
        if (timer_base_us32 < last_u32) {
            timer_base_us64 += ((uint32_t)-1);
            timer_base_ms += ((uint32_t)-1)/1000;
        }
    }
    last_systime = now;
    return timer_base_us32 + (uint32_t)now;
}

#elif CH_CFG_ST_RESOLUTION == 32
static uint32_t get_systime_us32(void)
{
    systime_t now = chVTGetSystemTimeX();
#if CH_CFG_ST_FREQUENCY != 1000000
    now *= (1000000UL / CH_CFG_ST_FREQUENCY);
#endif
    if (now < last_systime) {
        timer_base_us64 += TIME_MAX_SYSTIME;
        timer_base_ms += TIME_MAX_SYSTIME/1000;
    }
    last_systime = now;
    return now;
}
#else
#error "unsupported timer resolution"
#endif

/*
  we use chSysGetStatusAndLockX() to prevent an interrupt while
  allowing this call from any context
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
    syssts_t sts = chSysGetStatusAndLockX();
    uint32_t now = get_systime_us32();
    uint32_t ret = (now / 1000U) + timer_base_ms;
    chSysRestoreStatusX(sts);
    return ret;
}
