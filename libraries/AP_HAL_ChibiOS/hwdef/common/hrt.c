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

static uint64_t timer_base_us;
static uint32_t timer_base_ms;
static volatile systime_t last_systime;

static uint32_t get_systime_us32(void)
{
    systime_t now = chVTGetSystemTimeX();
    if (now < last_systime) {
        timer_base_us += TIME_MAX_SYSTIME;
        timer_base_ms += TIME_MAX_SYSTIME/1000;
    }
    last_systime = now;
    return now;
}

/*
  we use chSysGetStatusAndLockX() to prevent an interrupt while
  allowing this call from any context
*/

uint64_t hrt_micros64()
{
    syssts_t sts = chSysGetStatusAndLockX();
    uint32_t now = get_systime_us32();
    uint64_t ret = timer_base_us + now;
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
