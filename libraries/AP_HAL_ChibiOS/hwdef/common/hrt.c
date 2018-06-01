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

static uint64_t timer_base = 0;

uint64_t hrt_micros()
{
	static volatile uint64_t last_micros;

    /*
      use chSysGetStatusAndLockX() to prevent an interrupt while
      allowing this call from any context
     */
	syssts_t sts = chSysGetStatusAndLockX();
	uint64_t micros;
	micros = timer_base + (uint64_t)chVTGetSystemTimeX();
	// we are doing this to avoid an additional interupt routing
	// since we are definitely going to get called atleast once in
	// a full timer period
	if (last_micros > micros) {
        const uint64_t step = ST2US(1ULL<<CH_CFG_ST_RESOLUTION);
		timer_base += step;
		micros += step;
	}
	last_micros = micros;
	chSysRestoreStatusX(sts);
	return micros;
}
