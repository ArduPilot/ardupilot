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
/*
 * HRT GPT configuration
 */


//static void hrt_cb(GPTDriver*);
static uint64_t timer_base = 0;
static const GPTConfig hrtcfg = {
  1000000,    /* 1MHz timer clock.*/
  NULL,   /* Timer callback.*/
  0,
  0
};


void hrt_init()
{
	gptStart(&HRT_TIMER, &hrtcfg);
	gptStartContinuous(&HRT_TIMER, 0xFFFF);
}

uint64_t hrt_micros()
{
	static volatile uint64_t last_micros = 0;
	static volatile uint64_t micros;

	syssts_t sts = chSysGetStatusAndLockX();
	micros = timer_base + (uint64_t)gptGetCounterX(&HRT_TIMER);
	// we are doing this to avoid an additional interupt routing
	// since we are definitely going to get called atleast once in
	// a full timer period
	if(last_micros > micros) {
		timer_base += 0x10000;
		micros += 0x10000;
	}
	last_micros = micros;
	chSysRestoreStatusX(sts);
	return micros;
}

/*
static void  hrt_cb(GPTDriver* gptd)
{
	timer_base += 0x10000;
}
*/
