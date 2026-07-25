// ----------------------------------------------------------------------------
/*
 * Copyright (c) 2007 Fabian Greif, Roboterclub Aachen e.V.
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: at90can_set_dyn_filter.c 6837 2008-11-16 19:05:15Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

// ----------------------------------------------------------------------------
bool at90can_set_filter(uint8_t number, const can_filter_t *filter)
{
	if (number > 14) {
		// it is only possible to serve a maximum of 15 filters
		return false;
	}
	
	// set CAN Controller to standby mode
	_enter_standby_mode();
	
	CANPAGE = number << 4;
	
	CANSTMOB = 0;
	CANCDMOB = 0;
	
	#if SUPPORT_EXTENDED_CANID
	
	if (filter->flags.extended == 0x3)
	{
		// extended identifier
		CANIDT4 = (uint8_t)  filter->id << 3;
		CANIDT3 = 			 filter->id >> 5;
		CANIDT2 =            filter->id >> 13;
		CANIDT1 =            filter->id >> 21;
		
		CANIDM4 = ((uint8_t) filter->mask << 3) | (1 << IDEMSK);
		CANIDM3 = 			 filter->mask >> 5;
		CANIDM2 =            filter->mask >> 13;
		CANIDM1 =            filter->mask >> 21;
		
		CANCDMOB |= (1 << IDE);
	}
	else {
		CANIDT4 = 0;
		CANIDT3 = 0;
		CANIDT2 = (uint8_t)  filter->id << 5;
		CANIDT1 = (uint16_t) filter->id >> 3;
		
		if (filter->flags.extended) {
			CANIDM4 = (1 << IDEMSK);		// receive only standard frames
		} else {
			CANIDM4 = 0;					// receive all frames
		}
		
		CANIDM3 = 0;
		CANIDM2 = (uint8_t)  filter->mask << 5;
		CANIDM1 = (uint16_t) filter->mask >> 3;
	}
	
	#else
	
	CANIDT4 = 0;
	CANIDT3 = 0;
	CANIDT2 = (uint8_t)  filter->id << 5;
	CANIDT1 = (uint16_t) filter->id >> 3;
	
	CANIDM4 = (1 << IDEMSK);
	CANIDM3 = 0;
	CANIDM2 = (uint8_t)  filter->mask << 5;
	CANIDM1 = (uint16_t) filter->mask >> 3;
	
	#endif
	
	if (filter->flags.rtr & 0x2) {
		CANIDM4 |= (1 << RTRMSK);
		
		if (filter->flags.rtr & 0x1)
			CANIDT4 |= (1 << RTRMSK);		// only RTR-frames
	}
	
	CANCDMOB |= (1<<CONMOB1);
	
	_enable_mob_interrupt(number);
	
	// re-enable CAN Controller 
	_leave_standby_mode();
	
	return true;
}

#endif	// SUPPORT_FOR_AT90CAN__
