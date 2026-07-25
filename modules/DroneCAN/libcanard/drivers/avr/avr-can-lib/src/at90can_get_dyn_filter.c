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
 * $Id: at90can_get_dyn_filter.c 6837 2008-11-16 19:05:15Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

// ----------------------------------------------------------------------------

uint8_t at90can_get_filter(uint8_t number, can_filter_t *filter)
{
	if (number > 14) {
		// it is only possible to serve a maximum of 15 filters
		return 0;
	}
	
	// load corresponding MOb page
	CANPAGE = number << 4;
	
	if ((CANCDMOB & 0xc0) == 0) {
		// MOb is currently not used.
		return 2;
	}
	else if ((CANCDMOB & 0xc0) == (1 << CONMOB1))
	{
		// MOb is configured to receive message => read filter.
		if (CANIDM4 & (1 << RTRMSK))
		{
			if (CANIDT4 & (1 << RTRMSK))
			{
				// receive only messages with RTR-bit set
				filter->flags.rtr = 0x3;
			}
			else {
				filter->flags.rtr = 0x2;
			}
		}
		else {
			// receive all message, independent from RTR-bit
			filter->flags.rtr = 0;
		}
		
		#if SUPPORT_EXTENDED_CANID
		
		if ((CANIDM4 & (1 << IDEMSK)) && (CANCDMOB & (1 << IDE)))
		{
			filter->flags.extended = 0x3;
			
			// extended id
			uint32_t mask;
			mask  = (uint8_t)  CANIDM4 >> 3;
			mask |= (uint16_t) CANIDM3 << 5;
			mask |= (uint32_t) CANIDM2 << 13;
			mask |= (uint32_t) CANIDM1 << 21;
			
			filter->mask = mask;
			
			uint32_t id;
			id  = (uint8_t)  CANIDT4 >> 3;
			id |= (uint16_t) CANIDT3 << 5;
			id |= (uint32_t) CANIDT2 << 13;
			id |= (uint32_t) CANIDT1 << 21;
			
			// only the bits set in the mask are vaild for the id
			filter->id = id & mask;
		}
		else {
			if (CANIDM4 & (1 << IDEMSK)) {
				filter->flags.extended = 0x2;
			} else {
				filter->flags.extended = 0;
			}
			
			uint16_t mask;
			mask  = (uint8_t)  CANIDM2 >> 5;
			mask |= (uint16_t) CANIDM1 << 3;
			
			filter->mask = mask;
			
			uint16_t id;
			id  = (uint8_t)  CANIDT2 >> 5;
			id |= (uint16_t) CANIDT1 << 3;
			
			filter->id = id & mask;
		}
		
		#else
		
		uint16_t mask;
		mask  = (uint8_t)  CANIDM2 >> 5;
		mask |= (uint16_t) CANIDM1 << 3;
		
		filter->mask = mask;
		
		uint16_t id;
		id  = (uint8_t)  CANIDT2 >> 5;
		id |= (uint16_t) CANIDT1 << 3;
		
		filter->id = id & mask;
		
		#endif
		
		return 1;
	}
	
	// MOb is currently used to transmit a message.
	return 0xff;
}

#endif	// SUPPORT_FOR_AT90CAN__
