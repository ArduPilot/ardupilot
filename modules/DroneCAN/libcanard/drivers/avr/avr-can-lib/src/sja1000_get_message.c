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
 * $Id$
 */
// ----------------------------------------------------------------------------

#include "sja1000_private.h"
#ifdef	SUPPORT_FOR_SJA1000__

// ----------------------------------------------------------------------------

uint8_t sja1000_get_message(can_t *msg)
{
	uint8_t frame_info;
	uint8_t address;
	
	// check if there is actually a message in the buffers
	if (!sja1000_check_message())
		return FALSE;
	
	frame_info = sja1000_read(16);
	msg->length = frame_info & 0x0f;
	
	if (frame_info & (1<<FF))
	{
		// read extended identifier
		msg->flags.extended = 1;
		
		uint32_t tmp;
		uint8_t *ptr = (uint8_t *) &tmp;
		
		*ptr       = sja1000_read(20);
		*(ptr + 1) = sja1000_read(19);
		*(ptr + 2) = sja1000_read(18);
		*(ptr + 3) = sja1000_read(17);
		
		msg->id = tmp >> 3;
		
		/* equivalent to:
		msg->id	 = sja1000_read(20) >> 3;
		msg->id |= (uint16_t) sja1000_read(19) << 5;
		msg->id |= (uint32_t) sja1000_read(18) << 13;
		msg->id |= (uint32_t) sja1000_read(17) << 21;*/
		
		address = 21;
	}
	else
	{
		// read standard identifier
		msg->flags.extended = 0;
		
		uint32_t *ptr32 = &msg->id;		// used to supress a compiler warning
		uint16_t *ptr = (uint16_t *) ptr32;
		
		*(ptr + 1) = 0;
		
		*ptr  = sja1000_read(18) >> 5;
		*ptr |= sja1000_read(17) << 3;
		
		address = 19;
	}
	
	
	if (frame_info & (1<<RTR)) {
		msg->flags.rtr = 1;
	}
	else {
		msg->flags.rtr = 0;
		
		// read data
		for (uint8_t i = 0; i < msg->length; i++) {
			msg->data[i] = sja1000_read(address + i);
		}
	}
	
	// release buffer
	sja1000_write(CMR, (1<<RRB));
	
	CAN_INDICATE_RX_TRAFFIC_FUNCTION;
	
	return TRUE;
}

#endif	// SUPPORT_FOR_SJA1000__
