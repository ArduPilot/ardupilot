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

uint8_t sja1000_send_message(const can_t *msg)
{
	uint8_t frame_info;
	uint8_t address;
	
	if (!sja1000_check_free_buffer() || (msg->length > 8))
		return FALSE;
	
	frame_info = msg->length | ((msg->flags.rtr) ? (1<<RTR) : 0);
	
	if (msg->flags.extended)
	{
		// write frame info
		sja1000_write(TX_INFO, frame_info | (1<<FF));
		
		// write extended identifier
		sja1000_write(20, msg->id << 3);
		sja1000_write(19, msg->id >> 5);
		sja1000_write(18, msg->id >> 13);
		sja1000_write(17, msg->id >> 21);
		
		address = 21;
	}
	else
	{
		// write frame info
		sja1000_write(TX_INFO, frame_info);
		
		const uint32_t *ptr32 = &msg->id;		// used to supress a compiler warning
		uint16_t *ptr = (uint16_t *) ptr32;
		
		// write standard identifier
		sja1000_write(18, *ptr << 5);
		sja1000_write(17, *ptr >> 3);
		
		address = 19;
	}
	
	if (!msg->flags.rtr)
	{
		for (uint8_t i = 0;i < msg->length; i++) {
			sja1000_write(address + i, msg->data[i]);
		}
	}
	
	// send buffer
	sja1000_write(CMR, (1<<TR));
	
	CAN_INDICATE_TX_TRAFFIC_FUNCTION;
	
	return TRUE;
}

#endif	// SUPPORT_FOR_SJA1000__
