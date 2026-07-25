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
 * $Id: at90can_get_message.c 7224 2009-01-25 20:19:32Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

bool at90can_copy_mob_to_message(can_t *msg)
{
	// read status
	uint8_t cancdmob = CANCDMOB;
	
	// read length
	msg->length = cancdmob & 0x0f;
	
	#if SUPPORT_EXTENDED_CANID
	
	if (cancdmob & (1 << IDE))
	{
		// extended identifier
		uint32_t tmp;
		uint8_t *ptr = (uint8_t *) &tmp;
		
		*ptr       = CANIDT4;
		*(ptr + 1) = CANIDT3;
		*(ptr + 2) = CANIDT2;
		*(ptr + 3) = CANIDT1;
		
		msg->id = tmp >> 3;
		
		/* equivalent to:
		msg->id  = (uint8_t)  CANIDT4 >> 3;
		msg->id |= (uint32_t) CANIDT3 << 5;
		msg->id |= (uint32_t) CANIDT2 << 13;
		msg->id |= (uint32_t) CANIDT1 << 21;
		*/
		
		msg->flags.extended = 1;
	}
	else
	{
		// standard identifier
		uint16_t id;
		
		id  = (uint8_t)  CANIDT2 >> 5;
		id |= (uint16_t) CANIDT1 << 3;
		
		msg->id = (uint32_t) id;
		msg->flags.extended = 0;
	}
	
	#else
	
	if (cancdmob & (1 << IDE))
	{
		return false;
	}
	else
	{
		// standard identifier
		msg->id  = (uint8_t)  CANIDT2 >> 5;
		msg->id |= (uint16_t) CANIDT1 << 3;
	}
	
	#endif
	
	if (CANIDT4 & (1 << RTRTAG)) {
		msg->flags.rtr = 1;
	}
	else {
		msg->flags.rtr = 0;
		
		// read data
		uint8_t *p = msg->data;
		for (uint8_t i = 0;i < msg->length;i++) {
			*p++ = CANMSG;
		}
	}
	
	#if SUPPORT_TIMESTAMPS
	msg->timestamp = CANSTM;
	#endif
	
	return true;
}

// ----------------------------------------------------------------------------

uint8_t at90can_get_message(can_t *msg)
{
	bool found = false;
	uint8_t mob;
	
	// check if there is any waiting message
	if (!at90can_check_message())
		return 0;
	
	// find the MOb with the received message
	for (mob = 0; mob < 15; mob++)
	{
		CANPAGE = mob << 4;
		
		if (CANSTMOB & (1<<RXOK))
		{
			found = true;
			
			// clear flags
			CANSTMOB &= 0;
			break;
		}
	}
	
	if (!found) {
		return 0;		// should never happen
	}
	
	found = at90can_copy_mob_to_message( msg );
	
	#if CAN_RX_BUFFER_SIZE == 0
	// mark message as processed
	ENTER_CRITICAL_SECTION;
	_messages_waiting--;
	LEAVE_CRITICAL_SECTION;
	#endif
	
	// re-enable interrupts
	_enable_mob_interrupt( mob );
	
	// clear flags
	CANCDMOB = (1 << CONMOB1) | (CANCDMOB & (1 << IDE));
	
	if (found) {
		return (mob + 1);
	}
	else {
		// only if SUPPORT_EXTENDED_CANID=0 and a extended message was received
		return 0;
	}
}

#endif	// SUPPORT_FOR_AT90CAN__
