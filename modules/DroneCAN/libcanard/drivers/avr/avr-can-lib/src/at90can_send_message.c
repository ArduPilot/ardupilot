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
 * $Id: at90can_send_message.c 6837 2008-11-16 19:05:15Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

#include <string.h>

// ----------------------------------------------------------------------------
/**
 * \brief	Copy data form a message in RAM to the actual registers
 * 
 * \warning this function assumes CANPAGE to be set properly before the call
 */
void at90can_copy_message_to_mob(const can_t *msg)
{
	// write DLC (Data Length Code)
	CANCDMOB = msg->length;
	
	#if SUPPORT_EXTENDED_CANID
	
	if (msg->flags.extended) {
		// extended CAN ID
		CANCDMOB |= (1 << IDE);
		
		CANIDT4 = (uint8_t)  msg->id << 3;
		
		uint32_t temp = msg->id << 3;
		uint8_t *ptr = (uint8_t *) &temp;
		
		CANIDT3 = *(ptr + 1);
		CANIDT2 = *(ptr + 2);
		CANIDT1 = *(ptr + 3);
	}
	else {
		// standard CAN ID
		CANIDT4 = 0;
		CANIDT3 = 0;
		CANIDT2 = (uint8_t)  msg->id << 5;
		CANIDT1 = (uint16_t) msg->id >> 3;
	}
	
	#else
	
	CANIDT4 = 0;
	CANIDT3 = 0;
	CANIDT2 = (uint8_t)  msg->id << 5;
	CANIDT1 = (uint16_t) msg->id >> 3;
	
	#endif
	
	if (msg->flags.rtr) {
		CANIDT4 |= (1<<RTRTAG);
	}
	else {
		const uint8_t *p = msg->data;
		for (uint8_t i = 0;i < msg->length;i++) {
			CANMSG = *p++;
		}
	}
}

// ----------------------------------------------------------------------------
uint8_t at90can_send_message(const can_t *msg)
{
	// check if there is any free MOb
	uint8_t mob = _find_free_mob();
	if (mob >= 15)
		return 0;
	
	// load corresponding MOb page ...
	CANPAGE = (mob << 4);
	
	// clear flags
	CANSTMOB &= 0;
	
	// ... and copy the data
	at90can_copy_message_to_mob( msg );
	
	// enable interrupt
	_enable_mob_interrupt(mob);
	
	ENTER_CRITICAL_SECTION;
	#if CAN_TX_BUFFER_SIZE == 0
		_free_buffer--;
	#elif CAN_FORCE_TX_ORDER
		_transmission_in_progress = 1;
	#endif
	LEAVE_CRITICAL_SECTION;
	
	// enable transmission
	CANCDMOB |= (1<<CONMOB0);
	
	return (mob + 1);
}

#endif	// SUPPORT_FOR_AT90CAN__
