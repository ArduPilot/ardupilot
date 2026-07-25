// -----------------------------------------------------------------------------
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
 * $Id: at90can_send_buf_message.c 6837 2008-11-16 19:05:15Z fabian $
 */
// -----------------------------------------------------------------------------

#include "at90can_private.h"
#if defined(SUPPORT_FOR_AT90CAN__) && CAN_TX_BUFFER_SIZE > 0

#include <string.h>

// -----------------------------------------------------------------------------
uint8_t at90can_send_buffered_message(const can_t *msg)
{
	// check if there is any free buffer left
#if CAN_FORCE_TX_ORDER
	if (_transmission_in_progress)
#else
	if (_find_free_mob() == 0xff)
#endif
	{
		can_t *buf = can_buffer_get_enqueue_ptr(&can_tx_buffer); 
		
		if (buf == NULL)
			return 0;		// buffer full
		
		// copy message to the buffer
		memcpy( buf, msg, sizeof(can_t) );
		
		// In the interrupt it is checked if there are any waiting messages
		// in the queue, otherwise the interrupt will be disabled.
		// So, if the transmission finished while we are in this routine the 
		// message will be queued but not send.
		// Therefore interrupts have to disabled while putting the message
		// to the queue.
		bool enqueued = false;
		
#if CAN_FORCE_TX_ORDER
		ENTER_CRITICAL_SECTION;
		if (_transmission_in_progress)
		{
			can_buffer_enqueue(&can_tx_buffer);
			enqueued = true;
		}
		LEAVE_CRITICAL_SECTION;
#endif
		
		if (enqueued) {
			return 1;
		}
		else {
			// buffer gets free while we where preparing the message
			// => send message directly
			return at90can_send_message( msg );
		}
	}
	else
	{
		return at90can_send_message( msg );
	}
}

#endif	// SUPPORT_FOR_AT90CAN__
