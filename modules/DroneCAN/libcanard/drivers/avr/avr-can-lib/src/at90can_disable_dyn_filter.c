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
 * $Id: at90can_disable_dyn_filter.c 6721 2008-10-02 16:09:53Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

// ----------------------------------------------------------------------------
// disable mob

bool at90can_disable_filter(uint8_t number)
{
	if (number > 14)
	{
		if (number == CAN_ALL_FILTER)
		{
			// disable interrupts
			CANIE1 = 0;
			CANIE2 = 0;
			
			// disable all MObs
			for (uint8_t i = 0;i < 15;i++) {
				CANPAGE = (i << 4);
				
				// disable MOb (read-write required)
				CANCDMOB &= 0;
				CANSTMOB &= 0;
			}
			
			// mark all MObs as free
			#if CAN_RX_BUFFER_SIZE == 0
			_messages_waiting = 0;
			#endif
			
			#if CAN_TX_BUFFER_SIZE == 0
			_free_buffer = 15;
			#endif
			
			return true;
		}
		
		// it is only possible to serve a maximum of 15 filters
		return false;
	}
	
	// set CAN Controller to standby mode
	_enter_standby_mode();
	
	CANPAGE = number << 4;
	
	// reset flags
	CANSTMOB &= 0;
	CANCDMOB = 0;
	
	_disable_mob_interrupt(number);
	
	// re-enable CAN Controller 
	_leave_standby_mode();
	
	return true;
}

#endif	// SUPPORT_FOR_AT90CAN__
