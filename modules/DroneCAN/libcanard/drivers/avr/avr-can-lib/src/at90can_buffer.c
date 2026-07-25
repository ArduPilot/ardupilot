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
 * $Id: at90can_buffer.c 6565 2008-06-14 11:35:58Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

// ----------------------------------------------------------------------------
// Checks if there is any waiting message in the registers

bool at90can_check_message(void)
{
	#if CAN_RX_BUFFER_SIZE == 0
	if (_messages_waiting > 0)
		return true;
	else
		return false;
	#else
	return !can_buffer_empty( &can_rx_buffer );
	#endif
}

// ----------------------------------------------------------------------------

bool at90can_check_free_buffer(void)
{
	#if CAN_TX_BUFFER_SIZE == 0
	// check if there is any free MOb
	if (_free_buffer > 0)
		return true;
	else
		return false;
	#else
	return !can_buffer_full( &can_tx_buffer );
	#endif
}

#endif	// SUPPORT_FOR_AT90CAN__
