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
 * $Id: mcp2515_buffer.c 6653 2008-09-02 13:51:25Z fabian $
 */
// ----------------------------------------------------------------------------

#include "mcp2515_private.h"
#ifdef	SUPPORT_FOR_MCP2515__

// ----------------------------------------------------------------------------
// check if there are any new messages waiting

bool mcp2515_check_message(void)
{
	#if defined(MCP2515_INT)
		return ((!IS_SET(MCP2515_INT)) ? true : false);
	#else
		#ifdef RXnBF_FUNKTION
			if (!IS_SET(MCP2515_RX0BF) || !IS_SET(MCP2515_RX1BF))
				return true;
			else
				return false;
		#else
			return ((mcp2515_read_status(SPI_RX_STATUS) & 0xC0) ? true : false);
		#endif
	#endif
}

// ----------------------------------------------------------------------------
// check if there is a free buffer to send messages

bool mcp2515_check_free_buffer(void)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);
	
	if ((status & 0x54) == 0x54)
		return false;		// all buffers used
	else
		return true;
}

#endif	// SUPPORT_FOR_MCP2515__
