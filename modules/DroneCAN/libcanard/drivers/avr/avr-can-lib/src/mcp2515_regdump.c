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
 * $Id: mcp2515_regdump.c 6568 2008-06-16 13:56:26Z fabian $
 */
// -----------------------------------------------------------------------------

#include "mcp2515_private.h"
#ifdef	SUPPORT_FOR_MCP2515__

#include <stdio.h>
#include <avr/pgmspace.h>

// -----------------------------------------------------------------------------
void mcp2515_regdump(void)
{
	uint8_t mode = mcp2515_read_register( CANSTAT );
	
	// change to configuration mode
	mcp2515_change_operation_mode( (1<<REQOP2) );
	
	printf_P("MCP2515 Regdump:\n");
	uint8_t i;
	for (i=0; i < 16; i++) {
		printf_P("%3i: %02x   ", i, mcp2515_read_register(i));
		printf_P("%3i: %02x   ", i+16*1, mcp2515_read_register(i+16*1));
		printf_P("%3i: %02x   ", i+16*2, mcp2515_read_register(i+16*2));
		printf_P("%3i: %02x   ", i+16*3, mcp2515_read_register(i+16*3));
		printf_P("%3i: %02x   ", i+16*4, mcp2515_read_register(i+16*4));
		printf_P("%3i: %02x   ", i+16*5, mcp2515_read_register(i+16*5));
		printf_P("%3i: %02x   ", i+16*6, mcp2515_read_register(i+16*6));
		printf_P("%3i: %02x\n", i+16*7, mcp2515_read_register(i+16*7));
	}
	
	mcp2515_change_operation_mode( mode );
}

#endif	// SUPPORT_FOR_MCP2515__
