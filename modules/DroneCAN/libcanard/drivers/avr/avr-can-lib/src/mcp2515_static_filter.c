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
 * $Id: mcp2515_static_filter.c 6564 2008-06-14 11:33:57Z fabian $
 */
// ----------------------------------------------------------------------------

#include "mcp2515_private.h"
#ifdef	SUPPORT_FOR_MCP2515__

// ----------------------------------------------------------------------------
// Filter setzen

void mcp2515_static_filter(const uint8_t *filter)
{
	// change to configuration mode
	mcp2515_bit_modify(CANCTRL, 0xe0, (1<<REQOP2));
	while ((mcp2515_read_register(CANSTAT) & 0xe0) != (1<<REQOP2))
		;
	
	mcp2515_write_register(RXB0CTRL, (1<<BUKT));
	mcp2515_write_register(RXB1CTRL, 0);
    
	uint8_t i, j;
	for (i = 0; i < 0x30; i += 0x10)
	{
		RESET(MCP2515_CS);
		spi_putc(SPI_WRITE);
		spi_putc(i);
		
		for (j = 0; j < 12; j++) 
		{
			if (i == 0x20 && j >= 0x08)
				break;
			
			spi_putc(pgm_read_byte(filter++));
		}
		SET(MCP2515_CS);
	}
	
	mcp2515_bit_modify(CANCTRL, 0xe0, 0);
}

#endif	// SUPPORT_FOR_MCP2515__
