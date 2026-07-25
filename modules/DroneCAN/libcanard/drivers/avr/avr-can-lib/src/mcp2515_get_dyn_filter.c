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
 * $Id: mcp2515_get_dyn_filter.c 6837 2008-11-16 19:05:15Z fabian $
 */
// ----------------------------------------------------------------------------

#include "mcp2515_private.h"
#ifdef	SUPPORT_FOR_MCP2515__

// ----------------------------------------------------------------------------
// get a filter

uint8_t mcp2515_get_filter(uint8_t number, can_filter_t *filter)
{
	uint8_t mask_address;
	uint8_t filter_address;
	uint8_t temp;
	uint8_t mode = mcp2515_read_register(CANSTAT);
	
	if (number > 5)
		return 0;
	
	// change to configuration mode
	mcp2515_change_operation_mode( (1<<REQOP2) );
	
	if (number <= 1)
	{
		mask_address = RXM0SIDH;
		temp = mcp2515_read_register(RXB0CTRL);
	}
	else
	{
		mask_address = RXM1SIDH;
		temp = mcp2515_read_register(RXB1CTRL);
	}
	
	temp &= (1<<RXM1)|(1<<RXM0);
	
	if (temp == 0) {
		// filter and masks are disabled
		#if SUPPORT_EXTENDED_CANID
		filter->flags.extended = 0;
		#endif
		filter->flags.rtr = 0;
		filter->mask = 0;
		filter->id = 0;
		
		return 1;
	}
	
	#if SUPPORT_EXTENDED_CANID
	// transform bits so that they match the format from can.h
	temp >>= 5;
	temp = ~temp;
	if (temp & 1) temp = 0x3;
	
	filter->flags.extended = temp;
	#endif
	
	// read mask
	RESET(MCP2515_CS);
	spi_putc(SPI_READ);
	spi_putc(mask_address);
	mcp2515_read_id(&filter->mask);
	SET(MCP2515_CS);
	
	if (number <= 2)
	{
		filter_address = RXF0SIDH + number * 4;
	}
	else
	{
		filter_address = RXF3SIDH + (number - 3) * 4;
	}
	
	// read filter
	RESET(MCP2515_CS);
	spi_putc(SPI_READ);
	spi_putc(filter_address);
	mcp2515_read_id(&filter->id);
	SET(MCP2515_CS);
	
	// restore previous mode
	mcp2515_change_operation_mode( mode );
	
	return 1;
}

#endif	// SUPPORT_FOR_MCP2515__
