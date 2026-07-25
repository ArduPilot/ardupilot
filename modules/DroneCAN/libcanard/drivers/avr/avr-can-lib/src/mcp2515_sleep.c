// ----------------------------------------------------------------------------
/*
 * Copyright (c) 2007 Fabian Greif, Roboterclub Aachen e.V., Frédéric Lamorce
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
 * $Id: mcp2515_sleep.c 8086 2009-07-14 14:08:25Z fabian $
 */
// ----------------------------------------------------------------------------

#include "mcp2515_private.h"
#ifdef	SUPPORT_FOR_MCP2515__

// ----------------------------------------------------------------------------
void
mcp2515_sleep(void)
{
	// put also the 2551 in standby mode
	// for this, connect RX1BF to the RS pin of the 2551
	mcp2515_bit_modify(BFPCTRL, (1<<B1BFS), (1<<B1BFS));

	// put the 2515 in sleep more
	mcp2515_set_mode(SLEEP_MODE);

	// enable generating an interrupt for wakeup when activity on bus
	mcp2515_bit_modify(CANINTE, (1<<WAKIE), (1<<WAKIE));
}

// ----------------------------------------------------------------------------
void
mcp2515_wakeup(void)
{
	// reset int enable and cancel the interrupt flag
	mcp2515_bit_modify(CANINTE, (1<<WAKIE), 0);
	mcp2515_bit_modify(CANINTF, (1<<WAKIF), 0);

	// re-enable the 2551
	mcp2515_bit_modify(BFPCTRL, (1<<B1BFS), 0);

	// when we get up of sleep, we are in listen mode, return into normal mode
	mcp2515_set_mode(NORMAL_MODE);
}

#endif	// SUPPORT_FOR_MCP2515__
