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
 * $Id: mcp2515_set_mode.c 5752 2008-01-14 09:30:21Z fabian $
 */
// ----------------------------------------------------------------------------

#include "sja1000_private.h"
#ifdef	SUPPORT_FOR_SJA1000__

// ----------------------------------------------------------------------------
void sja1000_set_mode(can_mode_t mode)
{
	uint8_t reg = 0;
	
	// enter reset mode
	sja1000_write(MOD, (1<<AFM) | (1<<RM));
	
	if (mode == LISTEN_ONLY_MODE) {
		reg = (1<<LOM);
	}
	else if (mode == LOOPBACK_MODE) {
		reg = (1<<STM);
	}
	
	// set new mode
	sja1000_write(MOD, (1<<AFM) | (1<<RM) | reg);
	
	// leave reset mode
	sja1000_write(MOD, (1<<AFM) | reg);
}

#endif	// SUPPORT_FOR_SJA1000__
