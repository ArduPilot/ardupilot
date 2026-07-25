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
 * $Id$
 */
// ----------------------------------------------------------------------------

#include "sja1000_private.h"
#ifdef	SUPPORT_FOR_SJA1000__

#if !SJA1000_MEMORY_MAPPED
#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

void sja1000_write(uint8_t address, uint8_t data)
{
	// set address
	SET(SJA1000_ALE);
	PORT(SJA1000_DATA) = address;
	_NOP();
	RESET(SJA1000_ALE);
	
	// write data
	PORT(SJA1000_DATA) = data;
	RESET(SJA1000_WR);
	_NOP();
	SET(SJA1000_WR);
}

uint8_t sja1000_read(uint8_t address)
{
	uint8_t data;
	
	// set address
	SET(SJA1000_ALE);
	PORT(SJA1000_DATA) = address;
	_NOP();
	RESET(SJA1000_ALE);
	DDR(SJA1000_DATA) = 0;
	
	// read data
	RESET(SJA1000_RD);
	_NOP();
	data = PIN(SJA1000_DATA);
	SET(SJA1000_RD);
	DDR(SJA1000_DATA) = 0xff;
	
	return data;
}
#endif

// ----------------------------------------------------------------------------
// useable can-bitrates (for calculation see http://www.kvaser.com/index.htm)

const uint8_t _sja1000_cnf[8][2] PROGMEM = {
	// 10 kbps
	{	0xe7,
		0x4d
	},
	// 20 kbps
	{	0xd3,
		0x4d
	},
	// 50 kbps
	{	0xc7,
		0x4d
	},
	// 100 kbps
	{	0xc3,
		0x4d
	},
	// 125 kbps
	{	(1<<_SJW0)|(1<<_BRP1)|(1<<_BRP0),
		(1<<TSEG13)|(1<<TSEG12)|(1<<TSEG20)
	},
	// 250 kbps
	{	(1<<_SJW0)|(1<<_BRP0),
		(1<<TSEG13)|(1<<TSEG12)|(1<<TSEG20)
	},
	// 500 kbps
	{	(1<<_SJW0),
		(1<<TSEG13)|(1<<TSEG12)|(1<<TSEG20)
	},
	// 1 Mbps
	{	(1<<_SJW0),
		(1<<TSEG12)|(1<<TSEG20)
	}
};

// ----------------------------------------------------------------------------
// init sja1000-interface

bool sja1000_init(uint8_t bitrate)
{
	if (bitrate >= 8)
		return false;
	
	#if !SJA1000_MEMORY_MAPPED
		SET(SJA1000_WR);
		SET(SJA1000_RD);
		RESET(SJA1000_ALE);
		RESET(SJA1000_CS);
		
		SET_OUTPUT(SJA1000_WR);
		SET_OUTPUT(SJA1000_RD);
		SET_OUTPUT(SJA1000_ALE);

		SET_OUTPUT(SJA1000_CS);
		DDR(SJA1000_DATA) = 0xff;
	#endif
	
	// enter reset mode
	sja1000_write(MOD, (1<<RM)|(1<<AFM));
	
	// choose PeliCAN-Mode
	sja1000_write(CDR, (1<<CANMODE) | SJA1000_CLOCK_REGISTER);
	
	// select the bitrate configuration
	sja1000_write(BTR0, pgm_read_byte(&_sja1000_cnf[bitrate][0]));
	sja1000_write(BTR1, pgm_read_byte(&_sja1000_cnf[bitrate][1]));
	
	// filter are not practical useable, so we disable them
	sja1000_write(AMR0, 0xff);
	sja1000_write(AMR1, 0xff);
	sja1000_write(AMR2, 0xff);
	sja1000_write(AMR3, 0xff);
	
	// set output driver configuration
	sja1000_write(OCR, (1<<OCTP0)|(1<<OCTN0)|(1<<OCMODE1));
	
	// enable receive interrupt
	sja1000_write(IER, (1<<RIE));
	
	// leave reset-mode
	sja1000_write(MOD, (1<<AFM));
	
	return true;
}

#endif	// SUPPORT_FOR_SJA1000__
