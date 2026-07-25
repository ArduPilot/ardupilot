// coding: utf-8
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
 * $Id: mcp2515.c 8086 2009-07-14 14:08:25Z fabian $
 */
// ----------------------------------------------------------------------------
/* ---- Beispiel zum Einstellen des Bit Timings ----
 *	
 *	Fosc		= 16MHz
 *	BRP			= 7
 *	TQ 			= 2 * (BRP + 1) / Fosc
 *				= 1 uS
 *
 *	Sync Seg	= 					= 1 TQ
 *	Prop Seg	= (PRSEG + 1) * TQ	= 1 TQ
 *	Phase Seg1	= (PHSEG1 + 1) * TQ	= 3 TQ
 *	Phase Seg2	= (PHSEG2 + 1) * TQ = 3 TQ
 *									--------
 *									  8 TQ
 *	
 *	Bus speed	= 1 / ((Total # of TQ) * TQ)
 *				= 1 / (8 * TQ) = 125 kHz
 */
// -------------------------------------------------------------------------

#include "mcp2515_private.h"
#ifdef	SUPPORT_FOR_MCP2515__

#ifndef	MCP2515_CLKOUT_PRESCALER
	#error	MCP2515_CLKOUT_PRESCALER not defined!
#elif MCP2515_CLKOUT_PRESCALER == 0
	#define	CLKOUT_PRESCALER_	0x0
#elif MCP2515_CLKOUT_PRESCALER == 1
	#define	CLKOUT_PRESCALER_	0x4
#elif MCP2515_CLKOUT_PRESCALER == 2
	#define	CLKOUT_PRESCALER_	0x5
#elif MCP2515_CLKOUT_PRESCALER == 4
	#define	CLKOUT_PRESCALER_	0x6
#elif MCP2515_CLKOUT_PRESCALER == 8
	#define	CLKOUT_PRESCALER_	0x7
#else
	#error	invaild value of MCP2515_CLKOUT_PRESCALER
#endif

// -------------------------------------------------------------------------
void mcp2515_write_register( uint8_t adress, uint8_t data )
{
	RESET(MCP2515_CS);
	
	spi_putc(SPI_WRITE);
	spi_putc(adress);
	spi_putc(data);
	
	SET(MCP2515_CS);
}

// -------------------------------------------------------------------------
uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;
	
	RESET(MCP2515_CS);
	
	spi_putc(SPI_READ);
	spi_putc(adress);
	
	data = spi_putc(0xff);	
	
	SET(MCP2515_CS);
	
	return data;
}

// -------------------------------------------------------------------------
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	RESET(MCP2515_CS);
	
	spi_putc(SPI_BIT_MODIFY);
	spi_putc(adress);
	spi_putc(mask);
	spi_putc(data);
	
	SET(MCP2515_CS);
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;
	
	RESET(MCP2515_CS);
	
	spi_putc(type);
	data = spi_putc(0xff);
	
	SET(MCP2515_CS);
	
	return data;
}

// -------------------------------------------------------------------------

const uint8_t _mcp2515_cnf[8][3] PROGMEM = {
	// 10 kbps
	{	0x04,
		0xb6,
		0xe7
	},
	// 20 kbps
	{	0x04,
		0xb6,
		0xd3
	},
	// 50 kbps
	{	0x04,
		0xb6,
		0xc7
	},
	// 100 kbps
	{	0x04,
		0xb6,
		0xc3
	},
	// 125 kbps
	{	(1<<PHSEG21),					// CNF3
		(1<<BTLMODE)|(1<<PHSEG11),		// CNF2
		(1<<BRP2)|(1<<BRP1)|(1<<BRP0)	// CNF1
	},
	// 250 kbps
	{	0x03,
		0xac,
		0x81
	},
	// 500 kbps
	{	0x03,
		0xac,
		0x80
	},
	// 1 Mbps
	{	(1<<PHSEG21),
		(1<<BTLMODE)|(1<<PHSEG11),
		0
	}
};

// -------------------------------------------------------------------------
bool mcp2515_init(can_bitrate_t bitrate)
{
	if (bitrate >= 8)
		return false;
	
	SET(MCP2515_CS);
	SET_OUTPUT(MCP2515_CS);
	
	// Aktivieren der Pins fuer das SPI Interface
	RESET(P_SCK);
	RESET(P_MOSI);
	RESET(P_MISO);
	
	SET_OUTPUT(P_SCK);
	SET_OUTPUT(P_MOSI);
	SET_INPUT(P_MISO);
	
	// SPI Einstellung setzen
	mcp2515_spi_init();
	
	// MCP2515 per Software Reset zuruecksetzten,
	// danach ist er automatisch im Konfigurations Modus
	RESET(MCP2515_CS);
	spi_putc(SPI_RESET);
	
	_delay_ms(1);
	
	SET(MCP2515_CS);
	
	// ein bisschen warten bis der MCP2515 sich neu gestartet hat
	_delay_ms(10);
	
	// CNF1..3 Register laden (Bittiming)
	RESET(MCP2515_CS);
	spi_putc(SPI_WRITE);
	spi_putc(CNF3);
	for (uint8_t i=0; i<3 ;i++ ) {
		spi_putc(pgm_read_byte(&_mcp2515_cnf[bitrate][i]));
	}
	// aktivieren/deaktivieren der Interrupts
	spi_putc(MCP2515_INTERRUPTS);
	SET(MCP2515_CS);
	
	// TXnRTS Bits als Inputs schalten
	mcp2515_write_register(TXRTSCTRL, 0);
	
	#if defined(MCP2515_INT)
		SET_INPUT(MCP2515_INT);
		SET(MCP2515_INT);
	#endif
	
	#ifdef RXnBF_FUNKTION
		SET_INPUT(MCP2515_RX0BF);
		SET_INPUT(MCP2515_RX1BF);
		
		SET(MCP2515_RX0BF);
		SET(MCP2515_RX1BF);
		
		// Aktivieren der Pin-Funktionen fuer RX0BF und RX1BF
		mcp2515_write_register(BFPCTRL, (1<<B0BFE)|(1<<B1BFE)|(1<<B0BFM)|(1<<B1BFM));
	#else
		#ifdef MCP2515_TRANSCEIVER_SLEEP
			// activate the pin RX1BF as GPIO which is connected 
			// to RS of MCP2551 and set it to 0
			mcp2515_write_register(BFPCTRL, (1<<B1BFE));
		#else
			// Deaktivieren der Pins RXnBF Pins (High Impedance State)
			mcp2515_write_register(BFPCTRL, 0);
		#endif
	#endif
	
	// Testen ob das auf die beschreibenen Register zugegriffen werden kann
	// (=> ist der Chip ueberhaupt ansprechbar?)
	bool error = false;
	if (mcp2515_read_register(CNF2) != pgm_read_byte(&_mcp2515_cnf[bitrate][1])) {
		error = true;
	}
	
	// Device zurueck in den normalen Modus versetzten
	// und aktivieren/deaktivieren des Clkout-Pins
	mcp2515_write_register(CANCTRL, CLKOUT_PRESCALER_);
	
	if (error) {
		return false;
	}
	else
	{
		while ((mcp2515_read_register(CANSTAT) & 0xe0) != 0) {
			// warten bis der neue Modus uebernommen wurde
		}
		
		return true;
	}
}

#endif	// SUPPORT_FOR_MCP2515__
