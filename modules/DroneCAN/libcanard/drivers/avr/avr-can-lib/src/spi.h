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
 * $Id: spi.h 6910 2008-11-30 21:13:14Z fabian $
 */
// ----------------------------------------------------------------------------

#ifndef	SPI_H
#define	SPI_H

#include "can_private.h"

// ----------------------------------------------------------------------------
// load some default values

#ifndef SPI_PRESCALER
	#define	SPI_PRESCALER			8
#endif

// ----------------------------------------------------------------------------
/**
 * \brief	Initialize SPI interface
 */
extern void mcp2515_spi_init(void);

// ----------------------------------------------------------------------------
/**
 * \brief	Write/read one byte of the SPI interface
 *
 * \param	data	Data to be written
 * \return	Data read from the slave
 */
extern uint8_t spi_putc(uint8_t data);

// ----------------------------------------------------------------------------
#ifdef USE_SOFTWARE_SPI

static uint8_t usi_interface_spi_temp;

extern __attribute__ ((gnu_inline)) inline void spi_start(uint8_t data) {
	usi_interface_spi_temp = spi_putc(data);
}

extern __attribute__ ((gnu_inline)) inline uint8_t spi_wait(void) {
	return usi_interface_spi_temp;
}

#else

extern __attribute__ ((gnu_inline)) inline void spi_start(uint8_t data) {
	SPDR = data;
}

extern __attribute__ ((gnu_inline)) inline uint8_t spi_wait(void) {
	// warten bis der vorherige Werte geschrieben wurde
	while(!(SPSR & (1<<SPIF)))
		;
	
	return SPDR;
}

#endif

#endif	// SPI_H
