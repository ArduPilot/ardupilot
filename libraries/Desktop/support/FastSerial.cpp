// -*-  tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Interrupt-driven serial transmit/receive library.
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// Receive and baudrate calculations derived from the Arduino
// HardwareSerial driver:
//
//      Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
// Transmit algorithm inspired by work:
//
//      Code Jose Julio and Jordi Munoz. DIYDrones.com
//
//      This library is free software; you can redistribute it and/or
//      modify it under the terms of the GNU Lesser General Public
//      License as published by the Free Software Foundation; either
//      version 2.1 of the License, or (at your option) any later version.
//
//      This library is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//      Lesser General Public License for more details.
//
//      You should have received a copy of the GNU Lesser General Public
//      License along with this library; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//


//#include "../AP_Common/AP_Common.h"
#include "FastSerial.h"
#include "WProgram.h"
#include <unistd.h>

#if   defined(UDR3)
# define FS_MAX_PORTS   4
#elif defined(UDR2)
# define FS_MAX_PORTS   3
#elif defined(UDR1)
# define FS_MAX_PORTS   2
#else
# define FS_MAX_PORTS   1
#endif

FastSerial::Buffer __FastSerial__rxBuffer[FS_MAX_PORTS];
FastSerial::Buffer __FastSerial__txBuffer[FS_MAX_PORTS];

// Constructor /////////////////////////////////////////////////////////////////

FastSerial::FastSerial(const uint8_t portNumber, volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
					   volatile uint8_t *ucsra, volatile uint8_t *ucsrb, const uint8_t u2x,
					   const uint8_t portEnableBits, const uint8_t portTxBits) :
					   _ubrrh(ubrrh),
					   _ubrrl(ubrrl),
					   _ucsra(ucsra),
					   _ucsrb(ucsrb),
					   _u2x(u2x),
					   _portEnableBits(portEnableBits),
					   _portTxBits(portTxBits),
					   _rxBuffer(&__FastSerial__rxBuffer[portNumber]),
					   _txBuffer(&__FastSerial__txBuffer[portNumber])
{
}

// Public Methods //////////////////////////////////////////////////////////////

void FastSerial::begin(long baud)
{
}

void FastSerial::begin(long baud, unsigned int rxSpace, unsigned int txSpace)
{
}

void FastSerial::end()
{
}

int FastSerial::available(void)
{
	return 0;
}

int FastSerial::txspace(void)
{
	return 128;
}

int FastSerial::read(void)
{
	char c;
	pread(0, (void *)&c, 1, 0);
	return (int)c;
}

int FastSerial::peek(void)
{
	return -1;
}

void FastSerial::flush(void)
{
}

void FastSerial::write(uint8_t c)
{
	fwrite(&c, 1, 1, stdout);
}

// Buffer management ///////////////////////////////////////////////////////////

bool FastSerial::_allocBuffer(Buffer *buffer, unsigned int size)
{
	return false;
}

void FastSerial::_freeBuffer(Buffer *buffer)
{
}

