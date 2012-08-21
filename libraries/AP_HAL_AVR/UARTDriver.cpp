// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

//
// Enhancements to the Arduino Stream class.
//

#include <limits.h>
#include <AP_Common.h>
#include <avr/pgmspace.h>
#include <AP_HAL.h>

#include "UARTDriver.h"
using namespace AP_HAL_AVR;

#define FS_MAX_PORTS 4
AVRUARTDriver::Buffer __AVRUARTDriver__rxBuffer[FS_MAX_PORTS];
AVRUARTDriver::Buffer __AVRUARTDriver__txBuffer[FS_MAX_PORTS];

AVRUARTDriver::AVRUARTDriver(
        const uint8_t portNumber, volatile uint8_t *ubrrh,
        volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
        volatile uint8_t *ucsrb, const uint8_t u2x,
		const uint8_t portEnableBits, const uint8_t portTxBits) :
			_ubrrh(ubrrh),
			_ubrrl(ubrrl),
			_ucsra(ucsra),
			_ucsrb(ucsrb),
			_u2x(u2x),
			_portEnableBits(portEnableBits),
			_portTxBits(portTxBits),
			_rxBuffer(&__AVRUARTDriver__rxBuffer[portNumber]),
			_txBuffer(&__AVRUARTDriver__txBuffer[portNumber])
{
	_initialized = true;
	begin(57600);
}
/* */

/* BetterStream implementations */
void AVRUARTDriver::print_P(const prog_char_t *s)
{
        char    c;

        while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
                write(c);
}

void AVRUARTDriver::println_P(const prog_char_t *s)
{
        print_P(s);
        println();
}

void AVRUARTDriver::printf(const char *fmt, ...)
{
        va_list ap;

        va_start(ap, fmt);
        _vprintf(0, fmt, ap);
        va_end(ap);
}

void AVRUARTDriver::_printf_P(const prog_char *fmt, ...)
{
        va_list ap;

        va_start(ap, fmt);
        _vprintf(1, fmt, ap);
        va_end(ap);
}
