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
#include "BetterStream.h"

// Stream extensions////////////////////////////////////////////////////////////

void
BetterStream::print_P(const char *s)
{
        char    c;

        while ('\0' != (c = (const char *)s++))
        	write(c);
//		while ('\0' != (c = ((const prog_char *)s++)[0]))
//                write(c);
}

void
BetterStream::println_P(const prog_char_t *s)
{
        print_P(s);
        println();
}

void
BetterStream::printf(const char *fmt, ...)
{
        va_list ap;

        va_start(ap, fmt);

        unsigned char c;        /* holds a char from the format string */
        char buf[1024];  //destination buffer
		int i = 0;
		
		//sprintf(buf, fmt, ap);
		vsprintf(buf, fmt, ap);
		
		//per sicurezza....
		buf[1023] = 0;
		
		c = buf[i];
		while (c != 0)
		{
			/* emit cr before lf to make most terminals happy */
			if (c == '\n')
					write('\r');
			write(c);
			i++;
			c = buf[i];
		}

        va_end(ap);
}

void
BetterStream::_printf_P(const prog_char *fmt, ...)
{
        va_list ap;

        va_start(ap, fmt);

        unsigned char c;        /* holds a char from the format string */
        char buf[1024];  //destination buffer
		int i = 0;
		
		//sprintf(buf, fmt, ap);
		vsprintf(buf, fmt, ap);
		
		//per sicurezza....
		buf[1023] = 0;
		
		c = buf[i];
		while (c != 0)
		{
			/* emit cr before lf to make most terminals happy */
			if (c == '\n')
					write('\r');
			write(c);
			i++;
			c = buf[i];
		}


        va_end(ap);
}
void
BetterStream::vprintf_P(const prog_char *fmt, va_list ap)
{
    unsigned char c;        /* holds a char from the format string */
    char buf[1024];  //destination buffer
	int i = 0;

	//sprintf(buf, fmt, ap);
	vsprintf(buf, fmt, ap);

	//per sicurezza....
	buf[1023] = 0;

	c = buf[i];
	while (c != 0)
	{
		/* emit cr before lf to make most terminals happy */
		if (c == '\n')
				write('\r');
		write(c);
		i++;
		c = buf[i];
	}

}

int
BetterStream::txspace(void)
{
        // by default claim that there is always space in transmit buffer
        return(INT_MAX);
}
