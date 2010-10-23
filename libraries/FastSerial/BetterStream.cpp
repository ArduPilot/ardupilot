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

#include "BetterStream.h"

// Stream extensions////////////////////////////////////////////////////////////

void
BetterStream::print_P(const prog_char *s)
{
        char    c;

        while ('\0' != (c = pgm_read_byte(s++)))
                write(c);
}

void
BetterStream::println_P(const char *s)
{
        print_P(s);
        println();
}

// STDIO emulation /////////////////////////////////////////////////////////////

int
BetterStream::_putchar(char c, FILE *stream)
{
        BetterStream      *bs;

        bs = (BetterStream *)fdev_get_udata(stream);
        if ('\n' == c)
                bs->write('\r');        // ASCII translation on the cheap
        bs->write(c);
        return(0);
}

int
BetterStream::_getchar(FILE *stream)
{
        BetterStream      *bs;

        bs = (BetterStream *)fdev_get_udata(stream);

        // We return -1 if there is nothing to read, which the library interprets
        // as an error, which our clients will need to deal with.
        return(bs->read());
}

int
BetterStream::printf(const char *fmt, ...)
{
        va_list ap;
        int     i;

        va_start(ap, fmt);
        i = vfprintf(&fd, fmt, ap);
        va_end(ap);

        return(i);
}

int
BetterStream::printf_P(const char *fmt, ...)
{
        va_list ap;
        int     i;

        va_start(ap, fmt);
        i = vfprintf_P(&fd, fmt, ap);
        va_end(ap);

        return(i);
}

