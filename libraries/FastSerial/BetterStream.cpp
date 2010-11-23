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

void
BetterStream::printf(const char *fmt, ...)
{
        va_list ap;

        va_start(ap, fmt);
        _vprintf(0, fmt, ap);
        va_end(ap);
}

void
BetterStream::printf_P(const char *fmt, ...)
{
        va_list ap;

        va_start(ap, fmt);
        _vprintf(1, fmt, ap);
        va_end(ap);
}

