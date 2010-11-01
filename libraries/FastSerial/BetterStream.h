// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

#ifndef __BETTERSTREAM_H
#define __BETTERSTREAM_H

#include <Stream.h>
#include <avr/pgmspace.h>

class BetterStream : public Stream {
public:
        BetterStream(void) {
                // init stdio
                fdev_setup_stream(&fd, &BetterStream::_putchar, &BetterStream::_getchar, _FDEV_SETUP_RW);
                fdev_set_udata(&fd, this);
        }

        // Stream extensions
        void            print_P(const char *s);
        void            println_P(const char *s);

        // stdio extensions
        int             printf(const char *fmt, ...);
        int             printf_P(const char *fmt, ...);

protected:
        // subclasses can use this to e.g. set up stdin/stdout/stderr.
        FILE            fd;

private:
        // stdio emulation
        static int      _putchar(char c, FILE *stream);
        static int      _getchar(FILE *stream);
};

#endif // __BETTERSTREAM_H

