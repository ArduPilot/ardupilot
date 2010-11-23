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
        }

        // Stream extensions
        void            print_P(const char *);
        void            println_P(const char *);
        void            printf(const char *, ...);
        void            printf_P(const char *, ...);

private:
        void            _vprintf(unsigned char, const char *, va_list);
};

#endif // __BETTERSTREAM_H

