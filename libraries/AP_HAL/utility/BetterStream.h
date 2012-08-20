//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

#ifndef __AP_HAL_UTILITY_BETTERSTREAM_H__
#define __AP_HAL_UTILITY_BETTERSTREAM_H__

#include "../AP_HAL_Namespace.h"
#include "Stream.h"

/* prog_char_t: */
#include <AP_Common.h>
#include <avr/pgmspace.h>

/* AP_HAL::BetterStream is derived from Michael Smith's FastSerial library for
 * Arduino. Mike's library has an implementation of _vprintf() for AVR.
 * Please provide your own platform-specic implementation for this library.
 *
 * TODO: Segregate prog_char_t dependent functions to be available on AVR
 * platform only, with default implementations elsewhere.
 */

class AP_HAL::BetterStream : public AP_HAL::Stream {
public:
    BetterStream(void) {}

    // Stream extensions
    virtual void print_P(const prog_char_t *);
    virtual void println_P(const prog_char_t *);
    virtual void printf(const char *, ...)
                        __attribute__ ((format(__printf__, 2, 3)));
    virtual void _printf_P(const prog_char *, ...)
                        __attribute__ ((format(__printf__, 2, 3)));

    virtual int txspace(void);

#define printf_P(fmt, ...) _printf_P((const prog_char *)fmt, ## __VA_ARGS__)

private:
    virtual void _vprintf(unsigned char, const char *, va_list)
                __attribute__ ((format(__printf__, 3, 0)));


};

#endif // __AP_HAL_UTILITY_BETTERSTREAM_H__

