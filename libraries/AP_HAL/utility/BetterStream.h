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

/* AP_HAL::BetterStream is a pure virtual interface. It resembles
 * Michael Smith's BetterStream library for Arduino.
 * The Michael Smith BetterStream provided some implementations for AVR based
 * on _vprintf(). 
 * Please provide your own platform-specic implementation of vprintf, sprintf,
 * etc. to implement the printf functions.
 *
 * TODO: Segregate prog_char_t dependent functions to be available on AVR
 * platform only, with default implementations elsewhere.
 */

class AP_HAL::BetterStream : public AP_HAL::Stream {
public:
    BetterStream(void) {}

    // Stream extensions
    virtual int txspace(void) = 0;

    virtual void print_P(const prog_char_t *) = 0; 
    virtual void println_P(const prog_char_t *) = 0;
    virtual void printf(const char *, ...)
                        __attribute__ ((format(__printf__, 2, 3))) = 0;
    virtual void _printf_P(const prog_char *, ...)
                        __attribute__ ((format(__printf__, 2, 3))) = 0;

#define printf_P(fmt, ...) _printf_P((const prog_char *)fmt, ## __VA_ARGS__)

};

#endif // __AP_HAL_UTILITY_BETTERSTREAM_H__

