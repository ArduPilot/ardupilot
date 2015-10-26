/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
//

#ifndef __AP_HAL_UTILITY_BETTERSTREAM_H__
#define __AP_HAL_UTILITY_BETTERSTREAM_H__

#include <stdarg.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Namespace.h>
/* char: */
#include <AP_Progmem/AP_Progmem.h>

#include "Stream.h"

/* AP_HAL::BetterStream is a pure virtual interface. It resembles
 * Michael Smith's BetterStream library for Arduino.
 * The Michael Smith BetterStream provided some implementations for AVR based
 * on _vprintf(). 
 * Please provide your own platform-specic implementation of vprintf, sprintf,
 * etc. to implement the printf functions.
 */

class AP_HAL::BetterStream : public AP_HAL::Stream {
public:
    BetterStream(void) {}

    virtual void print_P(const char *) = 0;
    virtual void println_P(const char *) = 0;
    virtual void printf(const char *, ...) FORMAT(2, 3) = 0;
    /* No format checking on printf_P: can't currently support that on AVR */
    virtual void _printf_P(const char *, ...) = 0;

#define printf_P(fmt, ...) _printf_P((const char *)fmt, ## __VA_ARGS__)
    
    virtual void vprintf(const char *, va_list) = 0;
    virtual void vprintf_P(const char *, va_list) = 0;
};

#endif // __AP_HAL_UTILITY_BETTERSTREAM_H__

