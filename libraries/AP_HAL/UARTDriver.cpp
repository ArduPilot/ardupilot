// -*- Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL.h>

#include "utility/print_vprintf.h"
#include "UARTDriver.h"

/* 
   BetterStream method implementations
   These are implemented in AP_HAL to ensure consistent behaviour on
   all boards, although they can be overridden by a port
 */

void AP_HAL::UARTDriver::print_P(const prog_char_t *s) 
{
    char    c;
    while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
        write(c);
}

void AP_HAL::UARTDriver::println_P(const prog_char_t *s) 
{
    print_P(s);
    println();
}

void AP_HAL::UARTDriver::printf(const char *fmt, ...) 
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void AP_HAL::UARTDriver::vprintf(const char *fmt, va_list ap) 
{
    print_vprintf((AP_HAL::Print*)this, 0, fmt, ap);
}

void AP_HAL::UARTDriver::_printf_P(const prog_char *fmt, ...) 
{
    va_list ap;
    va_start(ap, fmt);
    vprintf_P(fmt, ap);
    va_end(ap);
}

void AP_HAL::UARTDriver::vprintf_P(const prog_char *fmt, va_list ap) 
{
    print_vprintf((AP_HAL::Print*)this, 1, fmt, ap);
}
