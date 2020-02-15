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
#pragma once

#include <stdarg.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Namespace.h>

class AP_HAL::BetterStream {
public:

    virtual void printf(const char *, ...) FMT_PRINTF(2, 3);
    virtual void vprintf(const char *, va_list);

    void print(const char *str) { write(str); }
    void println(const char *str) { printf("%s\r\n", str); }

    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    size_t write(const char *str);

    virtual uint32_t available() = 0;

    /* return value for read():
     * -1 if nothing available, uint8_t value otherwise. */
    virtual int16_t read() = 0;

    /* NB txspace was traditionally a member of BetterStream in the
     * FastSerial library. As far as concerns go, it belongs with available() */
    virtual uint32_t txspace() = 0;
};
