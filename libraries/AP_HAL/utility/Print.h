/*
  Print.h - Base class that provides print() and println()
  Copyright (c) 2008 David A. Mellis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __AP_HAL_UTILITY_PRINT_H__
#define __AP_HAL_UTILITY_PRINT_H__

#include "../AP_HAL_Namespace.h"

#include <inttypes.h>
#include <string.h>

/**
 * This is the Arduino (v1.0) Print class, with some changes:
 * - Removed methods for class String or _FlashStringHelper
 * - printFloat takes a float, not a double. (double === float on AVR, but
 *   not on other platforms)
 */

enum {
    BASE_DEFAULT = 0,
    BASE_BIN     = 2,
    BASE_OCT     = 8,
    BASE_DEC     = 10,
    BASE_HEX     = 16
};


class AP_HAL::Print {
  private:
    size_t printNumber(unsigned long, uint8_t);
    size_t printFloat(float, uint8_t);
  public:
    Print() {}

    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;

    size_t write(const char *str) { return write((const uint8_t *)str, strlen(str)); }
  public:
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = BASE_DEC);
    size_t print(int, int = BASE_DEC);
    size_t print(unsigned int, int = BASE_DEC);
    size_t print(long, int = BASE_DEC);
    size_t print(unsigned long, int = BASE_DEC);
    size_t print(float , int = 2);
    size_t print(double , int = 2);

    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = BASE_DEC);
    size_t println(int, int = BASE_DEC);
    size_t println(unsigned int, int = BASE_DEC);
    size_t println(long, int = BASE_DEC);
    size_t println(unsigned long, int = BASE_DEC);
    size_t println(float , int = 2);
    size_t println(double , int = 2);
    size_t println(void);

};

#endif // __AP_HAL_UTILITY_PRINT_H__

