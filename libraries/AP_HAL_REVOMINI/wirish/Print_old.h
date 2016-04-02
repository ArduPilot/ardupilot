/*
 * Print.h - Base class that provides print() and println()
 * Copyright (c) 2008 David A. Mellis.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 *
 * Modified 12 April 2011 by Marti Bolivar <mbolivar@leaflabs.com>
 */

#ifndef _PRINT_H_
#define _PRINT_H_

#include <stdlib.h>
#include "hal_types.h"


#define BYT 0
#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16
/*enum
{
    BYT  = 0,
    BIN  = 2,
    OCT  = 8,
    DEC  = 10,
    HEX  = 16
};
*/
class Print {
public:
    virtual void write(uint8 ch) = 0;
    virtual void write(const char *str);
    virtual void write(const void *buf, uint32 len);
    void print(char);
    void print(const char[]);
    void print(uint8, int=DEC);
    void print(int, int=DEC);
    void print(unsigned int, int=DEC);
    void print(long, int=DEC);
    void print(unsigned long, int=DEC);
    void print(long long, int=DEC);
    void print(unsigned long long, int=DEC);
    void print(double, int=2);
    void println(void);
    void println(char);
    void println(const char[]);
    void println(uint8, int=DEC);
    void println(int, int=DEC);
    void println(unsigned int, int=DEC);
    void println(long, int=DEC);
    void println(unsigned long, int=DEC);
    void println(long long, int=DEC);
    void println(unsigned long long, int=DEC);
    void println(double, int=2);
private:
    void printNumber(unsigned long long, uint8);
    void printFloat(double, uint8);
};

#endif
