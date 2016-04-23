/*
 * Print.cpp - Base class that provides print() and println()
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
 * 02110-1301 USA
 *
 * Modified 23 November 2006 by David A. Mellis
 * Modified 12 April 2011 by Marti Bolivar <mbolivar@leaflabs.com>
 */

#include "Print.h"

#include <limits.h>

#ifndef LLONG_MAX
/*
 * Note:
 *
 * At time of writing (12 April 2011), the limits.h that came with the
 * newlib we distributed didn't include LLONG_MAX.  Because we're
 * staying away from using templates (see /notes/coding_standard.rst,
 * "Language Features and Compiler Extensions"), this value was
 * copy-pasted from a println() of the value
 *
 *     std::numeric_limits<long long>::max().
 */
#define LLONG_MAX 9223372036854775807LL
#endif

//#include "wirish_math.h"

/*
 * Public methods
 */

void Print::write(const char *str) {
    while (*str) {
        write(*str++);
    }
}

void Print::write(const void *buffer, uint32 size) {
    uint8 *ch = (uint8*)buffer;
    while (size--) {
        write(*ch++);
    }
}

void Print::print(uint8 b, int base) {
    print((uint64)b, base);
}

void Print::print(char c) {
    write(c);
}

void Print::print(const char str[]) {
    write(str);
}

void Print::print(int n, int base) {
    print((long long)n, base);
}

void Print::print(unsigned int n, int base) {
    print((unsigned long long)n, base);
}

void Print::print(long n, int base) {
    print((long long)n, base);
}

void Print::print(unsigned long n, int base) {
    print((unsigned long long)n, base);
}

void Print::print(long long n, int base) {
    if (base == BYT) {
        write((uint8)n);
        return;
    }
    if (n < 0) {
        print('-');
        n = -n;
    }
    printNumber(n, base);
}

void Print::print(unsigned long long n, int base) {
    if (base == BYT) {
        write((uint8)n);
    } else {
        printNumber(n, base);
    }
}

void Print::print(double n, int digits) {
    printFloat(n, digits);
}

void Print::println(void) {
    print('\r');
    print('\n');
}

void Print::println(char c) {
    print(c);
    println();
}

void Print::println(const char c[]) {
    print(c);
    println();
}

void Print::println(uint8 b, int base) {
    print(b, base);
    println();
}

void Print::println(int n, int base) {
    print(n, base);
    println();
}

void Print::println(unsigned int n, int base) {
    print(n, base);
    println();
}

void Print::println(long n, int base) {
    print((long long)n, base);
    println();
}

void Print::println(unsigned long n, int base) {
    print((unsigned long long)n, base);
    println();
}

void Print::println(long long n, int base) {
    print(n, base);
    println();
}

void Print::println(unsigned long long n, int base) {
    print(n, base);
    println();
}

void Print::println(double n, int digits) {
    print(n, digits);
    println();
}

/*
 * Private methods
 */

void Print::printNumber(unsigned long long n, uint8 base) {
    unsigned char buf[CHAR_BIT * sizeof(long long)];
    unsigned long i = 0;

    if (n == 0) {
        print('0');
        return;
    }

    while (n > 0) {
        buf[i++] = n % base;
        n /= base;
    }

    for (; i > 0; i--) {
        print((char)(buf[i - 1] < 10 ?
                     '0' + buf[i - 1] :
                     'A' + buf[i - 1] - 10));
    }
}

/* According to snprintf(),
 *
 * nextafter((double)numeric_limits<long long>::max(), 0.0) ~= 9.22337e+18
 *
 * This slightly smaller value was picked semi-arbitrarily. */
#define LARGE_DOUBLE_TRESHOLD (9.1e18)

/* THIS FUNCTION SHOULDN'T BE USED IF YOU NEED ACCURATE RESULTS.
 *
 * This implementation is meant to be simple and not occupy too much
 * code size.  However, printing floating point values accurately is a
 * subtle task, best left to a well-tested library function.
 *
 * See Steele and White 2003 for more details:
 *
 * http://kurtstephens.com/files/p372-steele.pdf
 */
void Print::printFloat(double number, uint8 digits) {
    // Hackish fail-fast behavior for large-magnitude doubles
    if (abs(number) >= LARGE_DOUBLE_TRESHOLD) {
        if (number < 0.0) {
            print('-');
        }
        print("<large double>");
        return;
    }

    // Handle negative numbers
    if (number < 0.0) {
        print('-');
        number = -number;
    }

    // Simplistic rounding strategy so that e.g. print(1.999, 2)
    // prints as "2.00"
    double rounding = 0.5;
    for (uint8 i = 0; i < digits; i++) {
        rounding /= 10.0;
    }
    number += rounding;

    // Extract the integer part of the number and print it
    long long int_part = (long long)number;
    double remainder = number - int_part;
    print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        print(".");
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        int to_print = (int)remainder;
        print(to_print);
        remainder -= to_print;
    }
}
