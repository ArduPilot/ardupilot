/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish_math.h
 * @brief Includes <math.h>; provides Arduino-compatible math routines.
 */

#ifndef _WIRING_MATH_H_
#define _WIRING_MATH_H_

#include <arm_math.h>

/**
 * @brief Initialize the pseudo-random number generator.
 * @param seed the number used to initialize the seed; cannot be zero.
 */
void randomSeed(unsigned int seed);

/**
 * @brief Generate a pseudo-random number with upper bound.
 * @param max An upper bound on the returned value, exclusive.
 * @return A pseudo-random number in the range [0,max).
 * @see randomSeed()
 */
long random(long max);

/**
 * @brief Generate a pseudo-random number with lower and upper bounds.
 * @param min Lower bound on the returned value, inclusive.
 * @param max Upper bound on the returned value, exclusive.
 * @return A pseudo-random number in the range [min, max).
 * @see randomSeed()
 */
long random(long min, long max);

/**
 * @brief Remap a number from one range to another.
 *
 * That is, a value equal to fromStart gets mapped to toStart, a value
 * of fromEnd to toEnd, and other values are mapped proportionately.
 *
 * Does not constrain value to lie within [fromStart, fromEnd].
 *
 * If a "start" value is larger than its corresponding "end", the
 * ranges are reversed, so map(n, 1, 10, 10, 1) would reverse the
 * range [1,10].
 *
 * Negative numbers may appear as any argument.
 *
 * @param value the value to map.
 * @param fromStart the beginning of the value's current range.
 * @param fromEnd the end of the value's current range.
 * @param toStart the beginning of the value's mapped range.
 * @param toEnd the end of the value's mapped range.
 * @return the mapped value.
 */
static inline long map(long value, long fromStart, long fromEnd,
                long toStart, long toEnd) {
    return (value - fromStart) * (toEnd - toStart) / (fromEnd - fromStart) +
        toStart;
}
/*
//#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
//#define DEG_TO_RAD  0.017453292519943295769236907684886
//#define RAD_TO_DEG 57.295779513082320876798154814105

#define min(a,b)                ((a)<(b)?(a):(b))
#define max(a,b)                ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)                ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
//#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)
#define sq(x)                   ((x)*(x))
*/
/* undefine stdlib's abs if encountered */
//#ifdef abs
//  #undef abs
//#endif
//#define abs(x) (((x) > 0) ? (x) : -(x))

/* Following are duplicate declarations (with Doxygen comments) for
 * some of the math.h functions; this is for the convenience of the
 * Sphinx docs.
 */

/**
 * Compute the cosine of an angle, in radians.
 * @param x The radian measure of the angle.
 * @return The cosine of x.  This value will be between -1 and 1.
 */
double cos(double x);

/**
 * Compute the sine of an angle, in radians.
 * @param x The radian measure of the angle.
 * @return The sine of x.  This value will be between -1 and 1.
 */
double sin(double x);

/**
 * Compute the tangent of an angle, in radians.
 * @param x The radian measure of the angle.
 * @return The tangent of x.  There are no limits on the return value
 * of this function.
 */
double tan(double x);

/**
 * Compute the square root of a number.
 * @param x The number whose square root to find.  This value cannot
 * be negative.
 * @return The square root of x.  The return value is never negative.
 */
double sqrt(double x);

/**
 * Compute an exponentiation.
 * @param x the base. This value cannot be zero if y <= 0.  This value
 * cannot be negative if y is not an integral value.
 * @param y the exponent.
 * @return x raised to the power y.
 */
double pow(double x, double y);

#endif
