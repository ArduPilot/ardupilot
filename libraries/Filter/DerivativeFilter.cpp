// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/// @file	Derivative.cpp
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
//
#include <inttypes.h>
#include <AP_Math/AP_Math.h>
#include "Filter.h"
#include "DerivativeFilter.h"


template <class T,  uint8_t FILTER_SIZE>
void DerivativeFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp)
{
    uint8_t i = FilterWithBuffer<T,FILTER_SIZE>::sample_index;
    uint8_t i1;
    if (i == 0) {
        i1 = FILTER_SIZE-1;
    } else {
        i1 = i-1;
    }
    if (_timestamps[i1] == timestamp) {
        // this is not a new timestamp - ignore
        return;
    }

    // add timestamp before we apply to FilterWithBuffer
    _timestamps[i] = timestamp;

    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

    _new_data = true;
}


template <class T,  uint8_t FILTER_SIZE>
float DerivativeFilter<T,FILTER_SIZE>::slope(void)
{
    if (!_new_data) {
        return _last_slope;
    }

    float result = 0;

    // use f() to make the code match the maths a bit better. Note
    // that unlike an average filter, we care about the order of the elements
#define f(i) FilterWithBuffer<T,FILTER_SIZE>::samples[(((FilterWithBuffer<T,FILTER_SIZE>::sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]
#define x(i) _timestamps[(((FilterWithBuffer<T,FILTER_SIZE>::sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]

    if (_timestamps[FILTER_SIZE-1] == _timestamps[FILTER_SIZE-2]) {
        // we haven't filled the buffer yet - assume zero derivative
        return 0;
    }

    // N in the paper is FILTER_SIZE
    switch (FILTER_SIZE) {
    case 5:
        result = 2*2*(f(1) - f(-1)) / (x(1) - x(-1))
                 + 4*1*(f(2) - f(-2)) / (x(2) - x(-2));
        result /= 8;
        break;
    case 7:
        result = 2*5*(f(1) - f(-1)) / (x(1) - x(-1))
                 + 4*4*(f(2) - f(-2)) / (x(2) - x(-2))
                 + 6*1*(f(3) - f(-3)) / (x(3) - x(-3));
        result /= 32;
        break;
    case 9:
        result = 2*14*(f(1) - f(-1)) / (x(1) - x(-1))
                 + 4*14*(f(2) - f(-2)) / (x(2) - x(-2))
                 + 6* 6*(f(3) - f(-3)) / (x(3) - x(-3))
                 + 8* 1*(f(4) - f(-4)) / (x(4) - x(-4));
        result /= 128;
        break;
    case 11:
        result =  2*42*(f(1) - f(-1)) / (x(1) - x(-1))
                 +  4*48*(f(2) - f(-2)) / (x(2) - x(-2))
                 +  6*27*(f(3) - f(-3)) / (x(3) - x(-3))
                 +  8* 8*(f(4) - f(-4)) / (x(4) - x(-4))
                 + 10* 1*(f(5) - f(-5)) / (x(5) - x(-5));
        result /= 512;
        break;
    default:
        result = 0;
        break;
    }

    // cope with numerical errors
    if (isnan(result) || isinf(result)) {
        result = 0;
    }

    _new_data = false;
    _last_slope = result;

    return result;
}

// reset - clear all samples
template <class T, uint8_t FILTER_SIZE>
void DerivativeFilter<T,FILTER_SIZE>::reset(void)
{
    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::reset();
}

// add new instances as needed here
template void DerivativeFilter<float,5>::update(float sample, uint32_t timestamp);
template float DerivativeFilter<float,5>::slope(void);
template void DerivativeFilter<float,5>::reset(void);

template void DerivativeFilter<float,7>::update(float sample, uint32_t timestamp);
template float DerivativeFilter<float,7>::slope(void);
template void DerivativeFilter<float,7>::reset(void);

template void DerivativeFilter<float,9>::update(float sample, uint32_t timestamp);
template float DerivativeFilter<float,9>::slope(void);
template void DerivativeFilter<float,9>::reset(void);

template void DerivativeFilter<float,11>::update(float sample, uint32_t timestamp);
template float DerivativeFilter<float,11>::slope(void);
template void DerivativeFilter<float,11>::reset(void);



