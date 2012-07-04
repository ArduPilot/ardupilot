// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	Derivative.cpp
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/

#include <FastSerial.h>
#include <inttypes.h>
#include <Filter.h>
#include <DerivativeFilter.h>

template <class T, class U, uint8_t FILTER_SIZE>
float DerivativeFilter<T,U,FILTER_SIZE>::apply(T sample)
{
	float result = 0;
    uint32_t tnow = millis();
    float deltat = (tnow - _last_time) * 1.0e-3;
    _last_time = tnow;
	// call parent's apply function to get the sample into the array
	FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

	// increment the number of samples so far
	_num_samples++;
	if( _num_samples > FILTER_SIZE || _num_samples == 0 )
		_num_samples = FILTER_SIZE;

    // use f() to make the code match the maths a bit better. Note
    // that unlike an average filter, we care about the order of the elements
    #define f(i) FilterWithBuffer<T,FILTER_SIZE>::samples[(((FilterWithBuffer<T,FILTER_SIZE>::sample_index-1)-i)+FILTER_SIZE) % FILTER_SIZE]

    // N in the paper is _num_samples-1
    switch (FILTER_SIZE-1) {
    case 1:
        result = f(0) - f(1);
        break;
    case 2:
        result = f(0) - f(2);
        break;
    case 3:
        result = f(0) + f(1) - f(2) - f(3);
        break;
    case 4:
        result = f(0) + 2*f(1) - 2*f(3) - f(4);
        break;
    case 5:
        result = f(0) + 3*f(1) + 2*f(2) - 2*f(3) - 3*f(4) - f(5);
        break;
    case 6:
        result = f(0) + 4*f(1) + 5*f(2) - 5*f(4) - 4*f(5) - f(6);
        break;
    case 7:
        result = f(0) + 5*f(1) + 9*f(2) + 5*f(3) - 5*f(4) - 9*f(5) - 5*f(6) - f(7);
        break;
    case 8:
        result = f(0) + 6*f(1) + 14*f(2) + 14*f(3) - 14*f(5) - 14*f(6) - 6*f(7) - f(8);
        break;
    case 9:
        result = f(0) + 7*f(1) + 20*f(2) + 28*f(3) + 14*f(4) - 14*f(5) - 28*f(6) - 20*f(7) - 7*f(8) - f(9);
        break;
    default:
        result = 0;
        break;
    }

    result /= deltat * (1<<(uint16_t)(FILTER_SIZE-2));

    return result;
}

// reset - clear all samples
template <class T, class U, uint8_t FILTER_SIZE>
void DerivativeFilter<T,U,FILTER_SIZE>::reset(void)
{
	// call parent's apply function to get the sample into the array
	FilterWithBuffer<T,FILTER_SIZE>::reset();

	// clear our variable
	_num_samples = 0;
}

// add new instances as needed here
template float DerivativeFilter<float,float,9>::apply(float sample);
template void DerivativeFilter<float,float,9>::reset(void);
