// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	Derivative.h
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/

#ifndef Derivative_h
#define Derivative_h

#include <inttypes.h>
#include <Filter.h>
#include <FilterWithBuffer.h>

// 1st parameter <T> is the type of data being filtered.  
// 2nd parameter <U> is a larger data type used during summation to prevent overflows
// 3rd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, class U, uint8_t FILTER_SIZE>
class DerivativeFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
  public:
	// constructor
	DerivativeFilter() : FilterWithBuffer<T,FILTER_SIZE>(), _num_samples(0) {};

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual float apply(T sample);

	// reset - clear the filter
	virtual void reset();

  private:
    uint32_t _last_time;
	uint8_t	_num_samples;	// the number of samples in the filter, maxes out at size of the filter
};

typedef DerivativeFilter<float,float,9> DerivativeFilterFloat_Size9;


#endif // Derivative_h

