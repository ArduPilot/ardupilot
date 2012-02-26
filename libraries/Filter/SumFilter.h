// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	SumFilter.h
/// @brief	A class to apply an average filter (but we save some calc time by not averaging the values but instead save one division by just adding the values up
///
///         DO NOT CREATE AND DESTROY INSTANCES OF THIS CLASS BECAUSE THE ALLOC/MALLOC WILL LEAD TO MEMORY FRAGMENTATION

#ifndef SumFilter_h
#define SumFilter_h

#include <inttypes.h>
#include <Filter.h>

template <class T>
class SumFilter : public Filter<T>
{
  public:
	SumFilter(uint8_t requested_size) : Filter<T>(requested_size) {};

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual T apply(T sample);

  private:
};

// Typedef for convenience
typedef SumFilter<int16_t> SumFilterInt16;

// Public Methods //////////////////////////////////////////////////////////////
template <class T>
T SumFilter<T>::apply(T sample){

	T result = 0;

	// call parent's apply function to get the sample into the array
	Filter<T>::apply(sample);

	// get sum of all values - there is a risk of overflow here that we ignore
	for(int8_t i=0; i<Filter<T>::filter_size; i++)
		result += Filter<T>::samples[i];

	return result;
}

#endif



