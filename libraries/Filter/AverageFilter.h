// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	AverageFilter.h
/// @brief	A class to provide the average of a number of samples
///
///         DO NOT CREATE AND DESTROY INSTANCES OF THIS CLASS BECAUSE THE ALLOC/MALLOC WILL LEAD TO MEMORY FRAGMENTATION

#ifndef AverageFilter_h
#define AverageFilter_h

#include <inttypes.h>
#include <Filter.h>

// 1st parameter <T> is the type of data being filtered.  
// 2nd parameter <U> is a larger data type used during summation to prevent overflows
template <class T, class U>
class AverageFilter : public Filter<T>
{
  public:
	AverageFilter(uint8_t requested_size) : Filter<T>(requested_size) {};

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual T apply(T sample);

	// reset - clear the filter
	virtual void reset();

  private:
	uint8_t	_num_samples;
};

// Typedef for convenience (1st argument is the data size, 2nd argument is a datasize that's bigger to handle overflows)
typedef AverageFilter<int8_t, int16_t> AverageFilterInt8;
typedef AverageFilter<uint8_t, uint16_t> AverageFilterUInt8;
typedef AverageFilter<int16_t, int32_t> AverageFilterInt16;
typedef AverageFilter<uint16_t, uint32_t> AverageFilterUInt16;
typedef AverageFilter<int32_t, float> AverageFilterInt32;
typedef AverageFilter<uint32_t, float> AverageFilterUInt32;

// Public Methods //////////////////////////////////////////////////////////////

template <class T, class U>
T AverageFilter<T,U>::apply(T sample)
{
	U result = 0;

	// call parent's apply function to get the sample into the array
	Filter<T>::apply(sample);

	// increment the number of samples so far
	_num_samples++;
	if( _num_samples > Filter<T>::filter_size || _num_samples == 0 )
		_num_samples = Filter<T>::filter_size;

	// get sum of all values - there is a risk of overflow here that we ignore
	for(int8_t i=0; i<Filter<T>::filter_size; i++)
		result += Filter<T>::samples[i];

	return (T)(result / _num_samples);
}

// reset - clear all samples
template <class T, class U>
void AverageFilter<T,U>::reset()
{
	// call parent's apply function to get the sample into the array
	Filter<T>::reset();

	// clear our variable
	_num_samples = 0;
}

#endif



