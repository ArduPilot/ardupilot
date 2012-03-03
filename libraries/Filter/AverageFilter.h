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
#include <FilterWithBuffer.h>

// 1st parameter <T> is the type of data being filtered.  
// 2nd parameter <U> is a larger data type used during summation to prevent overflows
template <class T, class U, uint8_t FILTER_SIZE>
class AverageFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
  public:
	// constructor
	AverageFilter() : FilterWithBuffer<T,FILTER_SIZE>(), _num_samples(0) {};

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual T apply(T sample);

	// reset - clear the filter
	virtual void reset();

  private:
	uint8_t	_num_samples;	// the number of samples in the filter, maxes out at size of the filter
};

// Typedef for convenience (1st argument is the data type, 2nd is a larger datatype to handle overflows, 3rd is buffer size)
typedef AverageFilter<int8_t,int16_t,2> AverageFilterInt8_Size2;
typedef AverageFilter<int8_t,int16_t,3> AverageFilterInt8_Size3;
typedef AverageFilter<int8_t,int16_t,4> AverageFilterInt8_Size4;
typedef AverageFilter<int8_t,int16_t,5> AverageFilterInt8_Size5;
typedef AverageFilter<uint8_t,uint16_t,2> AverageFilterUInt8_Size2;
typedef AverageFilter<uint8_t,uint16_t,3> AverageFilterUInt8_Size3;
typedef AverageFilter<uint8_t,uint16_t,4> AverageFilterUInt8_Size4;
typedef AverageFilter<uint8_t,uint16_t,5> AverageFilterUInt8_Size5;

typedef AverageFilter<int16_t,int32_t,2> AverageFilterInt16_Size2;
typedef AverageFilter<int16_t,int32_t,3> AverageFilterInt16_Size3;
typedef AverageFilter<int16_t,int32_t,4> AverageFilterInt16_Size4;
typedef AverageFilter<int16_t,int32_t,5> AverageFilterInt16_Size5;
typedef AverageFilter<uint16_t,uint32_t,2> AverageFilterUInt16_Size2;
typedef AverageFilter<uint16_t,uint32_t,3> AverageFilterUInt16_Size3;
typedef AverageFilter<uint16_t,uint32_t,4> AverageFilterUInt16_Size4;
typedef AverageFilter<uint16_t,uint32_t,5> AverageFilterUInt16_Size5;

typedef AverageFilter<int32_t,float,2> AverageFilterInt32_Size2;
typedef AverageFilter<int32_t,float,3> AverageFilterInt32_Size3;
typedef AverageFilter<int32_t,float,4> AverageFilterInt32_Size4;
typedef AverageFilter<int32_t,float,5> AverageFilterInt32_Size5;
typedef AverageFilter<uint32_t,float,2> AverageFilterUInt32_Size2;
typedef AverageFilter<uint32_t,float,3> AverageFilterUInt32_Size3;
typedef AverageFilter<uint32_t,float,4> AverageFilterUInt32_Size4;
typedef AverageFilter<uint32_t,float,5> AverageFilterUInt32_Size5;

// Public Methods //////////////////////////////////////////////////////////////

template <class T, class U, uint8_t FILTER_SIZE>
T AverageFilter<T,U,FILTER_SIZE>::apply(T sample)
{
	U result = 0;

	// call parent's apply function to get the sample into the array
	FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

	// increment the number of samples so far
	_num_samples++;
	if( _num_samples > FILTER_SIZE || _num_samples == 0 )
		_num_samples = FILTER_SIZE;

	// get sum of all values - there is a risk of overflow here that we ignore
	for(uint8_t i=0; i<FILTER_SIZE; i++)
		result += FilterWithBuffer<T,FILTER_SIZE>::samples[i];

	return (T)(result / _num_samples);
}

// reset - clear all samples
template <class T, class U, uint8_t FILTER_SIZE>
void AverageFilter<T,U,FILTER_SIZE>::reset()
{
	// call parent's apply function to get the sample into the array
	FilterWithBuffer<T,FILTER_SIZE>::reset();

	// clear our variable
	_num_samples = 0;
}

#endif