// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	ModeFilter.h
/// @brief	A class to apply a mode filter which is basically picking the median value from the last x samples
///         the filter size (i.e buffer size) should always be an odd number
///
///         DO NOT CREATE AND DESTROY INSTANCES OF THIS CLASS BECAUSE THE ALLOC/MALLOC WILL LEAD TO MEMORY FRAGMENTATION

#ifndef ModeFilter_h
#define ModeFilter_h

#include <inttypes.h>
#include <Filter.h>

template <class T>
class ModeFilter : public Filter<T>
{
  public:
	ModeFilter(uint8_t requested_size, uint8_t return_element);

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual T	apply(T sample);

  private:
    // private methods
	uint8_t	_return_element;
  	void 	isort(T sample, bool drop_high_sample);
	bool	drop_high_sample;  // switch to determine whether to drop the highest or lowest sample when new value arrives
};

// Typedef for convenience
typedef ModeFilter<int16_t> ModeFilterInt16;

// Constructor    //////////////////////////////////////////////////////////////

template <class T>
ModeFilter<T>::ModeFilter(uint8_t requested_size, uint8_t return_element) :
	Filter<T>(requested_size),
	_return_element(return_element),
	drop_high_sample(true)
{
	// ensure we have a valid return_nth_element value.  if not, revert to median
	if( _return_element >= Filter<T>::filter_size )
		_return_element = Filter<T>::filter_size / 2;
};

// Public Methods //////////////////////////////////////////////////////////////

template <class T>
T ModeFilter<T>::apply(T sample)
{
	// insert the new items into the samples buffer
	isort(sample, drop_high_sample);
	
	// next time drop from the other end of the sample buffer
	drop_high_sample = !drop_high_sample;

	// return results
	if( Filter<T>::sample_index < Filter<T>::filter_size ) {
		// middle sample if buffer is not yet full
		return Filter<T>::samples[(Filter<T>::sample_index / 2)];
	}else{
		// return element specified by user in constructor
		return Filter<T>::samples[_return_element];
	}
}

//
// insertion sort - takes a new sample and pushes it into the sample array
//                  drops either the highest or lowest sample depending on the 'drop_high_sample' parameter
//
template <class T>
void ModeFilter<T>::isort(T new_sample, bool drop_high)
{
	int8_t i;

	// if the buffer isn't full simply increase the #items in the buffer (i.e. sample_index)
	// the rest is the same as dropping the high sample
	if( Filter<T>::sample_index < Filter<T>::filter_size ) {
		Filter<T>::sample_index++;
		drop_high = true;
	}

	if( drop_high ) {  // drop highest sample from the buffer to make room for our new sample

		// start from top. Note: sample_index always points to the next open space so we start from sample_index-1
		i = Filter<T>::sample_index-1;

		// if the next element is higher than our new sample, push it up one position
		while( Filter<T>::samples[i-1] > new_sample && i > 0 ) {
			Filter<T>::samples[i] = Filter<T>::samples[i-1];
			i--;
		}

		// add our new sample to the buffer
		Filter<T>::samples[i] = new_sample;
		
	}else{ // drop lowest sample from the buffer to make room for our new sample

		// start from the bottom
		i = 0;

		// if the element is lower than our new sample, push it down one position
		while( Filter<T>::samples[i+1] < new_sample && i < Filter<T>::sample_index-1 ) {
			Filter<T>::samples[i] = Filter<T>::samples[i+1];
			i++;
		}

		// add our new sample to the buffer
		Filter<T>::samples[i] = new_sample;
	}
}

#endif



