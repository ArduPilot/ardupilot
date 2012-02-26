// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	Filter.h
/// @brief	A base class for apply various filters to values
///
///         DO NOT CREATE AND DESTROY INSTANCES OF THIS CLASS BECAUSE THE ALLOC/MALLOC WILL LEAD TO MEMORY FRAGMENTATION

#ifndef Filter_h
#define Filter_h

#include <inttypes.h>
#include <AP_Common.h>

#define FILTER_MAX_SAMPLES 10  // maximum size of the sample buffer (normally older values will be overwritten as new appear)

template <class T>
class Filter
{
  public:
	Filter(uint8_t requested_size);
	~Filter();

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual T apply(T sample);
	
	// reset - clear the filter
	virtual void reset();
	
	uint8_t	filter_size;	// max number of items in filter
	T* 		samples;		// buffer of samples
	uint8_t	sample_index;	// pointer to the next empty slot in the buffer

  private:
};

// Typedef for convenience
typedef Filter<int16_t> FilterInt16;

// Constructor 
template <class T>
Filter<T>::Filter(uint8_t requested_size) :
	filter_size(requested_size), sample_index(0)
{
	// check filter size
    if( Filter<T>::filter_size > FILTER_MAX_SAMPLES )
		Filter<T>::filter_size = FILTER_MAX_SAMPLES;

	// create array
	samples = (T *)malloc(Filter<T>::filter_size * sizeof(T));

	// clear array
	reset();
}

// Destructor - THIS SHOULD NEVER BE CALLED OR IT COULD LEAD TO MEMORY FRAGMENTATION
template <class T>
Filter<T>::~Filter()
{
	// free up the samples array
	free(samples);
}
		
// reset - clear all samples
template <class T>
void Filter<T>::reset()
{
    for( int8_t i=0; i<filter_size; i++ ) {
	    samples[i] = 0;
	}
	sample_index = 0;
}

// apply - take in a new raw sample, and return the filtered results
template <class T>
T Filter<T>::apply(T sample){

	// add sample to array
	samples[sample_index++] = sample;

	// wrap index if necessary
	if( sample_index >= filter_size )
		sample_index = 0;

	// base class doesn't know what filtering to do so we just return the raw sample
	return sample;
}

#endif



