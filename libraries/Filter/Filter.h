// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	Filter.h
/// @brief	A base class from which various filters classes should inherit
///

#ifndef Filter_h
#define Filter_h

#include <inttypes.h>
#include <AP_Common.h>

template <class T>
class Filter
{
  public:
	// constructor
	Filter() {};

	// apply - Add a new raw value to the filter, retrieve the filtered result
	virtual T apply(T sample) { return sample; };
	
	// reset - clear the filter
	virtual void reset() {};

};

// Typedef for convenience
typedef Filter<int8_t> FilterInt8;
typedef Filter<uint8_t> FilterUInt8;
typedef Filter<int16_t> FilterInt16;
typedef Filter<uint16_t> FilterUInt16;
typedef Filter<int32_t> FilterInt32;
typedef Filter<uint32_t> FilterUInt32;

#endif



