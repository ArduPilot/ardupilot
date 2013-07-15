// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	FilterClass.h
/// @brief	A pure virtual interface class
///

#ifndef __FILTER_CLASS_H__
#define __FILTER_CLASS_H__

#include <inttypes.h>

template <class T>
class Filter
{
public:
    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample) = 0;

    // reset - clear the filter
    virtual void reset()  = 0;

};

// Typedef for convenience
typedef Filter<int8_t> FilterInt8;
typedef Filter<uint8_t> FilterUInt8;
typedef Filter<int16_t> FilterInt16;
typedef Filter<uint16_t> FilterUInt16;
typedef Filter<int32_t> FilterInt32;
typedef Filter<uint32_t> FilterUInt32;

#endif // __FILTER_CLASS_H__

