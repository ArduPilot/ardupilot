// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

#ifndef LowPassFilter_h
#define LowPassFilter_h

#include <inttypes.h>
#include <Filter.h>

// 1st parameter <T> is the type of data being filtered.
template <class T>
class LowPassFilter : public Filter<T>
{
public:
    // constructor
    LowPassFilter(float gain);

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T        apply(T sample);

    // reset - clear the filter - next sample added will become the new base value
    virtual void        reset() {
        _base_value_set = false;
    };

    // reset - clear the filter and provide the new base value
    virtual void        reset( T new_base_value ) {
        _base_value = new_base_value; _base_value_set = true;
    };

private:
    float           _gain;                      // gain value  (like 0.02) applied to each new value
    bool            _base_value_set;    // true if the base value has been set
    float           _base_value;        // the number of samples in the filter, maxes out at size of the filter
};

// Typedef for convenience (1st argument is the data type, 2nd is a larger datatype to handle overflows, 3rd is buffer size)
typedef LowPassFilter<int8_t> LowPassFilterInt8;
typedef LowPassFilter<uint8_t> LowPassFilterUInt8;

typedef LowPassFilter<int16_t> LowPassFilterInt16;
typedef LowPassFilter<uint16_t> LowPassFilterUInt16;

typedef LowPassFilter<int32_t> LowPassFilterInt32;
typedef LowPassFilter<uint32_t> LowPassFilterUInt32;

typedef LowPassFilter<float> LowPassFilterFloat;

// Constructor    //////////////////////////////////////////////////////////////

template <class T>
LowPassFilter<T>::LowPassFilter(float gain) :
    Filter<T>(),
    _gain(gain),
    _base_value_set(false)
{
    // bounds checking on _gain
    if( _gain > 1.0) {
        _gain = 1.0;
    }else if( _gain < 0.0 ) {
        _gain = 0.0;
    }
};

// Public Methods //////////////////////////////////////////////////////////////

template <class T>
T LowPassFilter<T>::        apply(T sample)
{
    // initailise _base_value if required
    if( !_base_value_set ) {
        _base_value = sample;
        _base_value_set = true;
    }

    // do the filtering
    _base_value = _gain * (float)sample + (1.0 - _gain) * _base_value;

    // return the value.  Should be no need to check limits
    return (T)_base_value;
}

#endif