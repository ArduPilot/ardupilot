// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

#ifndef __LOW_PASS_FILTER_H__
#define __LOW_PASS_FILTER_H__

#include <AP_Math.h>
#include "FilterClass.h"

// 1st parameter <T> is the type of data being filtered.
template <class T>
class LowPassFilter : public Filter<T>
{
public:
    // constructor
    LowPassFilter();

    virtual void    set_cutoff_frequency(float time_step, float cutoff_freq);
    virtual void    set_time_constant(float time_step, float time_constant);

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
    float           _alpha;             // gain value  (like 0.02) applied to each new value
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
LowPassFilter<T>::LowPassFilter() :
    Filter<T>(),
    _alpha(1),
    _base_value_set(false)
{};

//    F_Cut = 1; % Hz
//RC = 1/(2*pi*F_Cut);
//Alpha = Ts/(Ts + RC);

// Public Methods //////////////////////////////////////////////////////////////

template <class T>
void LowPassFilter<T>::set_cutoff_frequency(float time_step, float cutoff_freq)
{
    // calculate alpha
    float rc = 1/(2*PI*cutoff_freq);
    _alpha = time_step / (time_step + rc);
}

template <class T>
void LowPassFilter<T>::set_time_constant(float time_step, float time_constant)
{
    // calculate alpha
    _alpha = time_step / (time_constant + time_step);
}

template <class T>
T LowPassFilter<T>::apply(T sample)
{
    // initailise _base_value if required
    if( !_base_value_set ) {
        _base_value = sample;
        _base_value_set = true;
    }

    // do the filtering
    //_base_value = _alpha * (float)sample + (1.0 - _alpha) * _base_value;
    _base_value = _base_value + _alpha * ((float)sample - _base_value);

    // return the value.  Should be no need to check limits
    return (T)_base_value;
}

#endif // __LOW_PASS_FILTER_H__
