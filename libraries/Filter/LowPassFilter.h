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
#pragma once

#include <AP_Math/AP_Math.h>
#include "FilterClass.h"

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:
    struct lpf_params {
        float cutoff_freq;
        float sample_freq;
        float alpha;
    };  

    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, float cutoff_freq, float dt);
    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;
    void reset(T value);

private:
    T _output;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(float cutoff_freq);

    // change parameters
    void set_cutoff_frequency(float cutoff_freq);
    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    T apply(T sample, float dt);
    const T &get() const;
    void reset(T value);
    
protected:
    float _cutoff_freq;

private:
    DigitalLPF<T> _filter;
};

// Uncomment this, if you decide to remove the instantiations in the implementation file
/*
template <class T>
LowPassFilter<T>::LowPassFilter() : _cutoff_freq(0.0f) { 
  
}
// constructor
template <class T>
LowPassFilter<T>::LowPassFilter(float cutoff_freq) : _cutoff_freq(cutoff_freq) { 
  
}
*/

// typedefs for compatibility
typedef LowPassFilter<int>      LowPassFilterInt;
typedef LowPassFilter<long>     LowPassFilterLong;
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Vector2f> LowPassFilterVector2f;
typedef LowPassFilter<Vector3f> LowPassFilterVector3f;
