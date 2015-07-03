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

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF
{
public:
    DigitalLPF() {
      // built in initialization
      _output = T();
    }

    struct lpf_params {
        float cutoff_freq;
        float sample_freq;
        float alpha;
    };

    // add a new raw value to the filter, retrieve the filtered result
    T apply(T sample, float cutoff_freq, float dt) {
        if (cutoff_freq <= 0.0f || dt <= 0.0f) {
            _output = sample;
            return _output;
        }

        float rc = 1.0f/(M_2PI_F*cutoff_freq);
        float alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
        _output += (sample - _output) * alpha;
        return _output;
    }

    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    T get() const {
        return _output;
    }

    void reset(T value) { _output = value; }

private:
    T _output;
};

// LPF base class
template <class T>
class LowPassFilter
{
public:
    LowPassFilter() :
    _cutoff_freq(0.0f) { }
    // constructor
    LowPassFilter(float cutoff_freq) :
    _cutoff_freq(cutoff_freq) { }

    // change parameters
    void set_cutoff_frequency(float cutoff_freq) {
        _cutoff_freq = cutoff_freq;
    }

    // return the cutoff frequency
    float get_cutoff_freq(void) const {
        return _cutoff_freq;
    }

    T apply(T sample, float dt) {
        return _filter.apply(sample, _cutoff_freq, dt);
    }

    T get() const {
        return _filter.get();
    }

    void reset(T value) {
        _filter.reset(value);
    }
    
protected:
    float _cutoff_freq;

private:
    DigitalLPF<T> _filter;
};

// typedefs for compatibility
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Vector2f> LowPassFilterVector2f;
typedef LowPassFilter<Vector3f> LowPassFilterVector3f;

#endif // __LOW_PASS_FILTER_H__
