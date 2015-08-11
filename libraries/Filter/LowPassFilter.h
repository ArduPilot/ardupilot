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

#include <AP_Math/AP_Math.h>
#include "FilterClass.h"

// DigitalLPF implements the filter math
class DigitalLPF
{
public:
    // constructor
    DigitalLPF() :
    _output(0.0f) {}

    struct lpf_params {
        float cutoff_freq;
        float sample_freq;
        float alpha;
    };

    // add a new raw value to the filter, retrieve the filtered result
    float apply(float sample, float cutoff_freq, float dt) {
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
    float get() const {
        return _output;
    }

    void reset(float value) { _output = value; }

private:
    float _output;
};

// LPF base class
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

protected:
    float _cutoff_freq;
};

// LPF for a single float
class LowPassFilterFloat : public LowPassFilter
{
public:
    LowPassFilterFloat() :
    LowPassFilter() {}

    LowPassFilterFloat(float cutoff_freq):
    LowPassFilter(cutoff_freq) {}

    float apply(float sample, float dt) {
        return _filter.apply(sample, _cutoff_freq, dt);
    }

    float get() const {
        return _filter.get();
    }

    void reset(float value) {
        _filter.reset(value);
    }
private:
    DigitalLPF _filter;
};

// LPF for a 2D vector
class LowPassFilterVector2f : public LowPassFilter
{
public:
    LowPassFilterVector2f() :
    LowPassFilter() {}

    LowPassFilterVector2f(float cutoff_freq) :
    LowPassFilter(cutoff_freq) {}

    Vector2f apply(const Vector2f &sample, float dt) {
        Vector2f ret;
        ret.x = _filter_x.apply(sample.x, _cutoff_freq, dt);
        ret.y = _filter_y.apply(sample.y, _cutoff_freq, dt);
        return ret;
    }

    void reset(const Vector2f& value) {
        _filter_x.reset(value.x);
        _filter_y.reset(value.y);
    }

private:
    DigitalLPF _filter_x;
    DigitalLPF _filter_y;
};

// LPF for 3D vector
class LowPassFilterVector3f : public LowPassFilter
{
public:
    LowPassFilterVector3f() :
    LowPassFilter() {}

    LowPassFilterVector3f(float cutoff_freq) :
    LowPassFilter(cutoff_freq) {}

    Vector3f apply(const Vector3f &sample, float dt) {
        Vector3f ret;
        ret.x = _filter_x.apply(sample.x, _cutoff_freq, dt);
        ret.y = _filter_y.apply(sample.y, _cutoff_freq, dt);
        ret.z = _filter_z.apply(sample.z, _cutoff_freq, dt);
        return ret;
    }

    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    Vector3f get() const {
        Vector3f ret;
        ret.x = _filter_x.get();
        ret.y = _filter_y.get();
        ret.z = _filter_z.get();
        return ret;
    }

    void reset(const Vector3f& value) {
        _filter_x.reset(value.x);
        _filter_y.reset(value.y);
        _filter_z.reset(value.z);
    }

private:
    DigitalLPF _filter_x;
    DigitalLPF _filter_y;
    DigitalLPF _filter_z;
};

#endif // __LOW_PASS_FILTER_H__
