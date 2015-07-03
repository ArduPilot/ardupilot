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

#ifndef LOWPASSFILTER2P_H
#define LOWPASSFILTER2P_H

#include <AP_Math.h>
#include <inttypes.h>

/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
/// Author: Leonard Hall <LeonardTHall@gmail.com>

template <class T>
class DigitalBiquadFilter
{
public:
    DigitalBiquadFilter() {
      _delay_element_1 = T();
      _delay_element_2 = T();
    }

    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };

    T apply(T sample, const struct biquad_params &params) {
        if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
            return sample;
        }

        T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            delay_element_0 = sample;
        }
        T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

        _delay_element_2 = _delay_element_1;
        _delay_element_1 = delay_element_0;

        return output;
    }

    void reset() { _delay_element_1 = _delay_element_2 = T(); }

    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
        ret.cutoff_freq = cutoff_freq;
        ret.sample_freq = sample_freq;

        float fr = sample_freq/cutoff_freq;
        float ohm = tanf(PI/fr);
        float c = 1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;

        ret.b0 = ohm*ohm/c;
        ret.b1 = 2.0f*ret.b0;
        ret.b2 = ret.b0;
        ret.a1 = 2.0f*(ohm*ohm-1.0f)/c;
        ret.a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
    }
    
private:
    T _delay_element_1;
    T _delay_element_2;
};

template <class T>
class LowPassFilter2p
{
public:
    LowPassFilter2p() { memset(&_params, 0, sizeof(_params)); }
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq) {
        // set initial parameters
        set_cutoff_frequency(sample_freq, cutoff_freq);
    }

    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq) {
        DigitalBiquadFilter<T>::compute_params(sample_freq, cutoff_freq, _params);
    }

    // return the cutoff frequency
    float get_cutoff_freq(void) const {
        return _params.cutoff_freq;
    }

    float get_sample_freq(void) const {
        return _params.sample_freq;
    }
    
    T apply(T sample) {
        return _filter.apply(sample, _params);
    }

protected:
    struct DigitalBiquadFilter<T>::biquad_params _params;
    
private:
    DigitalBiquadFilter<T> _filter;
};

typedef LowPassFilter2p<float> LowPassFilter2pfloat;
typedef LowPassFilter2p<Vector3f> LowPassFilter2pVector3f;


#endif // LOWPASSFILTER2P_H
