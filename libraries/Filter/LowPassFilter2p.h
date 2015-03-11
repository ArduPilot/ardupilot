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

/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
/// Author: Leonard Hall <LeonardTHall@gmail.com>

class DigitalBiquadFilter
{
public:
    DigitalBiquadFilter() :
    _delay_element_1(0.0f),
    _delay_element_2(0.0f){}

    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };

    float apply(float sample, const struct biquad_params &params);

    void reset() { _delay_element_1 = _delay_element_2 = 0.0f; }

    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);

private:
    float _delay_element_1;
    float _delay_element_2;
};

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
        DigitalBiquadFilter::compute_params(sample_freq, cutoff_freq, _params);
    }

    // return the cutoff frequency
    float get_cutoff_freq(void) const {
        return _params.cutoff_freq;
    }

    float get_sample_freq(void) const {
        return _params.sample_freq;
    }

protected:
    struct DigitalBiquadFilter::biquad_params _params;
};

class LowPassFilter2pfloat : public LowPassFilter2p
{
public:
    LowPassFilter2pfloat() :
    LowPassFilter2p() {}

    LowPassFilter2pfloat(float sample_freq, float cutoff_freq):
    LowPassFilter2p(sample_freq,cutoff_freq) {}

    float apply(float sample) {
        return _filter.apply(sample, _params);
    }
private:
    DigitalBiquadFilter _filter;
};

class LowPassFilter2pVector3f : public LowPassFilter2p
{
public:
    LowPassFilter2pVector3f() :
    LowPassFilter2p() {}

    LowPassFilter2pVector3f(float sample_freq, float cutoff_freq) :
    LowPassFilter2p(sample_freq,cutoff_freq) {}

    Vector3f apply(const Vector3f &sample) {
        Vector3f ret;
        ret.x = _filter_x.apply(sample.x, _params);
        ret.y = _filter_y.apply(sample.y, _params);
        ret.z = _filter_z.apply(sample.z, _params);
        return ret;
    }

private:
    DigitalBiquadFilter _filter_x;
    DigitalBiquadFilter _filter_y;
    DigitalBiquadFilter _filter_z;
};


#endif // LOWPASSFILTER2P_H
