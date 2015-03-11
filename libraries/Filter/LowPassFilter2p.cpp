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

/// @file	LowPassFilter.cpp
/// @brief	A class to implement a second order low pass filter 
/// Author: Leonard Hall <LeonardTHall@gmail.com>

#include <inttypes.h>
#include <AP_Math.h>
#include "LowPassFilter2p.h"

float DigitalBiquadFilter::apply(float sample, const struct biquad_params &params)
{
    if(params.cutoff_freq == 0 || params.sample_freq == 0) {
        return sample;
    }

    float delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        delay_element_0 = sample;
    }
    float output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

void DigitalBiquadFilter::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret)
{
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
