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
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>


/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
/// @authors: Leonard Hall <LeonardTHall@gmail.com>, template implmentation: Daniel Frenzel <dgdanielf@gmail.com>
template <class T>
class DigitalBiquadFilter {
public:
    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };
  
    DigitalBiquadFilter();

    T apply(const T &sample, const struct biquad_params &params);
    void reset();
    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);
    
private:
    T _delay_element_1;
    T _delay_element_2;
};

template <class T>
class LowPassFilter2p {
public:
    LowPassFilter2p();
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq);
    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    float get_sample_freq(void) const;
    T apply(const T &sample);
    void reset(void);

protected:
    struct DigitalBiquadFilter<T>::biquad_params _params;
    
private:
    DigitalBiquadFilter<T> _filter;
};

// Uncomment this, if you decide to remove the instantiations in the implementation file
/*
template <class T>
LowPassFilter2p<T>::LowPassFilter2p() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
LowPassFilter2p<T>::LowPassFilter2p(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}
*/

typedef LowPassFilter2p<int>      LowPassFilter2pInt;
typedef LowPassFilter2p<long>     LowPassFilter2pLong;
typedef LowPassFilter2p<float>    LowPassFilter2pFloat;
typedef LowPassFilter2p<Vector2f> LowPassFilter2pVector2f;
typedef LowPassFilter2p<Vector3f> LowPassFilter2pVector3f;
