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

/// @file	LowPassFilter.h
/// @brief	A class to implement a second order low pass filter 
/// Author: Leonard Hall <LeonardTHall@gmail.com>

class LowPassFilter2p
{
public:
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq) {
        // set initial parameters
        set_cutoff_frequency(sample_freq, cutoff_freq);
        _delay_element_1 = _delay_element_2 = 0;
    }

    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    // apply - Add a new raw value to the filter 
    // and retrieve the filtered result
    float apply(float sample);

    // return the cutoff frequency
    float get_cutoff_freq(void) const {
        return _cutoff_freq;
    }

private:
    float           _cutoff_freq; 
    float           _a1;
    float           _a2;
    float           _b0;
    float           _b1;
    float           _b2;
    float           _delay_element_1;        // buffered sample -1
    float           _delay_element_2;        // buffered sample -2
};

#endif // LOWPASSFILTER2P_H
