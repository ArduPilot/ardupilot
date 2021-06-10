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

/*
  Note that this filter can be used in 2 ways:

   1) providing dt on every sample, and calling apply like this:

      // call once
      filter.set_cutoff_frequency(frequency_hz);

      // then on each sample
      output = filter.apply(sample, dt);

   2) providing a sample freq and cutoff_freq once at start

      // call once
      filter.set_cutoff_frequency(sample_freq, frequency_hz);

      // then on each sample
      output = filter.apply(sample);

  The second approach is more CPU efficient as it doesn't have to
  recalculate alpha each time, but it assumes that dt is constant
 */

#pragma once

#include <AP_Math/AP_Math.h>
#include "FilterClass.h"

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:
    DigitalLPF();
    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, float cutoff_freq, float dt);
    T apply(const T &sample);

    CLASS_NO_COPY(DigitalLPF);

    void compute_alpha(float sample_freq, float cutoff_freq);
    
    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;
    void reset(T value);
    void reset() {
        initialised = false;
    }

private:
    T _output;
    float alpha = 1.0f;
    bool initialised;
};

// LPF base class
template <class T>
class LowPassFilter {
public:
    LowPassFilter();
    LowPassFilter(float cutoff_freq);
    LowPassFilter(float sample_freq, float cutoff_freq);

    CLASS_NO_COPY(LowPassFilter);

    // change parameters
    void set_cutoff_frequency(float cutoff_freq);
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    T apply(T sample, float dt);
    T apply(T sample);
    const T &get() const;
    void reset(T value);
    void reset(void) { _filter.reset(); }

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
