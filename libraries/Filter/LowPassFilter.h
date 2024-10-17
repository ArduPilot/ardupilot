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
/// @brief	A class to implement a low pass filter.

/*
  Two classes are provided:

   LowPassFilter: providing dt on every sample, and calling apply like this:

      // call once
      filter.set_cutoff_frequency(frequency_hz);

      // then on each sample
      output = filter.apply(sample, dt);

   LowPassFilterConstDt: providing a sample freq and cutoff_freq once at start

      // call once
      filter.set_cutoff_frequency(sample_freq, frequency_hz);

      // then on each sample
      output = filter.apply(sample);

  The second approach is more CPU efficient as it doesn't have to
  recalculate alpha each time, but it assumes that dt is constant
 */

#pragma once

#include <AP_Math/AP_Math.h>

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF {
public:

    // constructor
    DigitalLPF();

    CLASS_NO_COPY(DigitalLPF);

    // get latest filtered value from filter (equal to the value returned by latest call to apply method)
    const T &get() const;

    // Reset filter to given value
    void reset(const T &value);

    // Set reset flag such that the filter will be reset to the next value applied
    void reset();

protected:
    // add a new raw value to the filter, retrieve the filtered result
    T _apply(const T &sample, const float &alpha);

private:
    T output;
    bool initialised;
};

// Low pass filter with constant time step
template <class T>
class LowPassFilterConstDt : public DigitalLPF<T> {
public:

    // constructors
    LowPassFilterConstDt() {};
    LowPassFilterConstDt(const float &sample_freq, const float &cutoff_freq);

    CLASS_NO_COPY(LowPassFilterConstDt);

    // change parameters
    void set_cutoff_frequency(const float &sample_freq, const float &cutoff_freq);

    // return the cutoff frequency
    float get_cutoff_freq() const;

    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample);

private:
    float cutoff_freq;
    float alpha;
};

typedef LowPassFilterConstDt<float>    LowPassFilterConstDtFloat;
typedef LowPassFilterConstDt<Vector2f> LowPassFilterConstDtVector2f;
typedef LowPassFilterConstDt<Vector3f> LowPassFilterConstDtVector3f;

// Low pass filter with variable time step
template <class T>
class LowPassFilter : public DigitalLPF<T> {
public:

    // constructors
    LowPassFilter() {};
    LowPassFilter(const float &cutoff_freq);

    CLASS_NO_COPY(LowPassFilter);

    // change parameters
    void set_cutoff_frequency(const float &cutoff_freq);

    // return the cutoff frequency
    float get_cutoff_freq() const;

    // add a new raw value to the filter, retrieve the filtered result
    T apply(const T &sample, const float &dt);

private:
    float cutoff_freq;
};

// typedefs for compatibility
typedef LowPassFilter<float>    LowPassFilterFloat;
typedef LowPassFilter<Vector2f> LowPassFilterVector2f;
typedef LowPassFilter<Vector3f> LowPassFilterVector3f;
