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

/*
  notch filter with settable sample rate, center frequency, bandwidth and attenuation

  Design by Leonard Hall
 */

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>
#include <AP_Param/AP_Param.h>


template <class T>
class HarmonicNotchFilter;

template <class T>
class NotchFilter {
public:
    friend class HarmonicNotchFilter<T>;

    NotchFilter();
    ~NotchFilter();
    CLASS_NO_COPY(NotchFilter);

    // maximum number of harmonics supported; sized to the width of the harmonics bitmask
    static constexpr uint8_t max_harmonics = sizeof(uint16_t) * 8;

    // set parameters
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB, uint16_t harmonics);
    void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q);
    T apply(const T &sample);
    void reset();
    float center_freq_hz() const { return _center_freq_hz; }
    float sample_freq_hz() const { return _sample_freq_hz; }
    // returns the requested harmonics bitmask; may include harmonics above Nyquist
    // that were not instantiated - check that initialised is true for active state
    uint16_t harmonics() const { return _harmonics; }

    // calculate attenuation and quality from provided center frequency and bandwidth
    static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q);

    void disable(void) {
        initialised = false;
    }

    // return the frequency to log for the notch
    float logging_frequency(void) const;

protected:
    // compute coefficients for a single biquad notch
    static void compute_coefficients(float &b0, float &b1, float &b2, float &a1, float &a2,
                                     float sample_freq_hz, float center_freq_hz, float A, float Q);

    bool initialised, need_reset;
    float b0, b1, b2, a1, a2;
    float _center_freq_hz, _sample_freq_hz, _A;
    T ntchsig1, ntchsig2, signal2, signal1;
    // requested harmonics bitmask as passed to init(); may include bits for harmonics
    // that were skipped due to Nyquist constraints - use _num_harmonics for active count
    uint16_t _harmonics;
    // number of sub-filters actually initialised (≤ popcount(_harmonics))
    uint8_t _num_harmonics;
    // array of single-notch filters, one per active harmonic (nullptr when harmonics == 1)
    NotchFilter<T>* _harmonic_filters;
};

/*
  notch filter enable and filter parameters
 */
class NotchFilterParams {
public:
    float center_freq_hz(void) const { return _center_freq_hz; }
    float bandwidth_hz(void) const { return _bandwidth_hz; }
    float attenuation_dB(void) const { return _attenuation_dB; }
    uint8_t enabled(void) const { return _enable; }
    void enable() { _enable.set(true); }
    
protected:
    AP_Int8 _enable;
    AP_Float _center_freq_hz;
    AP_Float _bandwidth_hz;
    AP_Float _attenuation_dB;
};

typedef NotchFilter<float> NotchFilterFloat;
typedef NotchFilter<Vector2f> NotchFilterVector2f;
typedef NotchFilter<Vector3f> NotchFilterVector3f;

