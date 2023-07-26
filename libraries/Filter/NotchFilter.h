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
class NotchFilter {
public:
    // set parameters
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q);
    T apply(const T &sample);
    void reset();

    // calculate attenuation and quality from provided center frequency and bandwidth
    static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q); 

protected:

    bool initialised, need_reset;
    float b0, b1, b2, a1, a2, a0_inv;
    float _center_freq_hz, _sample_freq_hz;
    T ntchsig, ntchsig1, ntchsig2, signal2, signal1;
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

