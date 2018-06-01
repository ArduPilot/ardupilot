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
    T apply(const T &sample);

private:
    bool initialised;
    float b0, b1, b2, a1, a2, a0_inv;
    T ntchsig, ntchsig1, ntchsig2, signal2, signal1;
};

/*
  a notch filter with enable and filter parameters
 */
class NotchFilterVector3fParam {
public:
    NotchFilterVector3fParam(void);
    void init(float sample_freq_hz);
    Vector3f apply(const Vector3f &sample);

    static const struct AP_Param::GroupInfo var_info[];
    
private:
    AP_Int8 enable;
    AP_Float center_freq_hz;
    AP_Float bandwidth_hz;
    AP_Float attenuation_dB;

    float sample_freq_hz;

    float last_center_freq;
    float last_bandwidth;
    float last_attenuation;
    
    NotchFilter<Vector3f> filter;
};

typedef NotchFilter<float> NotchFilterFloat;
typedef NotchFilter<Vector3f> NotchFilterVector3f;

