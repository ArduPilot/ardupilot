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
#include <AP_Param/AP_Param.h>
#include "NotchFilter.h"

template <class T>
class HarmonicNotchFilter {
public:
    // set parameters
    void create(uint8_t harmonics);
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    T apply(const T &sample);
    void reset();
    void update(float center_freq_hz);
    ~HarmonicNotchFilter();

private:
    NotchFilter<T>*  _filters;
    float _sample_freq_hz;
    float _A;
    float _Q;
    uint8_t _harmonics;
    uint8_t _num_filters;
    bool _initialised;
};

/*
  notch filter enable and filter parameters
 */
class HarmonicNotchFilterParams : public NotchFilterParams {
public:
    HarmonicNotchFilterParams(void);
    void set_center_freq_hz(float center_freq) { _center_freq_hz.set(center_freq); }
    uint8_t harmonics(void) const { return _harmonics; }
    float reference(void) const { return _reference; }
    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _harmonics;
    AP_Float _reference;
};

typedef HarmonicNotchFilter<Vector3f> HarmonicNotchFilterVector3f;

