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

#include "NotchFilter.h"

template <class T>
class NotchFilterIncParams : public NotchFilter<T>{

public:
    
    // Inherit constructor
    // using NotchFilter<T>::NotchFilter<T>;
    NotchFilterIncParams(void);

    void init(float sample_freq_hz);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Int8 _enable;
    AP_Float _center_freq_hz;
    AP_Float _bandwidth_hz;
    AP_Float _attenuation_dB;

};

typedef NotchFilterIncParams<float> NotchFilterIncParamsFloat;
typedef NotchFilterIncParams<Vector2f> NotchFilterIncParamVectors2f;
typedef NotchFilterIncParams<Vector3f> NotchFilterIncParamVectors3f;
