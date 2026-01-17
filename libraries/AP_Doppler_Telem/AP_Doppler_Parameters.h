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

#include "AP_Doppler_Telem.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

class AP_Doppler_Telem;

class AP_Doppler_Parameters
{
    friend class AP_Doppler_SPort_Passthrough;
public:
    AP_Doppler_Parameters();

    // parameters
    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enable.get() != 0; }
    float q_min() const { return _q_min.get(); }
    float vel_err() const { return _vel_err.get(); }
    float vel_err_wtr() const { return _vel_err_wtr.get(); }
    const Vector3f &pos_offset_body() const { return _pos_offset; }

private:
    // settable parameters
    AP_Int8 _enable;
    AP_Float _q_min;
    AP_Float _vel_err;
    AP_Float _vel_err_wtr;
    AP_Vector3f _pos_offset;
    AP_Int8 _options;
};
