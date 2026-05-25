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

    enum class FusionMode : int8_t {
        Disabled = 0,
        BodyOdom = 1,
        ExtNavVel = 2,
    };

    // parameters
    static const struct AP_Param::GroupInfo var_info[];

    bool mav_enabled() const { return _mav_en.get() != 0; }
    uint8_t mav_rate_hz() const { return uint8_t(MAX(int16_t(_mav_rate_hz.get()), int16_t(1))); }
    bool sim_enabled() const { return _sim_en.get() != 0; }
    FusionMode fusion_mode() const { return (FusionMode)_fusion_mode.get(); }
    bool body_odom_enabled() const { return fusion_mode() == FusionMode::BodyOdom; }
    bool extnav_vel_enabled() const { return fusion_mode() == FusionMode::ExtNavVel; }
    enum Rotation orientation() const { return (enum Rotation)_orientation.get(); }
    const Vector3f &pos_offset() const { return _pos_offset; }
    uint16_t delay_ms() const { return MAX(0, _delay_ms.get()); }
    bool use_water_track() const { return _use_water_track.get() != 0; }
    float min_quality() const { return constrain_float(_min_quality.get(), 0.0f, 100.0f); }
    const Vector3f &sim_velocity() const { return _sim_velocity; }
    float sim_altitude_m() const { return MAX(_sim_altitude_m.get(), 0.1f); }
    float sim_quality() const { return constrain_float(_sim_quality.get(), 0.0f, 100.0f); }
    bool debug_enabled() const { return _debug.get() != 0; }


private:
    AP_Int8 _options;
    AP_Int8 _mav_en;
    AP_Int8 _mav_rate_hz;
    AP_Int8 _sim_en;
    AP_Int8 _fusion_mode;
    AP_Int8 _orientation;
    AP_Vector3f _pos_offset;
    AP_Int16 _delay_ms;
    AP_Int8 _use_water_track;
    AP_Float _min_quality;
    AP_Vector3f _sim_velocity;
    AP_Float _sim_altitude_m;
    AP_Float _sim_quality;
    AP_Int8 _debug;

    // settable parameters

};
