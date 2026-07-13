#pragma once

/*
   BionicYaw
   Biomimetic tail mixer for ArduPilot

   Copyright (C) 2026 Abderrahim KHOUK

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "AP_BionicYaw_config.h"
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

class AP_BionicYaw
{
public:

    struct Output
    {
        float left;
        float right;
    };

    AP_BionicYaw() = default;

    // AP_Param group info, registered as a subgroup in Plane's
    // Parameters2 (g2) table so gains show up as BYAW_YAW_GAIN,
    // BYAW_ROLL_GAIN, BYAW_PITCH_GAIN in the GCS.
    static const struct AP_Param::GroupInfo var_info[];

    enum class Mode : uint8_t {
        DIFFERENTIAL_VTAIL = 0,  // Phase 2: two flat VTail surfaces mixed differentially
        ROTATING_TAIL      = 1,  // Phase 3: whole tail boom rotates as one rigid actuator
    };
    Mode get_mode(void) const { return (Mode)_mode.get(); }

    // which k_scriptingN (1..16) drives the rotating tail actuator
    // when in ROTATING_TAIL mode
    uint8_t get_rotator_function_offset(void) const { return (uint8_t)_rot_fn.get(); }

    void set_gains(float yaw_gain,
                   float roll_gain,
                   float pitch_gain = 1.0f);

    // roll/pitch/yaw are expected in centidegrees (ArduPilot's
    // standard SRV_Channels::get_output_scaled() range, -4500..4500),
    // matching k_aileron/k_elevator/k_rudder scaling.
    Output update(float roll,
                  float pitch,
                  float yaw);

    // Phase 3: converts a yaw demand (centidegrees, same -4500..4500
    // scale as k_rudder) into a rotation command for the tail-boom
    // actuator, clamped to +/-BYAW_ROT_MAX degrees. Returned value is
    // in the same centidegree "angle" scale used by k_scriptingN
    // outputs (set_angle(4500)), so it can be passed straight to
    // SRV_Channels::set_output_scaled().
    float update_rotator(float yaw) const;

private:

    AP_Float _yaw_gain;
    AP_Float _roll_gain;
    AP_Float _pitch_gain;
    AP_Int8 _enabled; // 1 = aktiv, 0 = deaktiviert
    AP_Int8 _mode;       // 0 = differential VTail, 1 = rotating tail
    AP_Float _rot_max_deg; // max rotator deflection in degrees, 0..45
    AP_Int8 _rot_fn;     // which k_scriptingN (1..16) to drive
    AP_Float _roll_couple_gain;   // BYAW_RC_GAIN: aileron correction per sin(phi), 0 = off
};
