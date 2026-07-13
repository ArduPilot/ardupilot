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


#include "AP_BionicYaw.h"

const AP_Param::GroupInfo AP_BionicYaw::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: BionicYaw enable
    // @Description: Enables the BionicYaw differential/turn-coordination tail mixer. When disabled, a plain pitch+/-yaw VTail mix is used instead.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_BionicYaw, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: YAW_GAIN
    // @DisplayName: BionicYaw yaw mixing gain
    // @Description: Amount of yaw demand mixed differentially into the tail surfaces.
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("YAW_GAIN", 1, AP_BionicYaw, _yaw_gain, 0.80f),

    // @Param: ROLL_GAIN
    // @DisplayName: BionicYaw roll assist gain
    // @Description: Amount of roll demand mixed differentially into the tail surfaces, for turn coordination (mimicking how birds use asymmetric tail deflection during banked turns).
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("ROLL_GAIN", 2, AP_BionicYaw, _roll_gain, 0.15f),

    // @Param: PITCH_GAIN
    // @DisplayName: BionicYaw pitch gain
    // @Description: Scaling applied to pitch demand before mixing into the tail surfaces.
    // @Range: 0.5 1.5
    // @User: Standard
    AP_GROUPINFO("PITCH_GAIN", 3, AP_BionicYaw, _pitch_gain, 1.00f),

    // @Param: MODE
    // @DisplayName: BionicYaw tail actuation mode
    // @Description: 0 selects the differential VTail mixer (two flat tail surfaces, YAW/ROLL/PITCH_GAIN apply). 1 selects the rotating tail-boom actuator (single servo rotates the whole tail, ROT_MAX/ROT_FN apply instead).
    // @Values: 0:DifferentialVTail,1:RotatingTail
    // @User: Standard
    AP_GROUPINFO("MODE", 4, AP_BionicYaw, _mode, 0),

    // @Param: ROT_MAX
    // @DisplayName: BionicYaw rotator max deflection
    // @Description: Maximum rotation of the tail-boom actuator in ROTATING_TAIL mode, in either direction. Full rudder stick deflection (45 deg / 4500 centidegrees) maps to this value.
    // @Range: 0.0 45.0
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ROT_MAX", 5, AP_BionicYaw, _rot_max_deg, 45.0f),

    // @Param: ROT_FN
    // @DisplayName: BionicYaw rotator scripting output
    // @Description: Selects which Scripting servo output (SERVOx_FUNCTION = ScriptingN) drives the rotating tail-boom actuator in ROTATING_TAIL mode. A value of 1 uses Scripting1, 2 uses Scripting2, and so on up to 16.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("ROT_FN", 6, AP_BionicYaw, _rot_fn, 1),

    AP_GROUPEND
};

void AP_BionicYaw::set_gains(float yaw_gain,
                             float roll_gain,
                             float pitch_gain)
{
    _yaw_gain.set(yaw_gain);
    _roll_gain.set(roll_gain);
    _pitch_gain.set(pitch_gain);
}

AP_BionicYaw::Output AP_BionicYaw::update(float roll,
                                          float pitch,
                                          float yaw)
{
    Output out;

    if (_enabled <= 0) {
        // Fallback auf Standard-Mischung
        out.left = pitch + yaw;
        out.right = pitch - yaw;
        return out;
    }

    const float pitch_cmd = pitch * _pitch_gain;
    const float yaw_cmd   = yaw * _yaw_gain;
    const float roll_cmd  = roll * _roll_gain;

    out.left =
        pitch_cmd +
        yaw_cmd +
        roll_cmd;

    out.right =
        pitch_cmd -
        yaw_cmd -
        roll_cmd;

    out.left  = constrain_float(out.left,  -4500.0f, 4500.0f);
    out.right = constrain_float(out.right, -4500.0f, 4500.0f);

    return out;
}

float AP_BionicYaw::update_rotator(float yaw) const
{
    if (_enabled <= 0) {
        // disabled: hold the actuator centered rather than driving
        // it, so a disabled BionicYaw doesn't leave the tail cocked
        // at some arbitrary angle
        return 0.0f;
    }

    // BYAW_ROT_MAX is in whole degrees, convert to the same
    // centidegree "angle" scale k_scriptingN outputs use
    // (set_angle(4500) == +/-45.00 deg, matching k_rudder).
    const float max_cd = constrain_float(_rot_max_deg, 0.0f, 45.0f) * 100.0f;

    return constrain_float(yaw, -max_cd, max_cd);
}
