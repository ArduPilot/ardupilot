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

// Vehicle-agnostic ground-effect takeoff/touchdown detector. Decides when
// the EKF should be told to expect baro disturbance from rotor downwash
// near the ground, and pushes that signal to AP_AHRS.
//
// The vehicle wires an AC_PosControl in once and the library queries it
// (and AP::ahrs()) directly each tick. Per-cycle vehicle state needed by
// the takeoff window is passed straight to update(); mode-conditional or
// sensor-derived flags use setters so the vehicle only updates them when
// the underlying signal actually changes.

#pragma once

#include "AP_GroundEffect_config.h"

#if AP_GROUNDEFFECT_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AC_PosControl;

class AP_GroundEffect
{
public:
    AP_GroundEffect();

    CLASS_NO_COPY(AP_GroundEffect);

    static const struct AP_Param::GroupInfo var_info[];

    // Wire the position controller the library queries each tick. Called
    // once after the vehicle has allocated its AC_PosControl. The library
    // can not take the controller as a constructor argument because it
    // typically lives inside the vehicle's static parameter object, which
    // is built before AC_PosControl is allocated.
    void set_pos_control(const AC_PosControl &pos_control)
    {
        _pos_control = &pos_control;
    }

    // Mode-conditional and sensor-derived signal setters. Each defaults
    // to false; the vehicle calls these when the underlying state
    // changes (per-tick polling is also fine).

    // suppress the takeoff_expected window even while armed and landed
    // (e.g. ArduCopter THROW mode, which never wants the takeoff signal)
    void set_takeoff_inhibited(bool b)
    {
        _takeoff_inhibited = b;
    }
    // mode-specific override for the slow-horizontal check: vehicle
    // pre-evaluates "I'm in a manual-attitude mode AND the requested
    // attitude is near level" and passes the boolean result here. Lets
    // the library stay ignorant of vehicle mode enums and attitude
    // controllers.
    void set_pilot_demanding_slow_horizontal(bool b)
    {
        _pilot_slow_horizontal = b;
    }
    // vehicle's high-vibration flag, forwarded to AHRS get_velocity_D()
    void set_high_vibrations(bool b)
    {
        _high_vibrations = b;
    }

    // Per-cycle entry point. Queries AHRS and the wired AC_PosControl,
    // runs the detector, and pushes set_takeoff_expected /
    // set_touchdown_expected to AP_AHRS. The three args are the basic
    // vehicle-state signals the takeoff window needs each tick:
    //   armed         - motors are armed
    //   land_complete - vehicle is on the ground per the land detector
    //   throttle_up   - pilot is manually commanding throttle up (while
    //                   false the takeoff timer/altitude reference is held)
    void update(bool armed, bool land_complete, bool throttle_up);

    bool takeoff_expected() const
    {
        return _state.takeoff_expected;
    }
    bool touchdown_expected() const
    {
        return _state.touchdown_expected;
    }

    bool enabled() const
    {
        return _enabled;
    }

private:
    AP_Int8  _enabled;
    AP_Float _alt_m;       // altitude threshold above which compensation clears
    AP_Float _timeout_s;   // minimum hold time before altitude check is allowed to release

    const AC_PosControl *_pos_control;

    // mode/sensor-derived inputs, updated via setters
    bool _takeoff_inhibited;
    bool _pilot_slow_horizontal;
    bool _high_vibrations;

    struct {
        bool     takeoff_expected;
        bool     touchdown_expected;
        uint32_t takeoff_time_ms;
        float    takeoff_alt_m;
        Vector2f takeoff_pos_ne_m;  // EKF-origin XY at liftoff, used by the relative-to-takeoff fallback
    } _state;
};

#endif  // AP_GROUNDEFFECT_ENABLED
