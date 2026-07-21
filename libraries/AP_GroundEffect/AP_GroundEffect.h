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
// near the ground, and pushes that signal to AP_AHRS via
// set_takeoff_expected / set_touchdown_expected. The vehicle wires an
// AC_PosControl in once and the library queries it (and AP::ahrs())
// directly each tick. Per-cycle vehicle state the takeoff window needs
// is passed straight to update(); mode-conditional or sensor-derived
// flags use setters so the vehicle only updates them when the underlying
// signal actually changes.
//
// Two independent signals are emitted:
//
//   takeoff_expected   - latched true once the vehicle is armed and
//                        land_complete, cleared once (a) GNDEFF_TMO has
//                        elapsed AND height has cleared GNDEFF_ALT, or
//                        (b) a 5 s hard timeout fires. GNDEFF_TMO
//                        exists to keep compensation engaged through a
//                        baro disturbance window even when the EKF
//                        altitude has already crossed the threshold.
//
//   touchdown_expected - re-evaluated every tick from slow horizontal
//                        speed AND slow descent AND "near ground"
//                        (defined by GNDEFF_ALT). Never latched: it is
//                        the instantaneous answer to "does this look
//                        like a landing approach right now?".
//
// Height source (used by both signals via "above GNDEFF_ALT" /
// "below GNDEFF_ALT" checks) is selected in this order:
//
//   1. AP_AHRS::get_hagl()             - rangefinder, or EKF3's
//                                        optflow AGL Kalman filter
//   2. AP_Terrain::height_above_terrain()
//                                      - GPS position plus onboard
//                                        terrain tiles for current loc
//   3. relative-to-takeoff (-pos_d minus the altitude latched at
//                          takeoff) with horizontal position available
//   4. relative-to-takeoff with no horizontal position (baro-only):
//                          assumes the ground beneath the vehicle is at
//                          the takeoff elevation
//
// Paths 3 and 4 are not strictly AGL: they trust that the ground has
// not changed elevation since takeoff. For the takeoff_expected window
// (which closes within ~5 s of takeoff) that is almost always fine.
// For touchdown_expected (which the vehicle may evaluate minutes later,
// hundreds of metres from launch) it is not, so path 3 additionally
// requires the vehicle to be within
// AP_GROUNDEFFECT_TAKEOFF_DRIFT_MAX_M of the takeoff XY position before
// the touchdown altitude gate is allowed to fire. Drift further than
// that and touchdown_expected stays false regardless of motion, since
// we have no basis to believe the ground below is at takeoff elevation.
// Path 4 (no horizontal position at all) cannot apply the drift gate
// and has to assume flat terrain.
//
// GNDEFF_ALT carries three regimes:
//
//   <  0   library entirely disabled, no signals emitted
//   == 0   library on; touchdown altitude gate disabled (matches the
//          legacy "any gentle descent counts" behaviour); takeoff window
//          relies on the 5 s hard cap (and GNDEFF_TMO if set)
//   >  0   library on with the threshold actively gating both sides

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
    void set_pos_control(const AC_PosControl &pos_control) { _pos_control = &pos_control; }

    // Mode-conditional and sensor-derived signal setters. The vehicle
    // calls these when the underlying state changes (per-tick polling is
    // also fine).

    // Enable ground effect compensation during takeoff.
    // Defaults to true, set to false in modes that disable ground effect compensation
    // (e.g. Copter's Throw mode)
    void enable_takeoff_comp(bool b) { _takeoff_comp_enabled = b; }

    // mode-specific override for the slow-horizontal check
    // should be set to true in angle control modes (e.g AltHold) with angle target < 7.5deg
    void set_pilot_demanding_slow_horizontal(bool b) { _pilot_slow_horizontal = b; }

    // vehicle's high-vibration flag, forwarded to AHRS get_velocity_D()
    void set_high_vibrations(bool b) { _high_vibrations = b; }

    // Per-cycle entry point. Queries AHRS and the wired AC_PosControl,
    // runs the detector, and pushes set_takeoff_expected /
    // set_touchdown_expected to AP_AHRS. The three args are the basic
    // vehicle-state signals the takeoff window needs each tick:
    //   armed         - motors are armed
    //   land_complete - vehicle is on the ground per the land detector
    //   throttle_up   - pilot is manually commanding throttle up (while
    //                   false the takeoff timer/altitude reference is held)
    void update(bool armed, bool land_complete, bool throttle_up);

private:
    AP_Float _alt_m;       // altitude threshold; <0 disables the library entirely
    AP_Float _timeout_s;   // minimum hold time before altitude check is allowed to release

    const AC_PosControl *_pos_control;

    // mode/sensor-derived inputs, updated via setters
    bool _takeoff_comp_enabled = true;
    bool _pilot_slow_horizontal;
    bool _high_vibrations;

    struct {
        bool     takeoff_expected;
        bool     touchdown_expected;
        uint32_t takeoff_time_ms;
        float    takeoff_alt_m;
        Vector2f takeoff_pos_ne_m;  // EKF-origin XY at takeoff, used by the relative-to-takeoff fallback
    } _state;
};

#endif  // AP_GROUNDEFFECT_ENABLED
