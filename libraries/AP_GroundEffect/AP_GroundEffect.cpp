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

#include "AP_GroundEffect.h"

#if AP_GROUNDEFFECT_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AC_AttitudeControl/AC_PosControl.h>

// hard cap on the takeoff_expected window, irrespective of GNDEFF_TMO
#define AP_GROUNDEFFECT_TAKEOFF_MAX_MS 5000U

// once we are using the relative-to-takeoff height fallback with GPS but
// no rangefinder/terrain, disable the touchdown altitude gate once the
// vehicle has drifted this far horizontally from where it lifted off, as
// the terrain elevation under it may differ from the launch site
#define AP_GROUNDEFFECT_TAKEOFF_DRIFT_MAX_M 20.0f

const AP_Param::GroupInfo AP_GroundEffect::var_info[] = {

    // 1 was ENABLE; the master switch is folded into GNDEFF_ALT
    // (negative value disables the library)

    // @Param: ALT
    // @DisplayName: Ground effect altitude threshold
    // @Description: Altitude threshold (AGL where possible) for the ground-effect signals, and master enable. Negative disables the library entirely (no takeoff or touchdown signals). Zero leaves the library enabled but with the touchdown altitude gate disabled, matching the legacy "any gentle descent counts" behaviour; the takeoff release then relies on the 5s hard cap and GNDEFF_TMO. Positive values gate both sides: on takeoff the compensation flag clears once the vehicle climbs above this altitude (subject to GNDEFF_TMO and a 5s hard cap); on approach the touchdown flag only sets when below this altitude. The library prefers HAGL from a rangefinder or onboard terrain data when available; otherwise it uses height-since-takeoff and assumes flat ground (with a 20m XY drift gate disabling the touchdown side once horizontal position is available).
    // @Range: -1 5
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ALT", 2, AP_GroundEffect, _alt_m, 0.5),

    // @Param: TMO
    // @DisplayName: Takeoff Ground Effect Timeout
    // @Description: Minimum hold time after liftoff before the takeoff ground-effect signal is allowed to clear on the GNDEFF_ALT altitude check. With this set the signal only clears once BOTH this time has elapsed AND height exceeds GNDEFF_ALT, which prevents premature release when baro disturbance corrupts the altitude estimate during the rotor-wash window. The 5s hard cap on the takeoff window still applies regardless. Zero disables this minimum hold and the altitude check alone releases the signal; vehicles with strong propwash baro disturbance benefit from values of 2-5s. Does not affect the touchdown signal.
    // @Range: 0 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TMO", 3, AP_GroundEffect, _timeout_s, 2),

    AP_GROUPEND
};

AP_GroundEffect::AP_GroundEffect()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_GroundEffect::update(bool armed, bool land_complete, bool throttle_up)
{
    AP_AHRS &ahrs = AP::ahrs();

    if (is_negative(_alt_m) || !armed) {
        // disarmed or disabled (GNDEFF_ALT < 0) - clear state and tell EKF nothing is expected
        _state.takeoff_expected = false;
        _state.touchdown_expected = false;
        ahrs.set_takeoff_expected(false);
        ahrs.set_touchdown_expected(false);
        return;
    }

    const uint32_t tnow_ms = AP_HAL::millis();

    // takeoff state machine
    if (_takeoff_inhibited) {
        _state.takeoff_expected = false;
    } else if (land_complete) {
        // armed and not yet airborne - takeoff is imminent
        _state.takeoff_expected = true;
    }

    // Anchor the takeoff timer, altitude and XY position while still on
    // the ground without throttle up. Only the relative-to-takeoff
    // fallback consumes these; HAGL / terrain paths ignore them.
    float pos_d_m = 0;
    UNUSED_RESULT(ahrs.get_relative_position_D_origin_float(pos_d_m));
    Vector2f pos_ne_m;
    const bool have_pos_ne = ahrs.get_relative_position_NE_origin_float(pos_ne_m);

    if (!throttle_up && land_complete) {
        _state.takeoff_time_ms = tnow_ms;
        _state.takeoff_alt_m = -pos_d_m;
        _state.takeoff_pos_ne_m = pos_ne_m;
    }

    // Pick the best available height. EKF HAGL covers rangefinder and
    // EKF3's optflow AGL KF; terrain database covers GPS + onboard tiles;
    // otherwise fall back to height-since-takeoff and assume flat ground.
    float height_m = 0;
    bool height_is_agl = false;
    if (ahrs.get_hagl(height_m)) {
        height_is_agl = true;
    }
#if AP_TERRAIN_AVAILABLE
    if (!height_is_agl) {
        AP_Terrain *terrain = AP::terrain();
        // extrapolate=false: with no tiles loaded, height_above_terrain
        // still "succeeds" by returning the raw AMSL altitude, which we
        // do not want; require real data.
        if (terrain != nullptr && terrain->height_above_terrain(height_m, false)) {
            height_is_agl = true;
        }
    }
#endif
    if (!height_is_agl) {
        height_m = -pos_d_m - _state.takeoff_alt_m;
    }

    // GNDEFF_TMO is a minimum hold time before the altitude check is
    // allowed to release; the 5s hard timeout still applies unconditionally.
    const uint32_t min_hold_ms = MIN(uint32_t(_timeout_s * 1000.0f), AP_GROUNDEFFECT_TAKEOFF_MAX_MS);
    const bool above_alt = height_m > _alt_m;
    const bool min_hold_elapsed = AP_HAL::timeout_expired(_state.takeoff_time_ms, tnow_ms, min_hold_ms);
    const bool max_timeout = AP_HAL::timeout_expired(_state.takeoff_time_ms, tnow_ms, AP_GROUNDEFFECT_TAKEOFF_MAX_MS);

    if (_state.takeoff_expected && (max_timeout || (min_hold_elapsed && above_alt))) {
        _state.takeoff_expected = false;
    }

    // touchdown logic - slow horizontal motion AND slow descent AND near ground
    const bool ne_active = (_pos_control != nullptr) && _pos_control->NE_is_active();
    const bool d_active  = (_pos_control != nullptr) && _pos_control->D_is_active();

    const bool xy_speed_demand_low = ne_active && _pos_control->get_vel_target_NED_ms().xy().length() <= 1.25f;

    Vector3f vel_ned_ms;
    const bool xy_speed_low = ahrs.get_velocity_NED(vel_ned_ms) && (vel_ned_ms.xy().length() < 1.25f);

    const bool slow_horizontal = xy_speed_demand_low
                                 || (xy_speed_low && !ne_active)
                                 || _pilot_slow_horizontal;

    const float target_climb_rate_ms = d_active ? _pos_control->get_vel_desired_U_ms() : 0.0f;
    const bool descent_demanded = d_active && target_climb_rate_ms < 0.0f;
    const bool slow_descent_demanded = descent_demanded && target_climb_rate_ms >= -1.0f;
    float vel_d_ms = 0;
    const bool speed_low_d = ahrs.get_velocity_D(vel_d_ms, _high_vibrations) && fabsf(vel_d_ms) <= 0.6f;
    const bool slow_descent = slow_descent_demanded || (speed_low_d && descent_demanded);

    // Touchdown altitude gate.
    //   - GNDEFF_ALT <= 0: legacy behaviour, any gentle descent counts
    //   - HAGL or terrain height available: trust height_m directly
    //   - relative-to-takeoff fallback with horizontal position: only
    //     trust the gate while still within AP_GROUNDEFFECT_TAKEOFF_DRIFT_MAX_M
    //     of the launch point; further out we cannot assume the ground
    //     beneath us is at the takeoff elevation
    //   - baro-only fallback (no horizontal position): assume flat ground
    bool near_ground;
    if (!is_positive(_alt_m)) {
        near_ground = true;
    } else if (height_is_agl || !have_pos_ne) {
        near_ground = height_m < _alt_m;
    } else {
        const float drift_m = (pos_ne_m - _state.takeoff_pos_ne_m).length();
        near_ground = (drift_m < AP_GROUNDEFFECT_TAKEOFF_DRIFT_MAX_M)
                      && (height_m < _alt_m);
    }

    _state.touchdown_expected = slow_horizontal && slow_descent && near_ground;

    ahrs.set_takeoff_expected(_state.takeoff_expected);
    ahrs.set_touchdown_expected(_state.touchdown_expected);
}

#endif  // AP_GROUNDEFFECT_ENABLED
