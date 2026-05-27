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
#include <AC_AttitudeControl/AC_PosControl.h>

// hard cap on the takeoff_expected window, irrespective of GNDEFF_TMO
#define AP_GROUNDEFFECT_TAKEOFF_MAX_MS 5000U

const AP_Param::GroupInfo AP_GroundEffect::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Ground Effect Compensation Enable/Disable
    // @Description: Ground Effect Compensation Enable/Disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_GroundEffect, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: ALT
    // @DisplayName: Ground effect altitude threshold
    // @Description: Altitude threshold for ground effect compensation. Takeoff ground effect compensation is cleared once the vehicle climbs above this altitude. Touchdown ground effect compensation is only signalled to the EKF when the vehicle descends below this altitude. Set to zero to disable the touchdown altitude gate.
    // @Range: 0 5
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ALT", 2, AP_GroundEffect, _alt_m, 0.5),

    // @Param: TMO
    // @DisplayName: Ground effect timeout
    // @Description: Time after throttle up before ground effect compensation can be disabled. When set, ground effect will only be disabled after BOTH this timeout has elapsed AND altitude exceeds GNDEFF_ALT. This prevents premature ground effect disabling when baro noise causes false altitude readings. Set to zero to disable (uses altitude threshold only). Maximum timeout is always 5 seconds regardless of this setting.
    // @Range: 0 5
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TMO", 3, AP_GroundEffect, _timeout_s, 0),

    AP_GROUPEND
};

AP_GroundEffect::AP_GroundEffect()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_GroundEffect::update(bool armed, bool land_complete, bool throttle_up)
{
    AP_AHRS &ahrs = AP::ahrs();

    if (!_enabled || !armed) {
        // disarmed or disabled - clear state and tell EKF nothing is expected
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

    float pos_d_m = 0;
    UNUSED_RESULT(ahrs.get_relative_position_D_origin_float(pos_d_m));

    // hold the takeoff timer at "now" and capture takeoff altitude until
    // the pilot commands throttle up while still landed
    if (!throttle_up && land_complete) {
        _state.takeoff_time_ms = tnow_ms;
        _state.takeoff_alt_m = -pos_d_m;
    }

    // GNDEFF_TMO is a minimum hold time before the altitude check is
    // allowed to release; the 5s hard timeout still applies unconditionally.
    const float height_above_takeoff_m = -pos_d_m - _state.takeoff_alt_m;
    const uint32_t min_hold_ms = MIN(uint32_t(_timeout_s * 1000.0f), AP_GROUNDEFFECT_TAKEOFF_MAX_MS);
    const bool above_alt = height_above_takeoff_m > _alt_m;
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

    // when GNDEFF_ALT is zero the touchdown altitude gate is disabled,
    // preserving the legacy "expect touchdown any time the descent looks
    // gentle" behaviour
    const bool near_ground = is_positive(_alt_m) ? (height_above_takeoff_m < _alt_m) : true;

    _state.touchdown_expected = slow_horizontal && slow_descent && near_ground;

    ahrs.set_takeoff_expected(_state.takeoff_expected);
    ahrs.set_touchdown_expected(_state.touchdown_expected);
}

#endif  // AP_GROUNDEFFECT_ENABLED
