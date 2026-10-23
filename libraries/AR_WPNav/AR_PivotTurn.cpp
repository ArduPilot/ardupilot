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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AR_PivotTurn.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define AR_PIVOT_TIMEOUT_MS     100 // pivot controller timesout and reset target if not called within this many milliseconds
#define AR_PIVOT_ANGLE_DEFAULT  0   // default PIVOT_ANGLE parameter value (disabled by default)
#define AR_PIVOT_ANGLE_ACCURACY 5   // vehicle will pivot to within this many degrees of destination
#define AR_PIVOT_RATE_DEFAULT   60  // default PIVOT_RATE parameter value
#define AR_PIVOT_DELAY_DEFAULT  0   // default PIVOT_DELAY parameter value

const AP_Param::GroupInfo AR_PivotTurn::var_info[] = {

    // @Param: ANGLE
    // @DisplayName: Pivot Angle
    // @Description: Pivot when the difference between the vehicle's heading and its target heading is more than this many degrees. Set to zero to disable pivot turns.  This parameter should be greater than 5 degrees for pivot turns to work.
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE", 1, AR_PivotTurn, _angle, AR_PIVOT_ANGLE_DEFAULT),

    // @Param: RATE
    // @DisplayName: Pivot Turn Rate
    // @Description: Turn rate during pivot turns
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE", 2, AR_PivotTurn, _rate_max, AR_PIVOT_RATE_DEFAULT),

    // @Param: DELAY
    // @DisplayName: Pivot Delay
    // @Description: Vehicle waits this many seconds after completing a pivot turn before proceeding
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DELAY", 3, AR_PivotTurn, _delay, AR_PIVOT_DELAY_DEFAULT),

    AP_GROUPEND
};

AR_PivotTurn::AR_PivotTurn(AR_AttitudeControl& atc) :
    _atc(atc)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// enable or disable pivot controller
void AR_PivotTurn::enable(bool enable_pivot)
{
    _enabled = enable_pivot;
    if (!_enabled) {
        _active = false;
    }
}

// true if update has been called recently
bool AR_PivotTurn::active() const
{
    return _enabled && _active;
}

// checks if pivot turns should be activated or deactivated
// force_active should be true if the caller wishes to trigger the start of a pivot turn regardless of the heading error
void AR_PivotTurn::check_activation(float desired_heading_deg, bool force_active)
{
    // check cases where we clearly cannot use pivot steering
    if (!_enabled || (_angle <= AR_PIVOT_ANGLE_ACCURACY)) {
        _active = false;
        return;
    }

    // calc yaw error in degrees
    const float yaw_error = fabsf(wrap_180(desired_heading_deg - (AP::ahrs().get_yaw_deg())));

    // if error is larger than _pivot_angle start pivot steering
    if (yaw_error > _angle || force_active) {
        _active = true;
        _delay_start_ms = 0;
        return;
    }

    uint32_t now_ms = AP_HAL::millis();

    // if within 5 degrees of the target heading, set start time of pivot steering
    if (_active && (yaw_error < AR_PIVOT_ANGLE_ACCURACY) && (_delay_start_ms == 0)) {
        _delay_start_ms = now_ms;
    }

    // exit pivot steering after the time set by pivot_delay has elapsed
    if ((_delay_start_ms > 0) &&
        (now_ms - _delay_start_ms) >= get_delay_duration_ms()) {
        _active = false;
        _delay_start_ms = 0;
    }
}

// check if pivot turn would be activated given an expected change in yaw in degrees
// note this does not actually active the pivot turn.  To activate use the check_activation method
bool AR_PivotTurn::would_activate(float yaw_change_deg) const
{
    // check cases where we clearly cannot use pivot steering
    if (!_enabled || (_angle <= AR_PIVOT_ANGLE_ACCURACY)) {
        return false;
    }

    // return true if yaw change is larger than _pivot_angle
    return fabsf(wrap_180(yaw_change_deg)) > _angle;
}

// get turn rate (in rad/sec)
// desired heading should be the heading towards the next waypoint in degrees
// dt should be the time since the last call in seconds
float AR_PivotTurn::get_turn_rate_rads(float desired_heading_deg, float dt)
{
    // handle pivot turns
    const float desired_turn_rate_rads = _atc.get_turn_rate_from_heading(radians(desired_heading_deg), radians(_rate_max));

    // update flag so that it can be cleared
    check_activation(desired_heading_deg);

    return desired_turn_rate_rads;
}

// return post-turn delay duration in milliseconds
uint32_t AR_PivotTurn::get_delay_duration_ms() const
{
    return constrain_float(_delay.get(), 0.0f, 60.0f) * 1000;
}
