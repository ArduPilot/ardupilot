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

//	Code by Sebastian Quilter

#include <AP_HAL/AP_HAL.h>
#include "AP_GroundEffectController.h"
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

#if HAL_GROUND_EFFECT_ENABLED

constexpr uint32_t RESET_TIMEOUT_MICROS{1000000};

const AP_Param::GroupInfo GroundEffectController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Is the ground effect controller available or not
    // @Description: Toggles the ground effect controller availability on and off
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, GroundEffectController, _ACTIVE, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: P
    // @DisplayName: P gain
    // @Description: P gain. A 1 meter error from desired alt changes throttle by this many percent.
    // @Range: 0.0 200.0
    // @User: Standard

    // @Param: I
    // @DisplayName: I gain
    // @Description: I gain
    // @User: Standard

    // @Param: D
    // @DisplayName: D gain
    // @Description: D gain
    // @User: Standard

    // @Param: IMAX
    // @DisplayName: IMax
    // @Description: Maximum integrator value
    // @User: Standard
    AP_SUBGROUPINFO(_throttle_pid, "_THR_", 2, GroundEffectController, PID),

    // @Param: P
    // @DisplayName: P gain
    // @Description: P gain. A 1 meter error from desired alt changes sets the vehicle pitch to this value.
    // @User: Standard

    // @Param: I
    // @DisplayName: I gain
    // @Description: I gain
    // @User: Standard

    // @Param: D
    // @DisplayName: D gain
    // @Description: D gain
    // @User: Standard

    // @Param: IMAX
    // @DisplayName: IMax
    // @Description: Maximum integrator value
    // @User: Standard
    AP_SUBGROUPINFO(_pitch_pid, "_PITCH_", 3, GroundEffectController, PID),

	// @Param: THR_REF
	// @DisplayName: Ground Effect desired throttle (percentage)
	// @Description:
	// @Range: 0.0 100.0
	// @Increment: 0.01
    // @User: Standard
	AP_GROUPINFO("_THR_REF",   4, GroundEffectController, _THR_REF,   35.0),

	// @Param: THR_MIN
	// @DisplayName: Ground Effect minimum throttle (percentage)
	// @Description:
	// @Range: 0.0 100.0
	// @Increment: 0.01
    // @User: Standard
	AP_GROUPINFO("_THR_MIN",   5, GroundEffectController, _THR_MIN,   20.0),

    // @Param: THR_MAX
	// @DisplayName: Ground Effect maximum throttle (percentage)
	// @Description:
	// @Range: 0.0 100.0
	// @Increment: 0.01
    // @User: Standard
	AP_GROUPINFO("_THR_MAX",   6, GroundEffectController, _THR_MAX,   50.0),

	// @Param: ALT_REF
	// @DisplayName: Ground Effect desired altitude (meters)
	// @Description:
	// @Range: 0.0 1.0
	// @Increment: 0.01
    // @User: Standard
	AP_GROUPINFO("_ALT_REF",   7, GroundEffectController, _ALT_REF,   0.45),

	// @Param: CUTOFF_FREQ
	// @DisplayName: Rangefinder Complementary Filter Cutoff Frequency
	// @Description: Lower values will trust the rangefinder less.
	// @Range: 0.0 2.0
	// @Increment: 0.01
    // @User: Advanced
	AP_GROUPINFO("_CUTOFF_FRQ",   8, GroundEffectController, _CUTOFF_FREQ,   0.1),

	// @Param: LIM_ROLL
	// @DisplayName: Max roll angle (degrees)
	// @Description: Max roll allowed in auto mode while going towards a ground effect waypoint. 0 to disable.
	// @Range: 0.0 45.0
	// @Increment: 0.01
    // @User: Advanced
	AP_GROUPINFO("_LIM_ROLL",   9, GroundEffectController, _LIM_ROLL,   10.0),

    AP_GROUPEND
};

bool GroundEffectController::user_request_enable(bool enable)
{
    if(enable){
        if(!_ACTIVE || !_rangefinder->has_orientation(ROTATION_PITCH_270)){
            _enabled = false;
            return false;
        }
    }

    _enabled = enable;
    return true;
}

void GroundEffectController::reset()
{
    _altFilter.set_cutoff_frequency(_CUTOFF_FREQ);
    _altFilter.reset();

    _pitch_pid.reset_I();
    _throttle_pid.reset_I();
    return;
}

int32_t GroundEffectController::get_auto_lim_roll_cd()
{
    if(_LIM_ROLL <= 0.0001f){
        return INT32_MAX;
    }
    return int32_t(_LIM_ROLL*100.0);
}

void GroundEffectController::update()
{
    uint32_t time = AP_HAL::micros();
    if(time - _last_time_called > RESET_TIMEOUT_MICROS){
        reset();
    }
    _last_time_called = time;

    if(_rangefinder->status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) {
        _last_good_rangefinder_reading = _rangefinder->distance_orient(ROTATION_PITCH_270);
    }

    // DCM altitude is not good. If EKF alt is not available, just use raw rangefinder data
    float alt_error, ahrs_negative_alt;
    if(_ahrs->get_active_AHRS_type() > 0 && _ahrs->get_relative_position_D_origin(ahrs_negative_alt)){
        _altFilter.apply(_last_good_rangefinder_reading, -ahrs_negative_alt, time);
        alt_error = _ALT_REF - _altFilter.get();
    } else {
        alt_error = _ALT_REF - _last_good_rangefinder_reading;
    }

    _pitch = _pitch_pid.get_pid(alt_error);
    _throttle = _throttle_pid.get_pid(alt_error) + _THR_REF;
    _throttle = constrain_int16(_throttle, _THR_MIN, _THR_MAX);

    return;
}

#endif // HAL_GROUND_EFFECT_ENABLED
