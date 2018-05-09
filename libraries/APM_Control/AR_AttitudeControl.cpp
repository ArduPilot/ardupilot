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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_AttitudeControl.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AR_AttitudeControl::var_info[] = {

    // @Param: _STR_RAT_P
    // @DisplayName: Steering control rate P gain
    // @Description: Steering control rate P gain.  Converts the turn rate error (in radians/sec) to a steering control output (in the range -1 to +1)
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _STR_RAT_I
    // @DisplayName: Steering control I gain
    // @Description: Steering control I gain.  Corrects long term error between the desired turn rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _STR_RAT_IMAX
    // @DisplayName: Steering control I gain maximum
    // @Description: Steering control I gain maximum.  Constraings the steering output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _STR_RAT_D
    // @DisplayName: Steering control D gain
    // @Description: Steering control D gain.  Compensates for short-term change in desired turn rate vs actual
    // @Range: 0.000 0.400
    // @Increment: 0.001
    // @User: Standard

    // @Param: _STR_RAT_FF
    // @DisplayName: Steering control feed forward
    // @Description: Steering control feed forward
    // @Range: 0.000 3.000
    // @Increment: 0.001
    // @User: Standard

    // @Param: _STR_RAT_FILT
    // @DisplayName: Steering control filter frequency
    // @Description: Steering control input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_steer_rate_pid, "_STR_RAT_", 1, AR_AttitudeControl, AC_PID),

    // @Param: _SPEED_P
    // @DisplayName: Speed control P gain
    // @Description: Speed control P gain.  Converts the error between the desired speed (in m/s) and actual speed to a motor output (in the range -1 to +1)
    // @Range: 0.010 2.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SPEED_I
    // @DisplayName: Speed control I gain
    // @Description: Speed control I gain.  Corrects long term error between the desired speed (in m/s) and actual speed
    // @Range: 0.000 2.000
    // @User: Standard

    // @Param: _SPEED_IMAX
    // @DisplayName: Speed control I gain maximum
    // @Description: Speed control I gain maximum.  Constraings the maximum motor output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: Standard

    // @Param: _SPEED_D
    // @DisplayName: Speed control D gain
    // @Description: Speed control D gain.  Compensates for short-term change in desired speed vs actual
    // @Range: 0.000 0.400
    // @Increment: 0.001
    // @User: Standard

    // @Param: _SPEED_FF
    // @DisplayName: Speed control feed forward
    // @Description: Speed control feed forward
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: Standard

    // @Param: _SPEED_FILT
    // @DisplayName: Speed control filter frequency
    // @Description: Speed control input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_throttle_speed_pid, "_SPEED_", 2, AR_AttitudeControl, AC_PID),

    // @Param: _ACCEL_MAX
    // @DisplayName: Speed control acceleration (and deceleration) maximum in m/s/s
    // @Description: Speed control acceleration (and deceleration) maximum in m/s/s.  0 to disable acceleration limiting
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @Units: m/s/s
    // @User: Standard
    AP_GROUPINFO("_ACCEL_MAX", 3, AR_AttitudeControl, _throttle_accel_max, AR_ATTCONTROL_THR_ACCEL_MAX),

    // @Param: _BRAKE
    // @DisplayName: Speed control brake enable/disable
    // @Description: Speed control brake enable/disable. Allows sending a reversed output to the motors to slow the vehicle.
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("_BRAKE", 4, AR_AttitudeControl, _brake_enable, 0),

    // @Param: _STOP_SPEED
    // @DisplayName: Speed control stop speed
    // @Description: Speed control stop speed.  Motor outputs to zero once vehicle speed falls below this value
    // @Range: 0.00 0.50
    // @Increment: 0.01
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_STOP_SPEED", 5, AR_AttitudeControl, _stop_speed, AR_ATTCONTROL_STOP_SPEED_DEFAULT),

    // @Param: _STR_ANG_P
    // @DisplayName: Steering control angle P gain
    // @Description: Steering control angle P gain.  Converts the error between the desired heading/yaw (in radians) and actual heading/yaw to a desired turn rate (in rad/sec)
    // @Range: 1.000 10.000
    // @Increment: 0.1
    // @User: Standard
    AP_SUBGROUPINFO(_steer_angle_p, "_STR_ANG_", 6, AR_AttitudeControl, AC_P),

    // @Param: _STR_ACC_MAX
    // @DisplayName: Steering control angular acceleration maximum
    // @Description: Steering control angular acceleartion maximum (in deg/s/s).  0 to disable acceleration limiting
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s/s
    // @User: Standard
    AP_GROUPINFO("_STR_ACC_MAX", 7, AR_AttitudeControl, _steer_accel_max, AR_ATTCONTROL_STEER_ACCEL_MAX),

    // @Param: _STR_RAT_MAX
    // @DisplayName: Steering control rotation rate maximum
    // @Description: Steering control rotation rate maximum in deg/s.  0 to remove rate limiting
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("_STR_RAT_MAX", 8, AR_AttitudeControl, _steer_rate_max, AR_ATTCONTROL_STEER_RATE_MAX),

    AP_GROUPEND
};

AR_AttitudeControl::AR_AttitudeControl(AP_AHRS &ahrs) :
    _ahrs(ahrs),
    _steer_angle_p(AR_ATTCONTROL_STEER_ANG_P),
    _steer_rate_pid(AR_ATTCONTROL_STEER_RATE_P, AR_ATTCONTROL_STEER_RATE_I, AR_ATTCONTROL_STEER_RATE_D, AR_ATTCONTROL_STEER_RATE_IMAX, AR_ATTCONTROL_STEER_RATE_FILT, AR_ATTCONTROL_DT, AR_ATTCONTROL_STEER_RATE_FF),
    _throttle_speed_pid(AR_ATTCONTROL_THR_SPEED_P, AR_ATTCONTROL_THR_SPEED_I, AR_ATTCONTROL_THR_SPEED_D, AR_ATTCONTROL_THR_SPEED_IMAX, AR_ATTCONTROL_THR_SPEED_FILT, AR_ATTCONTROL_DT)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// return a steering servo output from -1.0 to +1.0 given a desired lateral acceleration rate in m/s/s.
// positive lateral acceleration is to the right.
float AR_AttitudeControl::get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right)
{
    // record desired accel for reporting purposes
    _steer_lat_accel_last_ms = AP_HAL::millis();
    _desired_lat_accel = desired_accel;

    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return 0.0f;
    }

    // only use positive speed. Use reverse flag instead of negative speeds.
    speed = fabsf(speed);

    // enforce minimum speed to stop oscillations when first starting to move
    if (speed < AR_ATTCONTROL_STEER_SPEED_MIN) {
        speed = AR_ATTCONTROL_STEER_SPEED_MIN;
    }

    // Calculate the desired steering rate given desired_accel and speed
    float desired_rate = desired_accel / speed;

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right);
}

// return a steering servo output from -1 to +1 given a heading in radians
float AR_AttitudeControl::get_steering_out_heading(float heading_rad, bool motor_limit_left, bool motor_limit_right)
{
    // calculate heading error (in radians)
    const float yaw_error = wrap_PI(heading_rad - _ahrs.yaw);

    // Calculate the desired turn rate (in radians) from the angle error (also in radians)
    float desired_rate = _steer_angle_p.get_p(yaw_error);

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right);
}

// return a steering servo output from -1 to +1 given a
// desired yaw rate in radians/sec. Positive yaw is to the right.
float AR_AttitudeControl::get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right)
{
    // calculate dt
    const uint32_t now = AP_HAL::millis();
    float dt = (now - _steer_turn_last_ms) / 1000.0f;
    if ((_steer_turn_last_ms == 0) || (dt > (AR_ATTCONTROL_TIMEOUT_MS / 1000.0f))) {
        dt = 0.0f;
        _steer_rate_pid.reset_filter();
        // reset desired turn rate to actual turn rate for accel limiting
        _desired_turn_rate = _ahrs.get_yaw_rate_earth();
    } else {
        _steer_rate_pid.set_dt(dt);
    }
    _steer_turn_last_ms = now;

    // acceleration limit desired turn rate
    const float change_max = radians(_steer_accel_max) * dt;
    if (is_positive(dt) && is_positive(change_max)) {
        desired_rate = constrain_float(desired_rate, _desired_turn_rate - change_max, _desired_turn_rate + change_max);
    }
    _desired_turn_rate = desired_rate;

    // rate limit desired turn rate
    if (is_positive(_steer_rate_max)) {
        _desired_turn_rate = constrain_float(_desired_turn_rate, -_steer_rate_max, _steer_rate_max);
    }

    // Calculate the steering rate error (rad/sec)
    // We do this in earth frame to allow for rover leaning over in hard corners
    const float rate_error = (desired_rate - _ahrs.get_yaw_rate_earth());

    // record desired rate for logging purposes only
    _steer_rate_pid.set_desired_rate(desired_rate);

    // pass error to PID controller
    _steer_rate_pid.set_input_filter_all(rate_error);

    // get feed-forward
    const float ff = _steer_rate_pid.get_ff(desired_rate);

    // get p
    const float p = _steer_rate_pid.get_p();

    // get i unless non-skid-steering rover at low speed or steering output has hit a limit
    float i = _steer_rate_pid.get_integrator();
    if ((is_negative(rate_error) && !motor_limit_left) || (is_positive(rate_error) && !motor_limit_right)) {
        i = _steer_rate_pid.get_i();
    }

    // get d
    const float d = _steer_rate_pid.get_d();

    // constrain and return final output
    return constrain_float(ff + p + i + d, -1.0f, 1.0f);
}

// get latest desired turn rate in rad/sec (recorded during calls to get_steering_out_rate)
float AR_AttitudeControl::get_desired_turn_rate() const
{
    // return zero if no recent calls to turn rate controller
    if ((_steer_turn_last_ms == 0) || ((AP_HAL::millis() - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_turn_rate;
}

// get latest desired lateral acceleration in m/s/s (recorded during calls to get_steering_out_lat_accel)
float AR_AttitudeControl::get_desired_lat_accel() const
{
    // return zero if no recent calls to lateral acceleration controller
    if ((_steer_lat_accel_last_ms == 0) || ((AP_HAL::millis() - _steer_lat_accel_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_lat_accel;
}

// get actual lateral acceleration in m/s/s.  returns true on success
bool AR_AttitudeControl::get_lat_accel(float &lat_accel) const
{
    float speed;
    if (!get_forward_speed(speed)) {
        return false;
    }
    lat_accel = speed * _ahrs.get_yaw_rate_earth();
    return true;
}

// return a throttle output from -1 to +1 given a desired speed in m/s (use negative speeds to travel backwards)
//   motor_limit should be true if motors have hit their upper or lower limits
//   cruise speed should be in m/s, cruise throttle should be a number from -1 to +1
float AR_AttitudeControl::get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle)
{
    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // we expect caller will not try to control heading using rate control without a valid speed estimate
        // on failure to get speed we do not attempt to steer
        return 0.0f;
    }

    // calculate dt
    const uint32_t now = AP_HAL::millis();
    float dt = (now - _speed_last_ms) / 1000.0f;
    if ((_speed_last_ms == 0) || (dt > (AR_ATTCONTROL_TIMEOUT_MS / 1000.0f))) {
        dt = 0.0f;
        _throttle_speed_pid.reset_filter();
    } else {
        _throttle_speed_pid.set_dt(dt);
    }
    _speed_last_ms = now;

    // record desired speed for next iteration
    _desired_speed = desired_speed;

    // calculate speed error and pass to PID controller
    const float speed_error = desired_speed - speed;
    _throttle_speed_pid.set_input_filter_all(speed_error);

    // record desired speed for logging purposes only
    _throttle_speed_pid.set_desired_rate(desired_speed);

    // get feed-forward
    const float ff = _throttle_speed_pid.get_ff(desired_speed);

    // get p
    const float p = _throttle_speed_pid.get_p();

    // get i unless moving at low speed or motors have hit a limit
    float i = _throttle_speed_pid.get_integrator();
    if ((is_negative(speed_error) && !motor_limit_low && !_throttle_limit_low) || (is_positive(speed_error) && !motor_limit_high && !_throttle_limit_high)) {
        i = _throttle_speed_pid.get_i();
    }

    // get d
    const float d = _throttle_speed_pid.get_d();

    // calculate base throttle (protect against divide by zero)
    float throttle_base = 0.0f;
    if (is_positive(cruise_speed) && is_positive(cruise_throttle)) {
        throttle_base = desired_speed * (cruise_throttle / cruise_speed);
    }

    // calculate final output
    float throttle_out = (ff+p+i+d+throttle_base);

    // clear local limit flags used to stop i-term build-up as we stop reversed outputs going to motors
    _throttle_limit_low = false;
    _throttle_limit_high = false;

    // protect against reverse output being sent to the motors unless braking has been enabled
    if (!_brake_enable) {
        // if both desired speed and actual speed are positive, do not allow negative values
        if ((desired_speed >= 0.0f) && (throttle_out <= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_low = true;
        }
        if ((desired_speed <= 0.0f) && (throttle_out >= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_high = true;
        }
    }

    // final output throttle in range -1 to 1
    return throttle_out;
}

// return a throttle output from -1 to +1 to perform a controlled stop.  returns true once the vehicle has stopped
float AR_AttitudeControl::get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, bool &stopped)
{
    // get current system time
    const uint32_t now = AP_HAL::millis();

    // if we were stopped in the last 300ms, assume we are still stopped
    bool _stopped = (_stop_last_ms != 0) && (now - _stop_last_ms) < 300;

    // get speed forward
    float speed;
    if (!get_forward_speed(speed)) {
        // could not get speed so assume stopped
        _stopped = true;
    } else {
        // if desired speed is zero and vehicle drops below _stop_speed consider it stopped
        if (is_zero(_desired_speed) && fabsf(speed) <= fabsf(_stop_speed)) {
            _stopped = true;
        }
    }

    // set stopped status for caller
    stopped = _stopped;

    // if stopped return zero
    if (stopped) {
        // update last time we thought we were stopped
        _stop_last_ms = now;
        return 0.0f;
    } else {
        // clear stopped system time
        _stop_last_ms = 0;
        // run speed controller to bring vehicle to stop
        return get_throttle_out_speed(0.0f, motor_limit_low, motor_limit_high, cruise_speed, cruise_throttle);
    }
}

// get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
bool AR_AttitudeControl::get_forward_speed(float &speed) const
{
    Vector3f velocity;
    if (!_ahrs.get_velocity_NED(velocity)) {
        // use less accurate GPS, assuming entire length is along forward/back axis of vehicle
        if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            if (labs(wrap_180_cd(_ahrs.yaw_sensor - AP::gps().ground_course_cd())) <= 9000) {
                speed = AP::gps().ground_speed();
            } else {
                speed = -AP::gps().ground_speed();
            }
            return true;
        } else {
            return false;
        }
    }
    // calculate forward speed velocity into body frame
    speed = velocity.x*_ahrs.cos_yaw() + velocity.y*_ahrs.sin_yaw();
    return true;
}

// get latest desired speed recorded during call to get_throttle_out_speed.  For reporting purposes only
float AR_AttitudeControl::get_desired_speed() const
{
    // return zero if no recent calls to speed controller
    if ((_speed_last_ms == 0) || ((AP_HAL::millis() - _speed_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_speed;
}

// get acceleration limited desired speed
float AR_AttitudeControl::get_desired_speed_accel_limited(float desired_speed) const
{
    // return input value if no recent calls to speed controller
    const uint32_t now = AP_HAL::millis();
    if ((_speed_last_ms == 0) || ((now - _speed_last_ms) > AR_ATTCONTROL_TIMEOUT_MS) || !is_positive(_throttle_accel_max)) {
        return desired_speed;
    }

    // calculate dt
    const float dt = (now - _speed_last_ms) / 1000.0f;

    // acceleration limit desired speed
    const float speed_change_max = _throttle_accel_max * dt;
    return constrain_float(desired_speed, _desired_speed - speed_change_max, _desired_speed + speed_change_max);
}

// get minimum stopping distance (in meters) given a speed (in m/s)
float AR_AttitudeControl::get_stopping_distance(float speed)
{
    // get maximum vehicle deceleration
    const float accel_max = get_accel_max();

    // avoid divide by zero
    if ((accel_max <= 0.0f) || is_zero(speed)) {
        return 0.0f;
    }

    // assume linear deceleration
    return 0.5f * sq(speed) / accel_max;
}
