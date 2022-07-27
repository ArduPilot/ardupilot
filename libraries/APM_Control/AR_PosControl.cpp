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

#include "AR_PosControl.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_Avoidance/AC_Avoid.h>

extern const AP_HAL::HAL& hal;

#define AR_POSCON_TIMEOUT_MS            100     // timeout after 0.1 sec
#define AR_POSCON_POS_P                 0.2f    // default position P gain
#define AR_POSCON_VEL_P                 1.0f    // default velocity P gain
#define AR_POSCON_VEL_I                 0.0f    // default velocity I gain
#define AR_POSCON_VEL_D                 0.0f    // default velocity D gain
#define AR_POSCON_VEL_FF                0.0f    // default velocity FF gain
#define AR_POSCON_VEL_IMAX              1.0f    // default velocity IMAX
#define AR_POSCON_VEL_FILT              5.0f    // default velocity filter
#define AR_POSCON_VEL_FILT_D            5.0f    // default velocity D term filter
#define AR_POSCON_DT                    0.02f   // default dt for PID controllers

const AP_Param::GroupInfo AR_PosControl::var_info[] = {

    // @Param: _POS_P
    // @DisplayName: Position controller P gain
    // @Description: Position controller P gain.  Converts the distance to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 1, AR_PosControl, AC_P_2D),

    // @Param: _VEL_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VEL_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VEL_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VEL_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VEL_FLTE
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VEL_FLTD
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VEL_FF
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel, "_VEL_", 2, AR_PosControl, AC_PID_2D),

    AP_GROUPEND
};

AR_PosControl::AR_PosControl(AR_AttitudeControl& atc) :
    _atc(atc),
    _p_pos(AR_POSCON_POS_P, AR_POSCON_DT),
    _pid_vel(AR_POSCON_VEL_P, AR_POSCON_VEL_I, AR_POSCON_VEL_D, AR_POSCON_VEL_FF, AR_POSCON_VEL_IMAX, AR_POSCON_VEL_FILT, AR_POSCON_VEL_FILT_D, AR_POSCON_DT)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update navigation
void AR_PosControl::update(float dt)
{
    // exit immediately if no current location, destination or disarmed
    Vector2f curr_pos_NE;
    Vector3f curr_vel_NED;
    if (!hal.util->get_soft_armed() || !AP::ahrs().get_relative_position_NE_origin(curr_pos_NE) ||
        !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        _desired_speed = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        return;
    }

    // check for ekf xy position reset
    handle_ekf_xy_reset();

    // if no recent calls reset velocity controller
    if (!is_active()) {
        _pid_vel.reset_I();
        _pid_vel.reset_filter();
    }
    _last_update_ms = AP_HAL::millis();

    // update P, PID object's dt
    _p_pos.set_dt(dt);
    _pid_vel.set_dt(dt);

    // calculate position error and convert to desired velocity
    _vel_target.zero();
    if (_pos_target_valid) {
        Vector2p pos_target = _pos_target;
        _vel_target = _p_pos.update_all(pos_target.x, pos_target.y, curr_pos_NE);
    }

    // calculation velocity error
    if (_vel_desired_valid) {
        // add target velocity to desired velocity from position error
        _vel_target += _vel_desired;
    }

    // limit velocity to maximum speed
    _vel_target.limit_length(get_speed_max());

    // Limit the velocity to prevent fence violations
    bool backing_up = false;
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid != nullptr) {
        Vector3f vel_3d_cms{_vel_target.x * 100.0f, _vel_target.y * 100.0f, 0.0f};
        const float accel_max_cmss = MIN(_accel_max, _lat_accel_max) * 100.0;
        avoid->adjust_velocity(vel_3d_cms, backing_up, _p_pos.kP(), accel_max_cmss, _p_pos.kP(), accel_max_cmss, dt);
        _vel_target.x = vel_3d_cms.x * 0.01;
        _vel_target.y = vel_3d_cms.y * 0.01;
    }

    // calculate desired acceleration
    // To-Do: fixup _limit_vel used below
    _accel_target = _pid_vel.update_all(_vel_target, curr_vel_NED.xy(), _limit_vel);
    if (_accel_desired_valid) {
        _accel_target += _accel_desired;
    }

    // convert desired acceleration to desired forward-back speed, desired lateral speed and desired turn rate

    // rotate acceleration into body frame using current heading
    const Vector2f accel_target_FR = AP::ahrs().earth_to_body2D(_accel_target);

    // calculate minimum turn speed which is the max speed the vehicle could turn through the corner
    // given the vehicle's turn radius and half its max lateral acceleration
    // todo: remove MAX of zero when safe_sqrt fixed
    float turn_speed_min = MAX(safe_sqrt(_atc.get_turn_lat_accel_max() * 0.5 * _turn_radius), 0);

    // rotate target velocity from earth-frame to body frame
    const Vector2f vel_target_FR = AP::ahrs().earth_to_body2D(_vel_target);

    // desired speed is normally the forward component (only) of the target velocity
    // but we do not let it fall below the minimum turn speed unless the vehicle is slowing down
    const float abs_des_speed_min = MIN(_vel_target.length(), turn_speed_min);
    float des_speed;
    if (_reversed != backing_up) {
        // if reversed or backing up desired speed will be negative
        des_speed = MIN(-abs_des_speed_min, vel_target_FR.x);
    } else {
        des_speed = MAX(abs_des_speed_min, vel_target_FR.x);
    }
    _desired_speed = _atc.get_desired_speed_accel_limited(des_speed, dt);

    // calculate turn rate from desired lateral acceleration
    _desired_lat_accel = accel_target_FR.y;
    _desired_turn_rate_rads = _atc.get_turn_rate_from_lat_accel(_desired_lat_accel, _desired_speed);
}

// true if update has been called recently
bool AR_PosControl::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_POSCON_TIMEOUT_MS);
}

// set limits
void AR_PosControl::set_limits(float speed_max, float accel_max, float lat_accel_max, float jerk_max)
{
    _speed_max = MAX(speed_max, 0);
    _accel_max = MAX(accel_max, 0);
    _lat_accel_max = MAX(lat_accel_max, 0);
    _jerk_max = MAX(jerk_max, 0);

    // set position P controller limits
    _p_pos.set_limits(_speed_max, MIN(_accel_max, _lat_accel_max), _jerk_max);
}

// setter to allow vehicle code to provide turn related param values to this library (should be updated regularly)
void AR_PosControl::set_turn_params(float turn_radius, bool pivot_possible)
{
    if (pivot_possible) {
        _turn_radius = 0;
    } else {
        _turn_radius = turn_radius;
    }
}

// initialise the position controller to the current position, velocity, acceleration and attitude
// this should be called before the input shaping methods are used
bool AR_PosControl::init()
{
    // get current position and velocity from AHRS
    Vector2f pos_NE;
    Vector3f vel_NED;
    if (!AP::ahrs().get_relative_position_NE_origin(pos_NE) || !AP::ahrs().get_velocity_NED(vel_NED)) {
        return false;
    }

    // set target position to current position
    _pos_target.x = pos_NE.x;
    _pos_target.y = pos_NE.y;

    // set target velocity and acceleration
    _vel_desired = vel_NED.xy();
    _vel_target.zero();
    _accel_desired = AP::ahrs().get_accel_ef().xy();
    _accel_target.zero();

    // clear reversed setting
    _reversed = false;

    // initialise ekf xy reset handler
    init_ekf_xy_reset();

    return true;
}

// methods to adjust position, velocity and acceleration targets smoothly using input shaping
// pos should be the target position as an offset from the EKF origin (in meters)
// dt should be the update rate in seconds
void AR_PosControl::input_pos_target(const Vector2p &pos, float dt)
{
    // adjust target position, velocity and acceleration forward by dt
    update_pos_vel_accel_xy(_pos_target, _vel_desired, _accel_desired, dt, Vector2f(), Vector2f(), Vector2f());

    // call shape_pos_vel_accel_xy to pull target towards final destination
    Vector2f vel;
    Vector2f accel;
    const float accel_max = MIN(_accel_max, _lat_accel_max);
    shape_pos_vel_accel_xy(pos, vel, accel, _pos_target, _vel_desired, _accel_desired,
                           _speed_max, accel_max, _jerk_max, dt, false);

    // set flags so update will consume target position, desired velocity and desired acceleration
    _pos_target_valid = true;
    _vel_desired_valid = true;
    _accel_desired_valid = true;
}

// set target position, desired velocity and acceleration.  These should be from an externally created path and are not "input shaped"
void AR_PosControl::set_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel)
{
    _pos_target = pos;
    _vel_desired = vel;
    _accel_desired = accel;
    _pos_target_valid = true;
    _vel_desired_valid = true;
    _accel_desired_valid = true;
}

// returns desired velocity vector (i.e. feed forward) in cm/s in lat and lon direction
Vector2f AR_PosControl::get_desired_velocity() const
{
    if (_vel_desired_valid) {
        return _vel_desired;
    }
    return Vector2f();
}

// return desired acceleration vector in m/s in lat and lon direction
Vector2f AR_PosControl::get_desired_accel() const
{
    if (_accel_desired_valid) {
        return _accel_desired;
    }
    return Vector2f();
}

/// get position error as a vector from the current position to the target position
Vector2p AR_PosControl::get_pos_error() const
{
    // return zero error is not active or no position estimate
    Vector2f curr_pos_NE;
    if (!is_active() ||!AP::ahrs().get_relative_position_NE_origin(curr_pos_NE)) {
        return Vector2p{};
    }

    // get current position
    return (_pos_target - curr_pos_NE.topostype());
}

// write PSC logs
void AR_PosControl::write_log()
{
    // exit immediately if not active
    if (!is_active()) {
        return;
    }

    // exit immediately if no position or velocity estimate
    Vector3f curr_pos_NED;
    Vector3f curr_vel_NED;
    if (!AP::ahrs().get_relative_position_NED_origin(curr_pos_NED) || !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        return;
    }

    // get acceleration
    const Vector3f curr_accel_NED = AP::ahrs().get_accel_ef() * 100.0;

    // convert position to required format
    Vector2f pos_target_2d_cm = get_pos_target().tofloat() * 100.0;

    AP::logger().Write_PSCN(pos_target_2d_cm.x,     // position target
                            curr_pos_NED.x * 100.0, // position
                            _vel_desired.x * 100.0, // desired velocity
                            _vel_target.x * 100.0,  // target velocity
                            curr_vel_NED.x * 100.0, // velocity
                            _accel_desired.x * 100.0,   // desired accel
                            _accel_target.x * 100.0,    // target accel
                            curr_accel_NED.x);      // accel
    AP::logger().Write_PSCE(pos_target_2d_cm.y,     // position target
                            curr_pos_NED.y * 100.0, // position
                            _vel_desired.y * 100.0, // desired velocity
                            _vel_target.y * 100.0,  // target velocity
                            curr_vel_NED.y * 100.0, // velocity
                            _accel_desired.y * 100.0,   // desired accel
                            _accel_target.y * 100.0,    // target accel
                            curr_accel_NED.y);      // accel
}

/// initialise ekf xy position reset check
void AR_PosControl::init_ekf_xy_reset()
{
    Vector2f pos_shift;
    _ekf_xy_reset_ms = AP::ahrs().getLastPosNorthEastReset(pos_shift);
}

/// handle_ekf_xy_reset - check for ekf position reset and adjust loiter or brake target position
void AR_PosControl::handle_ekf_xy_reset()
{
    // check for position shift
    Vector2f pos_shift;
    uint32_t reset_ms = AP::ahrs().getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_xy_reset_ms) {
        Vector2f pos_NE;
        if (!AP::ahrs().get_relative_position_NE_origin(pos_NE)) {
            return;
        }
        _pos_target = (pos_NE + _p_pos.get_error()).topostype();

        Vector3f vel_NED;
        if (!AP::ahrs().get_velocity_NED(vel_NED)) {
            return;
        }
        _vel_desired = vel_NED.xy() + _pid_vel.get_error();

        _ekf_xy_reset_ms = reset_ms;
    }
}
