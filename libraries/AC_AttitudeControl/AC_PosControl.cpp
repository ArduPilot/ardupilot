/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PosControl.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PosControl::var_info[] PROGMEM = {
    // @Param: THR_HOVER
    // @DisplayName: Throttle Hover
    // @Description: The autopilot's estimate of the throttle required to maintain a level hover.  Calculated automatically from the pilot's throttle input while in stabilize mode
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Advanced
    AP_GROUPINFO("THR_HOVER",       0, AC_PosControl, _throttle_hover, POSCONTROL_THROTTLE_HOVER),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(const AP_AHRS& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             APM_PI& pi_alt_pos, AC_PID& pid_alt_rate, AC_PID& pid_alt_accel,
                             APM_PI& pi_pos_lat, APM_PI& pi_pos_lon, AC_PID& pid_rate_lat, AC_PID& pid_rate_lon) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _pi_alt_pos(pi_alt_pos),
    _pid_alt_rate(pid_alt_rate),
    _pid_alt_accel(pid_alt_accel),
    _pi_pos_lat(pi_pos_lat),
    _pi_pos_lon(pi_pos_lon),
    _pid_rate_lat(pid_rate_lat),
    _pid_rate_lon(pid_rate_lon),
    _dt(POSCONTROL_DT_10HZ),
    _last_update_ms(0),
    _last_update_rate_ms(0),
    _last_update_accel_ms(0),
    _step(0),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_XY_MAX),   // To-Do: check this default
    _accel_cms(POSCONTROL_ACCEL_XY_MAX),   // To-Do: check this default
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _cos_yaw(1.0),
    _sin_yaw(0.0),
    _cos_pitch(1.0),
    _roll_target(0.0),
    _pitch_target(0.0),
    _vel_target_filt_z(0),
    _alt_max(0),
    _distance_to_target(0),
    _xy_step(0),
    _dt_xy(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.force_recalc_xy = false;
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
    _flags.slow_cpu = false;
#else
    _flags.slow_cpu = true;
#endif
}

///
/// z-axis position controller
///

// get_alt_error - returns altitude error in cm
float AC_PosControl::get_alt_error() const
{
    return (_pos_target.z - _inav.get_position().z);
}

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
void AC_PosControl::set_target_to_stopping_point_z()
{
    const Vector3f& curr_pos = _inav.get_position();
    const Vector3f& curr_vel = _inav.get_velocity();
    float linear_distance;  // half the distace we swap between linear and sqrt and the distace we offset sqrt
    float linear_velocity;  // the velocity we swap between linear and sqrt

    // calculate the velocity at which we switch from calculating the stopping point using a linear funcction to a sqrt function
    linear_velocity = POSCONTROL_ALT_HOLD_ACCEL_MAX/_pi_alt_pos.kP();

    if (fabs(curr_vel.z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        _pos_target.z = curr_pos.z + curr_vel.z/_pi_alt_pos.kP();
    } else {
        linear_distance = POSCONTROL_ALT_HOLD_ACCEL_MAX/(2.0f*_pi_alt_pos.kP()*_pi_alt_pos.kP());
        if (curr_vel.z > 0){
            _pos_target.z = curr_pos.z + (linear_distance + curr_vel.z*curr_vel.z/(2.0f*POSCONTROL_ALT_HOLD_ACCEL_MAX));
        } else {
            _pos_target.z = curr_pos.z - (linear_distance + curr_vel.z*curr_vel.z/(2.0f*POSCONTROL_ALT_HOLD_ACCEL_MAX));
        }
    }
    _pos_target.z = constrain_float(_pos_target.z, curr_pos.z - POSCONTROL_STOPPING_DIST_Z_MAX, curr_pos.z + POSCONTROL_STOPPING_DIST_Z_MAX);
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    const Vector3f& curr_pos = _inav.get_position();

    _pos_target.z = curr_pos.z + POSCONTROL_TAKEOFF_JUMP_CM;

    // clear i term from acceleration controller
    if (_pid_alt_accel.get_integrator() < 0) {
        _pid_alt_accel.reset_I();
    }
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // call position controller
    pos_to_rate_z();
}

/// climb_at_rate - climb at rate provided in cm/s
void AC_PosControl::climb_at_rate(const float rate_target_cms)
{
    const Vector3f& curr_pos = _inav.get_position();

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // adjust desired alt if motors have not hit their limits
    // To-Do: should we use some other limits?  this controller's vel limits?
    if ((rate_target_cms<0 && !_motors.limit.throttle_lower) || (rate_target_cms>0 && !_motors.limit.throttle_upper)) {
        _pos_target.z += rate_target_cms * _dt;
    }

    // do not let target altitude get too far from current altitude
    if (_pos_target.z < curr_pos.z - POSCONTROL_LEASH_Z) {
        _pos_target.z = curr_pos.z - POSCONTROL_LEASH_Z;
        _limit.pos_down = true;
    }
    if (_pos_target.z > curr_pos.z + POSCONTROL_LEASH_Z) {
        _pos_target.z = curr_pos.z + POSCONTROL_LEASH_Z;
        _limit.pos_up = true;
    }

    // do not let target alt get above limit
    if (_alt_max > 0 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _limit.pos_up = true;
    }

    // call position controller
    pos_to_rate_z();
}

// pos_to_rate_z - position to rate controller for Z axis
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void AC_PosControl::pos_to_rate_z()
{
    const Vector3f& curr_pos = _inav.get_position();
    float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt.

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_pos.z;

    // check kP to avoid division by zero
    if (_pi_alt_pos.kP() != 0) {
        linear_distance = POSCONTROL_ALT_HOLD_ACCEL_MAX/(2.0f*_pi_alt_pos.kP()*_pi_alt_pos.kP());
        if (_pos_error.z > 2*linear_distance ) {
            _vel_target.z = safe_sqrt(2.0f*POSCONTROL_ALT_HOLD_ACCEL_MAX*(_pos_error.z-linear_distance));
        }else if (_pos_error.z < -2.0f*linear_distance) {
            _vel_target.z = -safe_sqrt(2.0f*POSCONTROL_ALT_HOLD_ACCEL_MAX*(-_pos_error.z-linear_distance));
        }else{
            _vel_target.z = _pi_alt_pos.get_p(_pos_error.z);
        }
    }else{
        _vel_target.z = 0;
    }

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z(_vel_target.z);
}

// rate_to_accel_z - calculates desired accel required to achieve the velocity target
// calculates desired acceleration and calls accel throttle controller
void AC_PosControl::rate_to_accel_z(float vel_target_z)
{
    uint32_t now = hal.scheduler->millis();
    const Vector3f& curr_vel = _inav.get_velocity();
    float z_target_speed_delta;             // The change in requested speed
    float p;                                // used to capture pid values for logging
    float desired_accel;                    // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // reset velocity error and filter if this controller has just been engaged
    if (now - _last_update_rate_ms > 100 ) {
        // Reset Filter
        _vel_error.z = 0;
        _vel_target_filt_z = vel_target_z;
        desired_accel = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        //To-Do: adjust constant below based on update rate
        _vel_error.z = _vel_error.z + 0.20085f * ((vel_target_z - curr_vel.z) - _vel_error.z);
        // feed forward acceleration based on change in the filtered desired speed.
        z_target_speed_delta = 0.20085f * (vel_target_z - _vel_target_filt_z);
        _vel_target_filt_z = _vel_target_filt_z + z_target_speed_delta;
        desired_accel = z_target_speed_delta / _dt;
    }
    _last_update_rate_ms = now;

    // calculate p
    p = _pid_alt_rate.kP() * _vel_error.z;

    // consolidate and constrain target acceleration
    desired_accel += p;
    desired_accel = constrain_int32(desired_accel, -32000, 32000);

    // To-Do: re-enable PID logging?
    // TO-DO: ensure throttle cruise is updated some other way in the main code or attitude control

    // set target for accel based throttle controller
    accel_to_throttle(desired_accel);
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void AC_PosControl::accel_to_throttle(float accel_target_z)
{
    uint32_t now = hal.scheduler->millis();
    float z_accel_meas;         // actual acceleration
    int32_t p,i,d;              // used to capture pid values for logging

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f;

    // reset target altitude if this controller has just been engaged
    if (now - _last_update_accel_ms > 100) {
        // Reset Filter
        _accel_error.z = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        // To-Do: replace constant below with one that is adjusted for update rate
        _accel_error.z = _accel_error.z + 0.11164f * (constrain_float(accel_target_z - z_accel_meas, -32000, 32000) - _accel_error.z);
    }
    _last_update_accel_ms = now;

    // separately calculate p, i, d values for logging
    p = _pid_alt_accel.get_p(_accel_error.z);

    // get i term
    i = _pid_alt_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
        i = _pid_alt_accel.get_i(_accel_error.z, _dt);
    }

    // get d term
    d = _pid_alt_accel.get_d(_accel_error.z, _dt);

    // To-Do: pull min/max throttle from motors
    // To-Do: where to get hover throttle?
    // To-Do: we had a contraint here but it's now removed, is this ok?  with the motors library handle it ok?
    _attitude_control.set_throttle_out((int16_t)p+i+d+_throttle_hover, true);
    
    // to-do add back in PID logging?
}
/*
// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    float alt_change = target_alt-controller_desired_alt;
    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !motors.limit.throttle_lower) || (alt_change>0 && !motors.limit.throttle_upper)) {
        controller_desired_alt += constrain_float(alt_change, min_climb_rate*0.02f, max_climb_rate*0.02f);
    }

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    // adjust desired alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        controller_desired_alt += target_rate * 0.02f;
    }

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

#if AC_FENCE == ENABLED
    // do not let target altitude be too close to the fence
    // To-Do: add this to other altitude controllers
    if((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        float alt_limit = fence.get_safe_alt() * 100.0f;
        if (controller_desired_alt > alt_limit) {
            controller_desired_alt = alt_limit;
        }
    }
#endif

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}
*/

///
/// position controller
///

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _pos_target = position;

    // initialise roll and pitch to current roll and pitch.  This avoids a twitch between when the target is set and the pos controller is first run
    _roll_target = constrain_int32(_ahrs.roll_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
    _pitch_target = constrain_int32(_ahrs.pitch_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
}

/// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
	Vector3f curr_pos = _inav.get_position();
	Vector3f curr_vel = _inav.get_velocity();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;		// the distance within the vehicle can stop
    float kP = _pi_pos_lat.kP();

    // calculate current velocity
    float vel_total = safe_sqrt(curr_vel.x*curr_vel.x + curr_vel.y*curr_vel.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (vel_total < 10.0f || kP <= 0.0f || _accel_cms <= 0.0f) {
        stopping_point = curr_pos;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _accel_cms/kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
    	stopping_dist = vel_total/kP;
    } else {
        linear_distance = _accel_cms/(2.0f*kP*kP);
        stopping_dist = linear_distance + (vel_total*vel_total)/(2.0f*_accel_cms);
    }

    // constrain stopping distance
    stopping_dist = constrain_float(stopping_dist, 0, _leash);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_point.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_point.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AC_PosControl::get_distance_to_target() const
{
    return _distance_to_target;
}

/// update_pos_controller - run the horizontal position controller - should be called at 100hz or higher
void AC_PosControl::update_pos_controller(bool use_desired_velocity)
{
    // catch if we've just been started
    uint32_t now = hal.scheduler->millis();
    if ((now - _last_update_ms) >= 1000) {
        _last_update_ms = now;
        reset_I_xy();
        _xy_step = 0;
    }

    // reset step back to 0 if loiter or waypoint parents have triggered an update and we completed the last full cycle
    if (_flags.force_recalc_xy && _xy_step > 3) {
        _flags.force_recalc_xy = false;
        _xy_step = 0;
    }

    // run loiter steps
    switch (_xy_step) {
        case 0:
            // capture time since last iteration
            _dt_xy = (now - _last_update_ms) / 1000.0f;
            _last_update_ms = now;

            // translate any adjustments from pilot to loiter target
            desired_vel_to_pos(_dt_xy);
            _xy_step++;
            break;
        case 1:
            // run position controller's position error to desired velocity step
            pos_to_rate_xy(use_desired_velocity,_dt_xy);
            _xy_step++;
            break;
        case 2:
            // run position controller's velocity to acceleration step
            rate_to_accel_xy(_dt_xy);
            _xy_step++;
            break;
        case 3:
            // run position controller's acceleration to lean angle step
            accel_to_lean_angles();
            _xy_step++;
            break;
    }
}

///
/// private methods
///

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos(float nav_dt)
{
    Vector2f target_vel_adj;
    float vel_desired_total;

    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // constrain and scale the desired velocity
    vel_desired_total = safe_sqrt(_vel_desired.x*_vel_desired.x + _vel_desired.y*_vel_desired.y);
    if (vel_desired_total > _speed_cms && vel_desired_total > 0.0f) {
        _vel_desired.x = _speed_cms * _vel_desired.x/vel_desired_total;
        _vel_desired.y = _speed_cms * _vel_desired.y/vel_desired_total;
    }

    // update target position
    _pos_target.x += _vel_desired.x * nav_dt;
    _pos_target.y += _vel_desired.y * nav_dt;
}

/// pos_to_rate_xy - horizontal position error to velocity controller
///     converts position (_pos_target) to target velocity (_vel_target)
///     when use_desired_rate is set to true:
///         desired velocity (_vel_desired) is combined into final target velocity and
///         velocity due to position error is reduce to a maximum of 1m/s
void AC_PosControl::pos_to_rate_xy(bool use_desired_rate, float dt)
{
    Vector3f curr_pos = _inav.get_position();
    float linear_distance;      // the distance we swap between linear and sqrt velocity response
    float kP = _pi_pos_lat.kP();

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0;
        _vel_target.y = 0.0;
    }else{
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        // constrain target position to within reasonable distance of current location
        _distance_to_target = safe_sqrt(_pos_error.x*_pos_error.x + _pos_error.y*_pos_error.y);
        if (_distance_to_target > _leash && _distance_to_target > 0.0f) {
            _pos_target.x = curr_pos.x + _leash * _pos_error.x/_distance_to_target;
            _pos_target.y = curr_pos.y + _leash * _pos_error.y/_distance_to_target;
            // re-calculate distance error
            _pos_error.x = _pos_target.x - curr_pos.x;
            _pos_error.y = _pos_target.y - curr_pos.y;
            _distance_to_target = _leash;
        }

        // calculate the distance at which we swap between linear and sqrt velocity response
        linear_distance = _accel_cms/(2.0f*kP*kP);

        if (_distance_to_target > 2.0f*linear_distance) {
            // velocity response grows with the square root of the distance
            float vel_sqrt = safe_sqrt(2.0f*_accel_cms*(_distance_to_target-linear_distance));
            _vel_target.x = vel_sqrt * _pos_error.x/_distance_to_target;
            _vel_target.y = vel_sqrt * _pos_error.y/_distance_to_target;
        }else{
            // velocity response grows linearly with the distance
            _vel_target.x = _pi_pos_lat.kP() * _pos_error.x;
            _vel_target.y = _pi_pos_lon.kP() * _pos_error.y;
        }

        // decide velocity limit due to position error
        float vel_max_from_pos_error;
        if (use_desired_rate) {
            // if desired velocity (i.e. velocity feed forward) is being used we limit the maximum velocity correction due to position error to 2m/s
            vel_max_from_pos_error = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR;
        }else{
            // if desired velocity is not used, we allow position error to increase speed up to maximum speed
            vel_max_from_pos_error = _speed_cms;
        }

        // scale velocity to stays within limits
        float vel_total = safe_sqrt(_vel_target.x*_vel_target.x + _vel_target.y*_vel_target.y);
        if (vel_total > vel_max_from_pos_error) {
            _vel_target.x = vel_max_from_pos_error * _vel_target.x/vel_total;
            _vel_target.y = vel_max_from_pos_error * _vel_target.y/vel_total;
        }

        // add desired velocity (i.e. feed forward).
        if (use_desired_rate) {
            _vel_target.x += _vel_desired.x;
            _vel_target.y += _vel_desired.y;
        }
    }
}

/// rate_to_accel_xy - horizontal desired rate to desired acceleration
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AC_PosControl::rate_to_accel_xy(float dt)
{
    const Vector3f &vel_curr = _inav.get_velocity();  // current velocity in cm/s
    float accel_total;                          // total acceleration in cm/s/s

    // reset accel limit flag
    _limit.accel_xy = false;

    // reset last velocity if this controller has just been engaged or dt is zero
    if (dt == 0.0) {
        _accel_target.x = 0;
        _accel_target.y = 0;
    } else {
        // feed forward desired acceleration calculation
        _accel_target.x = (_vel_target.x - _vel_last.x)/dt;
        _accel_target.y = (_vel_target.y - _vel_last.y)/dt;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = _vel_target.x;
    _vel_last.y = _vel_target.y;

    // calculate velocity error
    _vel_error.x = _vel_target.x - vel_curr.x;
    _vel_error.y = _vel_target.y - vel_curr.y;

    // combine feed foward accel with PID output from velocity error
    // To-Do: check accel limit flag before adding I term
    _accel_target.x += _pid_rate_lat.get_pid(_vel_error.x, dt);
    _accel_target.y += _pid_rate_lon.get_pid(_vel_error.y, dt);

    // scale desired acceleration if it's beyond acceptable limit
    // To-Do: move this check down to the accel_to_lean_angle method?
    accel_total = safe_sqrt(_accel_target.x*_accel_target.x + _accel_target.y*_accel_target.y);
    if (accel_total > POSCONTROL_ACCEL_XY_MAX) {
        _accel_target.x = POSCONTROL_ACCEL_XY_MAX * _accel_target.x/accel_total;
        _accel_target.y = POSCONTROL_ACCEL_XY_MAX * _accel_target.y/accel_total;
        _limit.accel_xy = true;     // unused
    }
}

/// accel_to_lean_angles - horizontal desired acceleration to lean angles
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::accel_to_lean_angles()
{
    float accel_right, accel_forward;
    float lean_angle_max = _attitude_control.lean_angle_max();

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward = _accel_target.x*_cos_yaw + _accel_target.y*_sin_yaw;
    accel_right = -_accel_target.x*_sin_yaw + _accel_target.y*_cos_yaw;

    // update angle targets that will be passed to stabilize controller
    _roll_target = constrain_float(fast_atan(accel_right*_cos_pitch/(GRAVITY_MSS * 100))*(18000/M_PI), -lean_angle_max, lean_angle_max);
    _pitch_target = constrain_float(fast_atan(-accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI),-lean_angle_max, lean_angle_max);
}

/// reset_I_xy - clears I terms from loiter PID controller
void AC_PosControl::reset_I_xy()
{
    _pi_pos_lon.reset_I();
    _pi_pos_lat.reset_I();
    _pid_rate_lon.reset_I();
    _pid_rate_lat.reset_I();

    // set last velocity to current velocity
    _vel_last = _inav.get_velocity();
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float AC_PosControl::calc_leash_length(float speed_cms, float accel_cms, float kP) const
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}
