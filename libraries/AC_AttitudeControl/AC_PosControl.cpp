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
    _speed_cms(POSCONTROL_SPEED),
    _vel_z_min(POSCONTROL_VEL_Z_MIN),
    _vel_z_max(POSCONTROL_VEL_Z_MAX),
    _accel_cms(POSCONTROL_ACCEL_XY_MAX),   // To-Do: check this default
    _cos_yaw(1.0),
    _sin_yaw(0.0),
    _cos_pitch(1.0),
    _desired_roll(0.0),
    _desired_pitch(0.0),
    _leash(POSCONTROL_LEASH_LENGTH_MIN)
{
    AP_Param::setup_object_defaults(this, var_info);

    // calculate leash length
    //calculate_leash_length();
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

/// fly_to_target_z - fly to altitude in cm above home
void AC_PosControl::fly_to_target_z()
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
    float linear_distance;  // half the distace we swap between linear and sqrt and the distace we offset sqrt.

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
    if (_vel_target.z < _vel_z_min) {
        _vel_target.z = _vel_z_min;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _vel_z_max) {
        _vel_target.z = _vel_z_max;
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
/*
/// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
void AC_PosControl::get_stopping_point(const Vector3f& position, const Vector3f& velocity, Vector3f &target) const
{
    float linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    float linear_velocity;      // the velocity we swap between linear and sqrt.
    float vel_total;
    float target_dist;
    float kP = _pid_pos_lat->kP();

    // calculate current velocity
    vel_total = safe_sqrt(velocity.x*velocity.x + velocity.y*velocity.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (vel_total < 10.0f || kP <= 0.0f || _wp_accel_cms <= 0.0f) {
        target = position;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _wp_accel_cms/kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
        target_dist = vel_total/kP;
    } else {
        linear_distance = _wp_accel_cms/(2.0f*kP*kP);
        target_dist = linear_distance + (vel_total*vel_total)/(2.0f*_wp_accel_cms);
    }
    target_dist = constrain_float(target_dist, 0, _wp_leash_xy*2.0f);

    target.x = position.x + (target_dist * velocity.x / vel_total);
    target.y = position.y + (target_dist * velocity.y / vel_total);
    target.z = position.z;
}

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _target = position;
    _target_vel.x = 0;
    _target_vel.y = 0;
}

/// init_pos_target - set initial loiter target based on current position and velocity
void AC_PosControl::init_pos_target(const Vector3f& position, const Vector3f& velocity)
{
    // set target position and velocity based on current pos and velocity
    _target.x = position.x;
    _target.y = position.y;
    _target_vel.x = velocity.x;
    _target_vel.y = velocity.y;

    // initialise desired roll and pitch to current roll and pitch.  This avoids a random twitch between now and when the loiter controller is first run
    _desired_roll = constrain_int32(_ahrs->roll_sensor,-_lean_angle_max_cd,_lean_angle_max_cd);
    _desired_pitch = constrain_int32(_ahrs->pitch_sensor,-_lean_angle_max_cd,_lean_angle_max_cd);

    // initialise pilot input
    _pilot_vel_forward_cms = 0;
    _pilot_vel_right_cms = 0;

    // set last velocity to current velocity
    // To-Do: remove the line below by instead forcing reset_I to be called on the first loiter_update call
    _vel_last = _inav->get_velocity();
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AC_PosControl::get_distance_to_target() const
{
    return _distance_to_target;
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AC_PosControl::get_bearing_to_target() const
{
    return get_bearing_cd(_inav->get_position(), _target);
}

/// update_loiter - run the loiter controller - should be called at 10hz
void AC_PosControl::update_loiter()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
        _loiter_step = 0;
    }

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt > 0.095f && _loiter_step > 3) {
        _loiter_step = 0;
    }

    // run loiter steps
    switch (_loiter_step) {
        case 0:
            // capture time since last iteration
            _loiter_dt = dt;
            _loiter_last_update = now;

            // translate any adjustments from pilot to loiter target
            translate_loiter_target_movements(_loiter_dt);
            _loiter_step++;
            break;
        case 1:
            // run loiter's position to velocity step
            get_loiter_position_to_velocity(_loiter_dt, WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR);
            _loiter_step++;
            break;
        case 2:
            // run loiter's velocity to acceleration step
            get_loiter_velocity_to_acceleration(desired_vel.x, desired_vel.y, _loiter_dt);
            _loiter_step++;
            break;
        case 3:
            // run loiter's acceleration to lean angle step
            get_loiter_acceleration_to_lean_angles(desired_accel.x, desired_accel.y);
            _loiter_step++;
            break;
    }
}

/// calculate_leash_length - calculates the maximum distance in cm that the target position may be from the current location
void AC_PosControl::calculate_leash_length()
{
    // get loiter position P
    float kP = _pid_pos_lat->kP();

    // check loiter speed
    if( _loiter_speed_cms < 100.0f) {
        _loiter_speed_cms = 100.0f;
    }

    // set loiter acceleration to 1/2 loiter speed
    _loiter_accel_cms = _loiter_speed_cms / 2.0f;

    // avoid divide by zero
    if (kP <= 0.0f || _wp_accel_cms <= 0.0f) {
        _loiter_leash = WPNAV_MIN_LEASH_LENGTH;
        return;
    }

    // calculate horizontal leash length
    if(WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR <= _wp_accel_cms / kP) {
        // linear leash length based on speed close in
        _loiter_leash = WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR / kP;
    }else{
        // leash length grows at sqrt of speed further out
        _loiter_leash = (_wp_accel_cms / (2.0f*kP*kP)) + (WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR*WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR / (2.0f*_wp_accel_cms));
    }

    // ensure leash is at least 1m long
    if( _loiter_leash < WPNAV_MIN_LEASH_LENGTH ) {
        _loiter_leash = WPNAV_MIN_LEASH_LENGTH;
    }
}

///
/// shared methods
///

/// get_loiter_position_to_velocity - loiter position controller
///     converts desired position held in _target vector to desired velocity
void AC_PosControl::get_loiter_position_to_velocity(float dt, float max_speed_cms)
{
    Vector3f curr = _inav->get_position();
    float dist_error_total;

    float vel_sqrt;
    float vel_total;

    float linear_distance;      // the distace we swap between linear and sqrt.
    float kP = _pid_pos_lat->kP();

    // avoid divide by zero
    if (kP <= 0.0f) {
        desired_vel.x = 0.0;
        desired_vel.y = 0.0;
    }else{
        // calculate distance error
        dist_error.x = _target.x - curr.x;
        dist_error.y = _target.y - curr.y;

        linear_distance = _wp_accel_cms/(2.0f*kP*kP);

        dist_error_total = safe_sqrt(dist_error.x*dist_error.x + dist_error.y*dist_error.y);
        _distance_to_target = dist_error_total;      // for reporting purposes

        if( dist_error_total > 2.0f*linear_distance ) {
            vel_sqrt = safe_sqrt(2.0f*_wp_accel_cms*(dist_error_total-linear_distance));
            desired_vel.x = vel_sqrt * dist_error.x/dist_error_total;
            desired_vel.y = vel_sqrt * dist_error.y/dist_error_total;
        }else{
            desired_vel.x = _pid_pos_lat->kP() * dist_error.x;
            desired_vel.y = _pid_pos_lon->kP() * dist_error.y;
        }

        // ensure velocity stays within limits
        vel_total = safe_sqrt(desired_vel.x*desired_vel.x + desired_vel.y*desired_vel.y);
        if( vel_total > max_speed_cms ) {
            desired_vel.x = max_speed_cms * desired_vel.x/vel_total;
            desired_vel.y = max_speed_cms * desired_vel.y/vel_total;
        }

        // feed forward velocity request
        desired_vel.x += _target_vel.x;
        desired_vel.y += _target_vel.y;
    }
}

/// get_loiter_velocity_to_acceleration - loiter velocity controller
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AC_PosControl::get_loiter_velocity_to_acceleration(float vel_lat, float vel_lon, float dt)
{
    const Vector3f &vel_curr = _inav->get_velocity();  // current velocity in cm/s
    Vector3f vel_error;                         // The velocity error in cm/s.
    float accel_total;                          // total acceleration in cm/s/s

    // reset last velocity if this controller has just been engaged or dt is zero
    if( dt == 0.0 ) {
        desired_accel.x = 0;
        desired_accel.y = 0;
    } else {
        // feed forward desired acceleration calculation
        desired_accel.x = (vel_lat - _vel_last.x)/dt;
        desired_accel.y = (vel_lon - _vel_last.y)/dt;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = vel_lat;
    _vel_last.y = vel_lon;

    // calculate velocity error
    vel_error.x = vel_lat - vel_curr.x;
    vel_error.y = vel_lon - vel_curr.y;

    // combine feed foward accel with PID outpu from velocity error
    desired_accel.x += _pid_rate_lat->get_pid(vel_error.x, dt);
    desired_accel.y += _pid_rate_lon->get_pid(vel_error.y, dt);

    // scale desired acceleration if it's beyond acceptable limit
    accel_total = safe_sqrt(desired_accel.x*desired_accel.x + desired_accel.y*desired_accel.y);
    if( accel_total > WPNAV_ACCEL_MAX ) {
        desired_accel.x = WPNAV_ACCEL_MAX * desired_accel.x/accel_total;
        desired_accel.y = WPNAV_ACCEL_MAX * desired_accel.y/accel_total;
    }
}

/// get_loiter_acceleration_to_lean_angles - loiter acceleration controller
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::get_loiter_acceleration_to_lean_angles(float accel_lat, float accel_lon)
{
    float z_accel_meas = -GRAVITY_MSS * 100;    // gravity in cm/s/s
    float accel_forward;
    float accel_right;

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward = accel_lat*_cos_yaw + accel_lon*_sin_yaw;
    accel_right = -accel_lat*_sin_yaw + accel_lon*_cos_yaw;

    // update angle targets that will be passed to stabilize controller
    _desired_roll = constrain_float(fast_atan(accel_right*_cos_pitch/(-z_accel_meas))*(18000/M_PI), -_lean_angle_max_cd, _lean_angle_max_cd);
    _desired_pitch = constrain_float(fast_atan(-accel_forward/(-z_accel_meas))*(18000/M_PI), -_lean_angle_max_cd, _lean_angle_max_cd);
}

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_PosControl::get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// reset_I - clears I terms from loiter PID controller
void AC_PosControl::reset_I()
{
    _pid_pos_lon->reset_I();
    _pid_pos_lat->reset_I();
    _pid_rate_lon->reset_I();
    _pid_rate_lat->reset_I();

    // set last velocity to current velocity
    _vel_last = _inav->get_velocity();
}
*/
