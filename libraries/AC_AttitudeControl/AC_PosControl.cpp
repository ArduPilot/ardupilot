#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PosControl::var_info[] = {
    // 0 was used for HOVER

    // @Param: _ACC_XY_FILT
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_ACC_XY_FILT", 1, AC_PosControl, _accel_xy_filt_hz, POSCONTROL_ACCEL_FILTER_HZ),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             AC_P& p_pos_z, AC_P& p_vel_z, AC_PID& pid_accel_z,
                             AC_P& p_pos_xy, AC_PI_2D& pi_vel_xy) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_z(p_pos_z),
    _p_vel_z(p_vel_z),
    _pid_accel_z(pid_accel_z),
    _p_pos_xy(p_pos_xy),
    _pi_vel_xy(pi_vel_xy),
    _dt(POSCONTROL_DT_400HZ),
    _dt_xy(POSCONTROL_DT_50HZ),
    _last_update_xy_ms(0),
    _last_update_z_ms(0),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_Z),
    _accel_last_z_cms(0.0f),
    _accel_cms(POSCONTROL_ACCEL_XY),
    _jerk_cmsss(POSCONTROL_JERK_LIMIT_CMSSS),
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(POSCONTROL_LEASH_LENGTH_MIN),
    _roll_target(0.0f),
    _pitch_target(0.0f),
    _distance_to_target(0.0f),
    _accel_target_jerk_limited(0.0f,0.0f),
    _accel_target_filter(POSCONTROL_ACCEL_FILTER_HZ)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.recalc_leash_z = true;
    _flags.recalc_leash_xy = true;
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;
    _flags.reset_accel_to_lean_xy = true;
    _flags.reset_rate_to_accel_z = true;
    _flags.reset_accel_to_throttle = true;
    _flags.freeze_ff_xy = true;
    _flags.freeze_ff_z = true;
    _flags.use_desvel_ff_z = true;
    _limit.pos_up = true;
    _limit.pos_down = true;
    _limit.vel_up = true;
    _limit.vel_down = true;
    _limit.accel_xy = true;
}

///
/// z-axis position controller
///


/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AC_PosControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update rate controller's dt
    _pid_accel_z.set_dt(_dt);

    // update rate z-axis velocity error and accel error filters
    _vel_error_filter.set_cutoff_frequency(POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
}

/// set_dt_xy - sets time delta in seconds for horizontal controller (i.e. 50hz = 0.02)
void AC_PosControl::set_dt_xy(float dt_xy)
{
    _dt_xy = dt_xy;
    _pi_vel_xy.set_dt(dt_xy);
}

/// set_speed_z - sets maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
///     calc_leash_length_z should be called afterwards
///     speed_down should be a negative number
void AC_PosControl::set_speed_z(float speed_down, float speed_up)
{
    // ensure speed_down is always negative
    speed_down = -fabsf(speed_down);

    if ((fabsf(_speed_down_cms-speed_down) > 1.0f) || (fabsf(_speed_up_cms-speed_up) > 1.0f)) {
        _speed_down_cms = speed_down;
        _speed_up_cms = speed_up;
        _flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

/// set_accel_z - set vertical acceleration in cm/s/s
void AC_PosControl::set_accel_z(float accel_cmss)
{
    if (fabsf(_accel_z_cms-accel_cmss) > 1.0f) {
        _accel_z_cms = accel_cmss;
        _flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

/// set_alt_target_with_slew - adjusts target towards a final altitude target
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_with_slew(float alt_cm, float dt)
{
    float alt_change = alt_cm-_pos_target.z;

    // do not use z-axis desired velocity feed forward
    _flags.use_desvel_ff_z = false;

    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !_motors.limit.throttle_lower) || (alt_change>0 && !_motors.limit.throttle_upper)) {
        if (!is_zero(dt)) {
            float climb_rate_cms = constrain_float(alt_change/dt, _speed_down_cms, _speed_up_cms);
            _pos_target.z += climb_rate_cms*dt;
            _vel_desired.z = climb_rate_cms;    // recorded for reporting purposes
        }
    }

    // do not let target get too far from current altitude
    float curr_alt = _inav.get_altitude();
    _pos_target.z = constrain_float(_pos_target.z,curr_alt-_leash_down_z,curr_alt+_leash_up_z);
}


/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms<0 && (!_motors.limit.throttle_lower || force_descend)) || (climb_rate_cms>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += climb_rate_cms * dt;
    }

    // do not use z-axis desired velocity feed forward
    // vel_desired set to desired climb rate for reporting and land-detector
    _flags.use_desvel_ff_z = false;
    _vel_desired.z = climb_rate_cms;
}

/// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
///     set force_descend to true during landing to allow target to move low enough to slow the motors
void AC_PosControl::set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
{
    // calculated increased maximum acceleration if over speed
    float accel_z_cms = _accel_z_cms;
    if (_vel_desired.z < _speed_down_cms && !is_zero(_speed_down_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down_cms;
    }
    if (_vel_desired.z > _speed_up_cms && !is_zero(_speed_up_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up_cms;
    }
    accel_z_cms = constrain_float(accel_z_cms, 0.0f, 750.0f);

    // jerk_z is calculated to reach full acceleration in 1000ms.
    float jerk_z = accel_z_cms * POSCONTROL_JERK_RATIO;

    float accel_z_max = MIN(accel_z_cms, safe_sqrt(2.0f*fabsf(_vel_desired.z - climb_rate_cms)*jerk_z));

    _accel_last_z_cms += jerk_z * dt;
    _accel_last_z_cms = MIN(accel_z_max, _accel_last_z_cms);

    float vel_change_limit = _accel_last_z_cms * dt;
    _vel_desired.z = constrain_float(climb_rate_cms, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    _flags.use_desvel_ff_z = true;

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((_vel_desired.z<0 && (!_motors.limit.throttle_lower || force_descend)) || (_vel_desired.z>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += _vel_desired.z * dt;
    }
}

/// add_takeoff_climb_rate - adjusts alt target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     almost no checks are performed on the input
void AC_PosControl::add_takeoff_climb_rate(float climb_rate_cms, float dt)
{
    _pos_target.z += climb_rate_cms * dt;
}

/// shift altitude target (positive means move altitude up)
void AC_PosControl::shift_alt_target(float z_cm)
{
    _pos_target.z += z_cm;

    // freeze feedforward to avoid jump
    if (!is_zero(z_cm)) {
        freeze_ff_z();
    }
}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl::relax_alt_hold_controllers(float throttle_setting)
{
    _pos_target.z = _inav.get_altitude();
    _vel_desired.z = 0.0f;
    _flags.use_desvel_ff_z = false;
    _vel_target.z = _inav.get_velocity_z();
    _vel_last.z = _inav.get_velocity_z();
    _accel_feedforward.z = 0.0f;
    _accel_last_z_cms = 0.0f;
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _flags.reset_accel_to_throttle = true;
    _pid_accel_z.set_integrator((throttle_setting-_motors.get_throttle_hover())*1000.0f);
}

// get_alt_error - returns altitude error in cm
float AC_PosControl::get_alt_error() const
{
    return (_pos_target.z - _inav.get_altitude());
}

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
void AC_PosControl::set_target_to_stopping_point_z()
{
    // check if z leash needs to be recalculated
    calc_leash_length_z();

    get_stopping_point_z(_pos_target);
}

/// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
void AC_PosControl::get_stopping_point_z(Vector3f& stopping_point) const
{
    const float curr_pos_z = _inav.get_altitude();
    float curr_vel_z = _inav.get_velocity_z();

    float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt
    float linear_velocity;  // the velocity we swap between linear and sqrt

    // if position controller is active add current velocity error to avoid sudden jump in acceleration
    if (is_active_z()) {
        curr_vel_z += _vel_error.z;
        if (_flags.use_desvel_ff_z) {
            curr_vel_z -= _vel_desired.z;
        }
    }

    // avoid divide by zero by using current position if kP is very low or acceleration is zero
    if (_p_pos_z.kP() <= 0.0f || _accel_z_cms <= 0.0f) {
        stopping_point.z = curr_pos_z;
        return;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    linear_velocity = _accel_z_cms/_p_pos_z.kP();

    if (fabsf(curr_vel_z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        stopping_point.z = curr_pos_z + curr_vel_z/_p_pos_z.kP();
    } else {
        linear_distance = _accel_z_cms/(2.0f*_p_pos_z.kP()*_p_pos_z.kP());
        if (curr_vel_z > 0){
            stopping_point.z = curr_pos_z + (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        } else {
            stopping_point.z = curr_pos_z - (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        }
    }
    stopping_point.z = constrain_float(stopping_point.z, curr_pos_z - POSCONTROL_STOPPING_DIST_DOWN_MAX, curr_pos_z + POSCONTROL_STOPPING_DIST_UP_MAX);
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    const Vector3f& curr_pos = _inav.get_position();

    _pos_target.z = curr_pos.z;

    // freeze feedforward to avoid jump
    freeze_ff_z();

    // shift difference between last motor out and hover throttle into accelerometer I
    _pid_accel_z.set_integrator((_motors.get_throttle()-_motors.get_throttle_hover())*1000.0f);

    // initialise ekf reset handler
    init_ekf_z_reset();
}

// is_active_z - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_z() const
{
    return ((AP_HAL::millis() - _last_update_z_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // check time since last cast
    uint32_t now = AP_HAL::millis();
    if (now - _last_update_z_ms > POSCONTROL_ACTIVE_TIMEOUT_MS) {
        _flags.reset_rate_to_accel_z = true;
        _flags.reset_accel_to_throttle = true;
    }
    _last_update_z_ms = now;

    // check for ekf altitude reset
    check_for_ekf_z_reset();

    // check if leash lengths need to be recalculated
    calc_leash_length_z();

    // call position controller
    pos_to_rate_z();
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by pos_to_rate_z if z-axis speed or accelerations are changed
void AC_PosControl::calc_leash_length_z()
{
    if (_flags.recalc_leash_z) {
        _leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, _p_pos_z.kP());
        _leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, _p_pos_z.kP());
        _flags.recalc_leash_z = false;
    }
}

// pos_to_rate_z - position to rate controller for Z axis
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void AC_PosControl::pos_to_rate_z()
{
    float curr_alt = _inav.get_altitude();

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    // do not let target altitude get too far from current altitude
    if (_pos_error.z > _leash_up_z) {
        _pos_target.z = curr_alt + _leash_up_z;
        _pos_error.z = _leash_up_z;
        _limit.pos_up = true;
    }
    if (_pos_error.z < -_leash_down_z) {
        _pos_target.z = curr_alt - _leash_down_z;
        _pos_error.z = -_leash_down_z;
        _limit.pos_down = true;
    }

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_z_cms);

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

    // add feed forward component
    if (_flags.use_desvel_ff_z) {
        _vel_target.z += _vel_desired.z;
    }

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z();
}

// rate_to_accel_z - calculates desired accel required to achieve the velocity target
// calculates desired acceleration and calls accel throttle controller
void AC_PosControl::rate_to_accel_z()
{
    const Vector3f& curr_vel = _inav.get_velocity();
    float p;                                // used to capture pid values for logging

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
    	if (!_flags.freeze_ff_z) {
    		_accel_feedforward.z = (_vel_target.z - _vel_last.z)/_dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_z = false;
    	}
    } else {
    	_accel_feedforward.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.z = _vel_target.z;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_z) {
        // Reset Filter
        _vel_error.z = 0;
        _vel_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
    }

    // calculate p
    p = _p_vel_z.kP() * _vel_error.z;

    // consolidate and constrain target acceleration
    _accel_target.z = _accel_feedforward.z + p;

    // set target for accel based throttle controller
    accel_to_throttle(_accel_target.z);
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void AC_PosControl::accel_to_throttle(float accel_target_z)
{
    float z_accel_meas;         // actual acceleration
    float p,i,d;              // used to capture pid values for logging

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // reset target altitude if this controller has just been engaged
    if (_flags.reset_accel_to_throttle) {
        // Reset Filter
        _accel_error.z = 0;
        _flags.reset_accel_to_throttle = false;
    } else {
        // calculate accel error
        _accel_error.z = accel_target_z - z_accel_meas;
    }

    // set input to PID
    _pid_accel_z.set_input_filter_all(_accel_error.z);
    _pid_accel_z.set_desired_rate(accel_target_z);

    // separately calculate p, i, d values for logging
    p = _pid_accel_z.get_p();

    // get i term
    i = _pid_accel_z.get_integrator();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
        i = _pid_accel_z.get_i();
    }

    // get d term
    d = _pid_accel_z.get_d();

    float thr_out = (p+i+d)/1000.0f +_motors.get_throttle_hover();

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);
}

///
/// position controller
///

/// set_accel_xy - set horizontal acceleration in cm/s/s
///     calc_leash_length_xy should be called afterwards
void AC_PosControl::set_accel_xy(float accel_cmss)
{
    if (fabsf(_accel_cms-accel_cmss) > 1.0f) {
        _accel_cms = accel_cmss;
        _flags.recalc_leash_xy = true;
        calc_leash_length_xy();
    }
}

/// set_speed_xy - set horizontal speed maximum in cm/s
///     calc_leash_length_xy should be called afterwards
void AC_PosControl::set_speed_xy(float speed_cms)
{
    if (fabsf(_speed_cms-speed_cms) > 1.0f) {
        _speed_cms = speed_cms;
        _flags.recalc_leash_xy = true;
        calc_leash_length_xy();
    }
}

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _pos_target = position;

    _flags.use_desvel_ff_z = false;
    _vel_desired.z = 0.0f;
    // initialise roll and pitch to current roll and pitch.  This avoids a twitch between when the target is set and the pos controller is first run
    // To-Do: this initialisation of roll and pitch targets needs to go somewhere between when pos-control is initialised and when it completes it's first cycle
    //_roll_target = constrain_int32(_ahrs.roll_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
    //_pitch_target = constrain_int32(_ahrs.pitch_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
}

/// set_xy_target in cm from home
void AC_PosControl::set_xy_target(float x, float y)
{
    _pos_target.x = x;
    _pos_target.y = y;
}

/// shift position target target in x, y axis
void AC_PosControl::shift_pos_xy_target(float x_cm, float y_cm)
{
    // move pos controller target
    _pos_target.x += x_cm;
    _pos_target.y += y_cm;

    // disable feed forward
    if (!is_zero(x_cm) || !is_zero(y_cm)) {
        freeze_ff_xy();
    }
}

/// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
void AC_PosControl::set_target_to_stopping_point_xy()
{
    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    get_stopping_point_xy(_pos_target);
}

/// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
    const Vector3f curr_pos = _inav.get_position();
    Vector3f curr_vel = _inav.get_velocity();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;		// the distance within the vehicle can stop
    float kP = _p_pos_xy.kP();

    // add velocity error to current velocity
    if (is_active_xy()) {
        curr_vel.x += _vel_error.x;
        curr_vel.y += _vel_error.y;
    }

    // calculate current velocity
    float vel_total = norm(curr_vel.x, curr_vel.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || _accel_cms <= 0.0f || is_zero(vel_total)) {
        stopping_point.x = curr_pos.x;
        stopping_point.y = curr_pos.y;
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

// is_active_xy - returns true if the xy position controller has been run very recently
bool AC_PosControl::is_active_xy() const
{
    return ((AP_HAL::millis() - _last_update_xy_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// init_xy_controller - initialise the xy controller
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the xy target
void AC_PosControl::init_xy_controller(bool reset_I)
{
    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // initialise I terms from lean angles
    if (reset_I) {
        // reset last velocity if this controller has just been engaged or dt is zero
        lean_angles_to_accel(_accel_target.x, _accel_target.y);
        _pi_vel_xy.set_integrator(_accel_target);
    }

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;
    _flags.reset_accel_to_lean_xy = true;

    // initialise ekf xy reset handler
    init_ekf_xy_reset();
}

/// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
void AC_PosControl::update_xy_controller(xy_mode mode, float ekfNavVelGainScaler, bool use_althold_lean_angle)
{
    // compute dt
    uint32_t now = AP_HAL::millis();
    float dt = (now - _last_update_xy_ms) / 1000.0f;
    _last_update_xy_ms = now;

    // sanity check dt - expect to be called faster than ~5hz
    if (dt > POSCONTROL_ACTIVE_TIMEOUT_MS*1.0e-3f) {
        dt = 0.0f;
    }

    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // translate any adjustments from pilot to loiter target
    desired_vel_to_pos(dt);

    // run position controller's position error to desired velocity step
    pos_to_rate_xy(mode, dt, ekfNavVelGainScaler);

    // run position controller's velocity to acceleration step
    rate_to_accel_xy(dt, ekfNavVelGainScaler);

    // run position controller's acceleration to lean angle step
    accel_to_lean_angles(dt, ekfNavVelGainScaler, use_althold_lean_angle);
}

float AC_PosControl::time_since_last_xy_update() const
{
    uint32_t now = AP_HAL::millis();
    return (now - _last_update_xy_ms)*0.001f;
}

/// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
void AC_PosControl::init_vel_controller_xyz()
{
    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // reset last velocity if this controller has just been engaged or dt is zero
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pi_vel_xy.set_integrator(_accel_target);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_rate_to_accel_xy = true;
    _flags.reset_accel_to_lean_xy = true;

    // set target position
    const Vector3f& curr_pos = _inav.get_position();
    set_xy_target(curr_pos.x, curr_pos.y);
    set_alt_target(curr_pos.z);

    // move current vehicle velocity into feed forward velocity
    const Vector3f& curr_vel = _inav.get_velocity();
    set_desired_velocity(curr_vel);

    // initialise ekf reset handlers
    init_ekf_xy_reset();
    init_ekf_z_reset();
}

/// update_velocity_controller_xy - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xy() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xy(float ekfNavVelGainScaler)
{
    // capture time since last iteration
    uint32_t now = AP_HAL::millis();
    float dt = (now - _last_update_xy_ms)*0.001f;

    // call xy controller
    if (dt >= get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // check for ekf xy position reset
        check_for_ekf_xy_reset();

        // check if xy leash needs to be recalculated
        calc_leash_length_xy();

        // apply desired velocity request to position target
        desired_vel_to_pos(dt);

        // run position controller's position error to desired velocity step
        pos_to_rate_xy(XY_MODE_POS_LIMITED_AND_VEL_FF, dt, ekfNavVelGainScaler);

        // run velocity to acceleration step
        rate_to_accel_xy(dt, ekfNavVelGainScaler);

        // run acceleration to lean angle step
        accel_to_lean_angles(dt, ekfNavVelGainScaler, false);

        // update xy update time
        _last_update_xy_ms = now;
    }
}


/// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xyz() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xyz(float ekfNavVelGainScaler)
{
    update_vel_controller_xy(ekfNavVelGainScaler);

    // update altitude target
    set_alt_target_from_climb_rate_ff(_vel_desired.z, _dt, false);

    // run z-axis position controller
    update_z_controller();
}

float AC_PosControl::get_horizontal_error() const
{
    return norm(_pos_error.x, _pos_error.y);
}

///
/// private methods
///

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
///     should be called whenever the speed, acceleration or position kP is modified
void AC_PosControl::calc_leash_length_xy()
{
    if (_flags.recalc_leash_xy) {
        _leash = calc_leash_length(_speed_cms, _accel_cms, _p_pos_xy.kP());
        _flags.recalc_leash_xy = false;
    }
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // update target position
    if (_flags.reset_desired_vel_to_pos) {
        _flags.reset_desired_vel_to_pos = false;
    } else {
        _pos_target.x += _vel_desired.x * nav_dt;
        _pos_target.y += _vel_desired.y * nav_dt;
    }
}

/// pos_to_rate_xy - horizontal position error to velocity controller
///     converts position (_pos_target) to target velocity (_vel_target)
///     when use_desired_rate is set to true:
///         desired velocity (_vel_desired) is combined into final target velocity and
///         velocity due to position error is reduce to a maximum of 1m/s
void AC_PosControl::pos_to_rate_xy(xy_mode mode, float dt, float ekfNavVelGainScaler)
{
    Vector3f curr_pos = _inav.get_position();
    float linear_distance;      // the distance we swap between linear and sqrt velocity response
    float kP = ekfNavVelGainScaler * _p_pos_xy.kP(); // scale gains to compensate for noisy optical flow measurement in the EKF

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0f;
        _vel_target.y = 0.0f;
    }else{
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        // constrain target position to within reasonable distance of current location
        _distance_to_target = norm(_pos_error.x, _pos_error.y);
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
            _vel_target.x = kP * _pos_error.x;
            _vel_target.y = kP * _pos_error.y;
        }

        if (mode == XY_MODE_POS_LIMITED_AND_VEL_FF) {
            // this mode is for loiter - rate-limiting the position correction
            // allows the pilot to always override the position correction in
            // the event of a disturbance

            // scale velocity within limit
            float vel_total = norm(_vel_target.x, _vel_target.y);
            if (vel_total > POSCONTROL_VEL_XY_MAX_FROM_POS_ERR) {
                _vel_target.x = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR * _vel_target.x/vel_total;
                _vel_target.y = POSCONTROL_VEL_XY_MAX_FROM_POS_ERR * _vel_target.y/vel_total;
            }

            // add velocity feed-forward
            _vel_target.x += _vel_desired.x;
            _vel_target.y += _vel_desired.y;
        } else {
            if (mode == XY_MODE_POS_AND_VEL_FF) {
                // add velocity feed-forward
                _vel_target.x += _vel_desired.x;
                _vel_target.y += _vel_desired.y;
            }

            // scale velocity within speed limit
            float vel_total = norm(_vel_target.x, _vel_target.y);
            if (vel_total > _speed_cms) {
                _vel_target.x = _speed_cms * _vel_target.x/vel_total;
                _vel_target.y = _speed_cms * _vel_target.y/vel_total;
            }
        }
    }
}

/// rate_to_accel_xy - horizontal desired rate to desired acceleration
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AC_PosControl::rate_to_accel_xy(float dt, float ekfNavVelGainScaler)
{
    Vector2f vel_xy_p, vel_xy_i;

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_xy) {
        _vel_last.x = _vel_target.x;
        _vel_last.y = _vel_target.y;
        _flags.reset_rate_to_accel_xy = false;
    }

    // check if vehicle velocity is being overridden
    if (_flags.vehicle_horiz_vel_override) {
        _flags.vehicle_horiz_vel_override = false;
    } else {
        _vehicle_horiz_vel.x = _inav.get_velocity().x;
        _vehicle_horiz_vel.y = _inav.get_velocity().y;
    }

    // feed forward desired acceleration calculation
    if (dt > 0.0f) {
    	if (!_flags.freeze_ff_xy) {
    		_accel_feedforward.x = (_vel_target.x - _vel_last.x)/dt;
    		_accel_feedforward.y = (_vel_target.y - _vel_last.y)/dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_xy = false;
    	}
    } else {
    	_accel_feedforward.x = 0.0f;
    	_accel_feedforward.y = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = _vel_target.x;
    _vel_last.y = _vel_target.y;

    // calculate velocity error
    _vel_error.x = _vel_target.x - _vehicle_horiz_vel.x;
    _vel_error.y = _vel_target.y - _vehicle_horiz_vel.y;

    // call pi controller
    _pi_vel_xy.set_input(_vel_error);

    // get p
    vel_xy_p = _pi_vel_xy.get_p();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    if (!_limit.accel_xy && !_motors.limit.throttle_upper) {
        vel_xy_i = _pi_vel_xy.get_i();
    } else {
        vel_xy_i = _pi_vel_xy.get_i_shrink();
    }

    // combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    _accel_target.x = _accel_feedforward.x + (vel_xy_p.x + vel_xy_i.x) * ekfNavVelGainScaler;
    _accel_target.y = _accel_feedforward.y + (vel_xy_p.y + vel_xy_i.y) * ekfNavVelGainScaler;
}

/// accel_to_lean_angles - horizontal desired acceleration to lean angles
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::accel_to_lean_angles(float dt, float ekfNavVelGainScaler, bool use_althold_lean_angle)
{
    float accel_total;                          // total acceleration in cm/s/s
    float accel_right, accel_forward;
    float lean_angle_max = _attitude_control.lean_angle_max();
    float accel_max = POSCONTROL_ACCEL_XY_MAX;

    // limit acceleration if necessary
    if (use_althold_lean_angle) {
        accel_max = MIN(accel_max, GRAVITY_MSS * 100.0f * tanf(ToRad(constrain_float(_attitude_control.get_althold_lean_angle_max(),1000,8000)/100.0f)));
    }

    // scale desired acceleration if it's beyond acceptable limit
    accel_total = norm(_accel_target.x, _accel_target.y);
    if (accel_total > accel_max && accel_total > 0.0f) {
        _accel_target.x = accel_max * _accel_target.x/accel_total;
        _accel_target.y = accel_max * _accel_target.y/accel_total;
        _limit.accel_xy = true;     // unused
    } else {
        // reset accel limit flag
        _limit.accel_xy = false;
    }

    // reset accel to current desired acceleration
    if (_flags.reset_accel_to_lean_xy) {
        _accel_target_jerk_limited.x = _accel_target.x;
        _accel_target_jerk_limited.y = _accel_target.y;
        _accel_target_filter.reset(Vector2f(_accel_target.x, _accel_target.y));
        _flags.reset_accel_to_lean_xy = false;
    }

    // apply jerk limit of 17 m/s^3 - equates to a worst case of about 100 deg/sec/sec
    float max_delta_accel = dt * _jerk_cmsss;

    Vector2f accel_in(_accel_target.x, _accel_target.y);
    Vector2f accel_change = accel_in-_accel_target_jerk_limited;
    float accel_change_length = accel_change.length();

    if(accel_change_length > max_delta_accel) {
        accel_change *= max_delta_accel/accel_change_length;
    }
    _accel_target_jerk_limited += accel_change;

    // lowpass filter on NE accel
    _accel_target_filter.set_cutoff_frequency(MIN(_accel_xy_filt_hz, 5.0f*ekfNavVelGainScaler));
    Vector2f accel_target_filtered = _accel_target_filter.apply(_accel_target_jerk_limited, dt);

    // rotate accelerations into body forward-right frame
    accel_forward = accel_target_filtered.x*_ahrs.cos_yaw() + accel_target_filtered.y*_ahrs.sin_yaw();
    accel_right = -accel_target_filtered.x*_ahrs.sin_yaw() + accel_target_filtered.y*_ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    _pitch_target = constrain_float(atanf(-accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI),-lean_angle_max, lean_angle_max);
    float cos_pitch_target = cosf(_pitch_target*M_PI/18000);
    _roll_target = constrain_float(atanf(accel_right*cos_pitch_target/(GRAVITY_MSS * 100))*(18000/M_PI), -lean_angle_max, lean_angle_max);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    accel_x_cmss = (GRAVITY_MSS * 100) * (-(_ahrs.cos_yaw() * _ahrs.sin_pitch() / MAX(_ahrs.cos_pitch(),0.5f)) - _ahrs.sin_yaw() * _ahrs.sin_roll() / MAX(_ahrs.cos_roll(),0.5f));
    accel_y_cmss = (GRAVITY_MSS * 100) * (-(_ahrs.sin_yaw() * _ahrs.sin_pitch() / MAX(_ahrs.cos_pitch(),0.5f)) + _ahrs.cos_yaw() * _ahrs.sin_roll() / MAX(_ahrs.cos_roll(),0.5f));
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

/// initialise ekf xy position reset check
void AC_PosControl::init_ekf_xy_reset()
{
    Vector2f pos_shift;
    _ekf_xy_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_xy_reset()
{
    // check for position shift
    Vector2f pos_shift;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_xy_reset_ms) {
        shift_pos_xy_target(pos_shift.x * 100.0f, pos_shift.y * 100.0f);
        _ekf_xy_reset_ms = reset_ms;
    }
}

/// initialise ekf z axis reset check
void AC_PosControl::init_ekf_z_reset()
{
    float alt_shift;
    _ekf_z_reset_ms = _ahrs.getLastPosDownReset(alt_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_z_reset()
{
    // check for position shift
    float alt_shift;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(alt_shift);
    if (reset_ms != _ekf_z_reset_ms) {
        shift_alt_target(-alt_shift * 100.0f);
        _ekf_z_reset_ms = reset_ms;
    }
}
