#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.3f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.02f   // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.4f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.7f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.35f   // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define POSCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

// vibration compensation gains
#define POSCONTROL_VIBE_COMP_P_GAIN 0.250f
#define POSCONTROL_VIBE_COMP_I_GAIN 0.125f

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

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_z, "_POSZ_", 2, AC_PosControl, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_z, "_VELZ_", 3, AC_PosControl, AC_P),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FF
    // @DisplayName: Acceleration (vertical) controller feed forward
    // @Description: Acceleration (vertical) controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: _ACCZ_FLTT
    // @DisplayName: Acceleration (vertical) controller target frequency in Hz
    // @Description: Acceleration (vertical) controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTE
    // @DisplayName: Acceleration (vertical) controller error frequency in Hz
    // @Description: Acceleration (vertical) controller error frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTD
    // @DisplayName: Acceleration (vertical) controller derivative frequency in Hz
    // @Description: Acceleration (vertical) controller derivative frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_z, "_ACCZ_", 4, AC_PosControl, AC_PID),

    // @Param: _POSXY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_xy, "_POSXY_", 5, AC_PosControl, AC_P),

    // @Param: _VELXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VELXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELXY_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VELXY_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_D_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_xy, "_VELXY_", 6, AC_PosControl, AC_PID_2D),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_ANGLE_MAX", 7, AC_PosControl, _lean_angle_max, 0.0f),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_z(POSCONTROL_POS_Z_P),
    _p_vel_z(POSCONTROL_VEL_Z_P),
    _pid_accel_z(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, 0.0f, POSCONTROL_ACC_Z_IMAX, 0.0f, POSCONTROL_ACC_Z_FILT_HZ, 0.0f, POSCONTROL_ACC_Z_DT),
    _p_pos_xy(POSCONTROL_POS_XY_P),
    _pid_vel_xy(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_50HZ),
    _dt(POSCONTROL_DT_400HZ),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_Z),
    _accel_cms(POSCONTROL_ACCEL_XY),
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(POSCONTROL_LEASH_LENGTH_MIN),
    _accel_target_filter(POSCONTROL_ACCEL_FILTER_HZ)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.recalc_leash_z = true;
    _flags.recalc_leash_xy = true;
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;
    _flags.reset_rate_to_accel_z = true;
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

    // update PID controller dt
    _pid_accel_z.set_dt(_dt);
    _pid_vel_xy.set_dt(_dt);

    // update rate z-axis velocity error and accel error filters
    _vel_error_filter.set_cutoff_frequency(POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
}

/// set_max_speed_z - set the maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
void AC_PosControl::set_max_speed_z(float speed_down, float speed_up)
{
    // ensure speed_down is always negative
    speed_down = -fabsf(speed_down);

    // only update if there is a minimum of 1cm/s change and is valid
    if (((fabsf(_speed_down_cms - speed_down) > 1.0f) || (fabsf(_speed_up_cms - speed_up) > 1.0f)) && is_positive(speed_up) && is_negative(speed_down) ) {
        _speed_down_cms = speed_down;
        _speed_up_cms = speed_up;
        _flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

/// set_max_accel_z - set the maximum vertical acceleration in cm/s/s
void AC_PosControl::set_max_accel_z(float accel_cmss)
{
    if (fabsf(_accel_z_cms - accel_cmss) > 1.0f) {
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
    float alt_change = alt_cm - _pos_target.z;

    // do not use z-axis desired velocity feed forward
    _flags.use_desvel_ff_z = false;

    // adjust desired alt if motors have not hit their limits
    if ((alt_change < 0 && !_motors.limit.throttle_lower) || (alt_change > 0 && !_motors.limit.throttle_upper)) {
        if (!is_zero(dt)) {
            float climb_rate_cms = constrain_float(alt_change / dt, _speed_down_cms, _speed_up_cms);
            _pos_target.z += climb_rate_cms * dt;
            _vel_desired.z = climb_rate_cms;  // recorded for reporting purposes
        }
    } else {
        // recorded for reporting purposes
        _vel_desired.z = 0.0f;
    }

    // do not let target get too far from current altitude
    float curr_alt = _inav.get_altitude();
    _pos_target.z = constrain_float(_pos_target.z, curr_alt - _leash_down_z, curr_alt + _leash_up_z);
}

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms < 0 && (!_motors.limit.throttle_lower || force_descend)) || (climb_rate_cms > 0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
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

    float accel_z_max = MIN(accel_z_cms, safe_sqrt(2.0f * fabsf(_vel_desired.z - climb_rate_cms) * jerk_z));

    _accel_last_z_cms += jerk_z * dt;
    _accel_last_z_cms = MIN(accel_z_max, _accel_last_z_cms);

    float vel_change_limit = _accel_last_z_cms * dt;
    _vel_desired.z = constrain_float(climb_rate_cms, _vel_desired.z - vel_change_limit, _vel_desired.z + vel_change_limit);
    _flags.use_desvel_ff_z = true;

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((_vel_desired.z < 0 && (!_motors.limit.throttle_lower || force_descend)) || (_vel_desired.z > 0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
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
    _accel_desired.z = 0.0f;
    _accel_last_z_cms = 0.0f;
    _flags.reset_rate_to_accel_z = true;
    _pid_accel_z.set_integrator((throttle_setting - _motors.get_throttle_hover()) * 1000.0f);
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _pid_accel_z.reset_filter();
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

    float linear_distance; // half the distance we swap between linear and sqrt and the distance we offset sqrt
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
    linear_velocity = _accel_z_cms / _p_pos_z.kP();

    if (fabsf(curr_vel_z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        stopping_point.z = curr_pos_z + curr_vel_z / _p_pos_z.kP();
    } else {
        linear_distance = _accel_z_cms / (2.0f * _p_pos_z.kP() * _p_pos_z.kP());
        if (curr_vel_z > 0) {
            stopping_point.z = curr_pos_z + (linear_distance + curr_vel_z * curr_vel_z / (2.0f * _accel_z_cms));
        } else {
            stopping_point.z = curr_pos_z - (linear_distance + curr_vel_z * curr_vel_z / (2.0f * _accel_z_cms));
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
    _pid_accel_z.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f);

    // initialise ekf reset handler
    init_ekf_z_reset();
}

// is_active_z - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_z() const
{
    return ((AP_HAL::micros64() - _last_update_z_us) <= POSCONTROL_ACTIVE_TIMEOUT_US);
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // check time since last cast
    const uint64_t now_us = AP_HAL::micros64();
    if (now_us - _last_update_z_us > POSCONTROL_ACTIVE_TIMEOUT_US) {
        _flags.reset_rate_to_accel_z = true;
        _pid_accel_z.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f);
        _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
        _pid_accel_z.reset_filter();
    }
    _last_update_z_us = now_us;

    // check for ekf altitude reset
    check_for_ekf_z_reset();

    // check if leash lengths need to be recalculated
    calc_leash_length_z();

    // call z-axis position controller
    run_z_controller();
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by update_z_controller if z-axis speed or accelerations are changed
void AC_PosControl::calc_leash_length_z()
{
    if (_flags.recalc_leash_z) {
        _leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, _p_pos_z.kP());
        _leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, _p_pos_z.kP());
        _flags.recalc_leash_z = false;
    }
}

// run position control for Z axis
// target altitude should be set with one of these functions: set_alt_target, set_target_to_stopping_point_z, init_takeoff
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void AC_PosControl::run_z_controller()
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
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_z_cms, _dt);

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

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();

    // TODO: remove velocity derivative calculation
    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
        if (!_flags.freeze_ff_z) {
            _accel_desired.z = (_vel_target.z - _vel_last.z) / _dt;
        } else {
            // stop the feed forward being calculated during a known discontinuity
            _flags.freeze_ff_z = false;
        }
    } else {
        _accel_desired.z = 0.0f;
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

    _accel_target.z = _p_vel_z.get_p(_vel_error.z);

    _accel_target.z += _accel_desired.z;


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float z_accel_meas;         // actual acceleration

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }

    float thr_out;
    if (_vibe_comp_enabled) {
        _flags.freeze_ff_z = true;
        _accel_desired.z = 0.0f;
        const float thr_per_accelz_cmss = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
        // during vibration compensation use feed forward with manually calculated gain
        // ToDo: clear pid_info P, I and D terms for logging
        if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_z.get_i()) && is_negative(_vel_error.z)) || (is_negative(_pid_accel_z.get_i()) && is_positive(_vel_error.z)))) {
            _pid_accel_z.set_integrator(_pid_accel_z.get_i() + _dt * thr_per_accelz_cmss * 1000.0f * _vel_error.z * _p_vel_z.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
        }
        thr_out = POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accelz_cmss * _accel_target.z + _pid_accel_z.get_i() * 0.001f;
    } else {
        thr_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
    }
    thr_out += _motors.get_throttle_hover();

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

    // _speed_down_cms is checked to be non-zero when set
    float error_ratio = _vel_error.z/_speed_down_cms;

    _vel_z_control_ratio += _dt*0.1f*(0.5-error_ratio);
    _vel_z_control_ratio = constrain_float(_vel_z_control_ratio, 0.0f, 2.0f);
}

///
/// lateral position controller
///

/// set_max_accel_xy - set the maximum horizontal acceleration in cm/s/s
void AC_PosControl::set_max_accel_xy(float accel_cmss)
{
    if (fabsf(_accel_cms - accel_cmss) > 1.0f) {
        _accel_cms = accel_cmss;
        _flags.recalc_leash_xy = true;
        calc_leash_length_xy();
    }
}

/// set_max_speed_xy - set the maximum horizontal speed maximum in cm/s
void AC_PosControl::set_max_speed_xy(float speed_cms)
{
    if (fabsf(_speed_cms - speed_cms) > 1.0f) {
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
///     set_max_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
    const Vector3f curr_pos = _inav.get_position();
    Vector3f curr_vel = _inav.get_velocity();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;	    // the distance within the vehicle can stop
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
    linear_velocity = _accel_cms / kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
        stopping_dist = vel_total / kP;
    } else {
        linear_distance = _accel_cms / (2.0f * kP * kP);
        stopping_dist = linear_distance + (vel_total * vel_total) / (2.0f * _accel_cms);
    }

    // constrain stopping distance
    stopping_dist = constrain_float(stopping_dist, 0, _leash);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_point.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_point.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_distance_to_target - get horizontal distance to target position in cm
float AC_PosControl::get_distance_to_target() const
{
    return norm(_pos_error.x, _pos_error.y);
}

/// get_bearing_to_target - get bearing to target position in centi-degrees
int32_t AC_PosControl::get_bearing_to_target() const
{
    return get_bearing_cd(_inav.get_position(), _pos_target);
}

// is_active_xy - returns true if the xy position controller has been run very recently
bool AC_PosControl::is_active_xy() const
{
    return ((AP_HAL::micros64() - _last_update_xy_us) <= POSCONTROL_ACTIVE_TIMEOUT_US);
}

/// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
float AC_PosControl::get_lean_angle_max_cd() const
{
    if (is_zero(_lean_angle_max)) {
        return _attitude_control.lean_angle_max();
    }
    return _lean_angle_max * 100.0f;
}

/// init_xy_controller - initialise the xy controller
///     this should be called after setting the position target and the desired velocity and acceleration
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the xy target
void AC_PosControl::init_xy_controller()
{
    // set roll, pitch lean angle targets to current attitude
    // todo: this should probably be based on the desired attitude not the current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    // initialise I terms from lean angles
    _pid_vel_xy.reset_filter();
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pid_vel_xy.set_integrator(_accel_target - _accel_desired);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;

    // initialise ekf xy reset handler
    init_ekf_xy_reset();
}

/// standby_xyz_reset - resets I terms and removes position error
///     This function will let Loiter and Alt Hold continue to operate
///     in the event that the flight controller is in control of the
///     aircraft when in standby.
void AC_PosControl::standby_xyz_reset()
{
    // Set _pid_accel_z integrator to zero.
    _pid_accel_z.set_integrator(0.0f);

    // Set the target position to the current pos.
    _pos_target = _inav.get_position();

    // Set _pid_vel_xy integrators and derivative to zero.
    _pid_vel_xy.reset_filter();

    // initialise ekf xy reset handler
    init_ekf_xy_reset();
}

/// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
void AC_PosControl::update_xy_controller()
{
    // compute dt
    const uint64_t now_us = AP_HAL::micros64();
    float dt = (now_us - _last_update_xy_us) * 1.0e-6f;

    // sanity check dt
    if (dt >= POSCONTROL_ACTIVE_TIMEOUT_US * 1.0e-6f) {
        dt = 0.0f;
    }

    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // translate any adjustments from pilot to loiter target
    desired_vel_to_pos(dt);

    // run horizontal position controller
    run_xy_controller(dt);

    // update xy update time
    _last_update_xy_us = now_us;
}

float AC_PosControl::time_since_last_xy_update() const
{
    const uint64_t now_us = AP_HAL::micros64();
    return (now_us - _last_update_xy_us) * 1.0e-6f;
}

// write log to dataflash
void AC_PosControl::write_log()
{
    const Vector3f &pos_target = get_pos_target();
    const Vector3f &vel_target = get_vel_target();
    const Vector3f &accel_target = get_accel_target();
    const Vector3f &position = _inav.get_position();
    const Vector3f &velocity = _inav.get_velocity();
    float accel_x, accel_y;
    lean_angles_to_accel(accel_x, accel_y);

    AP::logger().Write("PSC",
                       "TimeUS,TPX,TPY,PX,PY,TVX,TVY,VX,VY,TAX,TAY,AX,AY",
                       "smmmmnnnnoooo",
                       "F000000000000",
                       "Qffffffffffff",
                       AP_HAL::micros64(),
                       double(pos_target.x * 0.01f),
                       double(pos_target.y * 0.01f),
                       double(position.x * 0.01f),
                       double(position.y * 0.01f),
                       double(vel_target.x * 0.01f),
                       double(vel_target.y * 0.01f),
                       double(velocity.x * 0.01f),
                       double(velocity.y * 0.01f),
                       double(accel_target.x * 0.01f),
                       double(accel_target.y * 0.01f),
                       double(accel_x * 0.01f),
                       double(accel_y * 0.01f));
}

/// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
void AC_PosControl::init_vel_controller_xyz()
{
    // set roll, pitch lean angle targets to current attitude
    _roll_target = _ahrs.roll_sensor;
    _pitch_target = _ahrs.pitch_sensor;

    _pid_vel_xy.reset_filter();
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pid_vel_xy.set_integrator(_accel_target);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;

    // set target position
    const Vector3f& curr_pos = _inav.get_position();
    set_xy_target(curr_pos.x, curr_pos.y);
    set_alt_target(curr_pos.z);

    // move current vehicle velocity into feed forward velocity
    const Vector3f& curr_vel = _inav.get_velocity();
    set_desired_velocity(curr_vel);

    // set vehicle acceleration to zero
    set_desired_accel_xy(0.0f, 0.0f);

    // initialise ekf reset handlers
    init_ekf_xy_reset();
    init_ekf_z_reset();
}

/// update_velocity_controller_xy - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xy() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xy()
{
    // capture time since last iteration
    const uint64_t now_us = AP_HAL::micros64();
    float dt = (now_us - _last_update_xy_us) * 1.0e-6f;

    // sanity check dt
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // apply desired velocity request to position target
    // TODO: this will need to be removed and added to the calling function.
    desired_vel_to_pos(dt);

    // run position controller
    run_xy_controller(dt);

    // update xy update time
    _last_update_xy_us = now_us;
}

/// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xyz() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xyz()
{
    update_vel_controller_xy();

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
    // todo: remove _flags.recalc_leash_xy or don't call this function after each variable change.
    if (_flags.recalc_leash_xy) {
        _leash = calc_leash_length(_speed_cms, _accel_cms, _p_pos_xy.kP());
        _flags.recalc_leash_xy = false;
    }
}

/// move velocity target using desired acceleration
void AC_PosControl::desired_accel_to_vel(float nav_dt)
{
    // range check nav_dt
    if (nav_dt < 0) {
        return;
    }

    // update target velocity
    if (_flags.reset_desired_vel_to_pos) {
        _flags.reset_desired_vel_to_pos = false;
    } else {
        _vel_desired.x += _accel_desired.x * nav_dt;
        _vel_desired.y += _accel_desired.y * nav_dt;
    }
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos(float nav_dt)
{
    // range check nav_dt
    if (nav_dt < 0) {
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

/// run horizontal position controller correcting position and velocity
///     converts position (_pos_target) to target velocity (_vel_target)
///     desired velocity (_vel_desired) is combined into final target velocity
///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::run_xy_controller(float dt)
{
    float ekfGndSpdLimit, ekfNavVelGainScaler;
    AP::ahrs_navekf().getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    Vector3f curr_pos = _inav.get_position();
    float kP = ekfNavVelGainScaler * _p_pos_xy.kP(); // scale gains to compensate for noisy optical flow measurement in the EKF

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0f;
        _vel_target.y = 0.0f;
    } else {
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        // Constrain _pos_error and target position
        // Constrain the maximum length of _vel_target to the maximum position correction velocity
        // TODO: replace the leash length with a user definable maximum position correction
        if (limit_vector_length(_pos_error.x, _pos_error.y, _leash)) {
            _pos_target.x = curr_pos.x + _pos_error.x;
            _pos_target.y = curr_pos.y + _pos_error.y;
        }

        _vel_target = sqrt_controller(_pos_error, kP, _accel_cms);
    }

    // add velocity feed-forward
    _vel_target.x += _vel_desired.x;
    _vel_target.y += _vel_desired.y;

    // the following section converts desired velocities in lat/lon directions to accelerations in lat/lon frame

    Vector2f accel_target, vel_xy_p, vel_xy_i, vel_xy_d;

    // check if vehicle velocity is being overridden
    if (_flags.vehicle_horiz_vel_override) {
        _flags.vehicle_horiz_vel_override = false;
    } else {
        _vehicle_horiz_vel.x = _inav.get_velocity().x;
        _vehicle_horiz_vel.y = _inav.get_velocity().y;
    }

    // calculate velocity error
    _vel_error.x = _vel_target.x - _vehicle_horiz_vel.x;
    _vel_error.y = _vel_target.y - _vehicle_horiz_vel.y;
    // TODO: constrain velocity error and velocity target

    // call pi controller
    _pid_vel_xy.set_input(_vel_error);

    // get p
    vel_xy_p = _pid_vel_xy.get_p();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    // TODO: move limit handling into the PI and PID controller
    if (!_limit.accel_xy && !_motors.limit.throttle_upper) {
        vel_xy_i = _pid_vel_xy.get_i();
    } else {
        vel_xy_i = _pid_vel_xy.get_i_shrink();
    }

    // get d
    vel_xy_d = _pid_vel_xy.get_d();

    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    accel_target.x = (vel_xy_p.x + vel_xy_i.x + vel_xy_d.x) * ekfNavVelGainScaler;
    accel_target.y = (vel_xy_p.y + vel_xy_i.y + vel_xy_d.y) * ekfNavVelGainScaler;

    // reset accel to current desired acceleration
    if (_flags.reset_accel_to_lean_xy) {
        _accel_target_filter.reset(Vector2f(accel_target.x, accel_target.y));
        _flags.reset_accel_to_lean_xy = false;
    }

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(MIN(_accel_xy_filt_hz, 5.0f * ekfNavVelGainScaler));
    _accel_target_filter.apply(accel_target, dt);

    // pass the correction acceleration to the target acceleration output
    _accel_target.x = _accel_target_filter.get().x;
    _accel_target.y = _accel_target_filter.get().y;

    // Add feed forward into the target acceleration output
    _accel_target.x += _accel_desired.x;
    _accel_target.y += _accel_desired.y;

    // the following section converts desired accelerations provided in lat/lon frame to roll/pitch angles

    // limit acceleration using maximum lean angles
    float angle_max = MIN(_attitude_control.get_althold_lean_angle_max(), get_lean_angle_max_cd());
    float accel_max = MIN(GRAVITY_MSS * 100.0f * tanf(ToRad(angle_max * 0.01f)), POSCONTROL_ACCEL_XY_MAX);
    _limit.accel_xy = limit_vector_length(_accel_target.x, _accel_target.y, accel_max);

    // update angle targets that will be passed to stabilize controller
    accel_to_lean_angles(_accel_target.x, _accel_target.y, _roll_target, _pitch_target);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const
{
    float accel_right, accel_forward;

    // rotate accelerations into body forward-right frame
    // todo: this should probably be based on the desired heading not the current heading
    accel_forward = accel_x_cmss * _ahrs.cos_yaw() + accel_y_cmss * _ahrs.sin_yaw();
    accel_right = -accel_x_cmss * _ahrs.sin_yaw() + accel_y_cmss * _ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target = atanf(-accel_forward / (GRAVITY_MSS * 100.0f)) * (18000.0f / M_PI);
    float cos_pitch_target = cosf(pitch_target * M_PI / 18000.0f);
    roll_target = atanf(accel_right * cos_pitch_target / (GRAVITY_MSS * 100.0f)) * (18000.0f / M_PI);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    // todo: this should probably be based on the desired attitude not the current attitude
    accel_x_cmss = (GRAVITY_MSS * 100) * (-_ahrs.cos_yaw() * _ahrs.sin_pitch() * _ahrs.cos_roll() - _ahrs.sin_yaw() * _ahrs.sin_roll()) / MAX(_ahrs.cos_roll() * _ahrs.cos_pitch(), 0.5f);
    accel_y_cmss = (GRAVITY_MSS * 100) * (-_ahrs.sin_yaw() * _ahrs.sin_pitch() * _ahrs.cos_roll() + _ahrs.cos_yaw() * _ahrs.sin_roll()) / MAX(_ahrs.cos_roll() * _ahrs.cos_pitch(), 0.5f);
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
    if (speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    } else {
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f * kP * kP)) + (speed_cms * speed_cms / (2.0f * accel_cms));
    }

    // ensure leash is at least 1m long
    if (leash_length < POSCONTROL_LEASH_LENGTH_MIN) {
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
    if (reset_ms != 0 && reset_ms != _ekf_z_reset_ms) {
        shift_alt_target(-alt_shift * 100.0f);
        _ekf_z_reset_ms = reset_ms;
    }
}

/// limit vector to a given length, returns true if vector was limited
bool AC_PosControl::limit_vector_length(float& vector_x, float& vector_y, float max_length)
{
    float vector_length = norm(vector_x, vector_y);
    if ((vector_length > max_length) && is_positive(vector_length)) {
        vector_x *= (max_length / vector_length);
        vector_y *= (max_length / vector_length);
        return true;
    }
    return false;
}

/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector3f AC_PosControl::sqrt_controller(const Vector3f& error, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return Vector3f(error.x * p, error.y * p, error.z);
    }

    float linear_dist = second_ord_lim / sq(p);
    float error_length = norm(error.x, error.y);
    if (error_length > linear_dist) {
        float first_order_scale = safe_sqrt(2.0f * second_ord_lim * (error_length - (linear_dist * 0.5f))) / error_length;
        return Vector3f(error.x * first_order_scale, error.y * first_order_scale, error.z);
    } else {
        return Vector3f(error.x * p, error.y * p, error.z);
    }
}

bool AC_PosControl::pre_arm_checks(const char *param_prefix,
                                   char *failure_msg,
                                   const uint8_t failure_msg_len)
{
    // validate AC_P members:
    const struct {
        const char *pid_name;
        AC_P &p;
    } ps[] = {
        { "POSXY", get_pos_xy_p() },
        { "POSZ", get_pos_z_p() },
        { "VELZ", get_vel_z_p() },
    };
    for (uint8_t i=0; i<ARRAY_SIZE(ps); i++) {
        // all AC_P's must have a positive P value:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // z-axis acceleration control PID doesn't use FF, so P and I must be positive
    if (!is_positive(get_accel_z_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_accel_z_pid().kI())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_I must be > 0", param_prefix);
        return false;
    }

    return true;
}
