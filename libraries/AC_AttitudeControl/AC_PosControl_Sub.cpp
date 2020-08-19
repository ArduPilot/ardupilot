#include "AC_PosControl_Sub.h"

AC_PosControl_Sub::AC_PosControl_Sub(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                     const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    AC_PosControl(ahrs, inav, motors, attitude_control),
    _alt_max(0.0f),
    _alt_min(0.0f)
{}

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl_Sub::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms<0 && (!_motors.limit.throttle_lower || force_descend)) || (climb_rate_cms>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += climb_rate_cms * dt;
    }

    // do not let target alt get above limit
    if (_alt_max < 100 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _limit.pos_up = true;
    }

    // do not let target alt get below limit
    if (_alt_min < 0 && _alt_min < _alt_max && _pos_target.z < _alt_min) {
        _pos_target.z = _alt_min;
        _limit.pos_down = true;
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
void AC_PosControl_Sub::set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
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

    // do not let target alt get above limit
    if (_alt_max < 100 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _limit.pos_up = true;
        // decelerate feed forward to zero
        _vel_desired.z = constrain_float(0.0f, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    }

    // do not let target alt get below limit
    if (_alt_min < 0 && _alt_min < _alt_max && _pos_target.z < _alt_min) {
        _pos_target.z = _alt_min;
        _limit.pos_down = true;
        // decelerate feed forward to zero
        _vel_desired.z = constrain_float(0.0f, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    }
}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl_Sub::relax_alt_hold_controllers()
{
    _pos_target.z = _inav.get_altitude();
    _vel_desired.z = 0.0f;
    _flags.use_desvel_ff_z = false;
    _vel_target.z = _inav.get_velocity_z();
    _vel_last.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _accel_last_z_cms = 0.0f;
    _flags.reset_rate_to_accel_z = true;
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _pid_accel_z.reset_filter();
}
