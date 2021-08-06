#include "AC_PosControl_Sub.h"

AC_PosControl_Sub::AC_PosControl_Sub(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                     const AP_Motors& motors, AC_AttitudeControl& attitude_control, float dt) :
    AC_PosControl(ahrs, inav, motors, attitude_control, dt),
    _alt_max(0.0f),
    _alt_min(0.0f)
{}

/// input_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
void AC_PosControl_Sub::input_vel_accel_z(float &vel, const float accel, bool force_descend, bool limit_output)
{
    // check for ekf z position reset
    handle_ekf_z_reset();

    // limit desired velocity to prevent breeching altitude limits
    if (_alt_min < 0 && _alt_min < _alt_max && _alt_max < 100 && _pos_target.z < _alt_min) {
        vel = constrain_float(vel,
            sqrt_controller(_alt_min-_pos_target.z, 0.0f, _accel_max_z_cmss, 0.0f),
            sqrt_controller(_alt_max-_pos_target.z, 0.0f, _accel_max_z_cmss, 0.0f));
    }

    // calculated increased maximum acceleration if over speed
    float accel_z_cms = _accel_max_z_cmss;
    if (_vel_desired.z < _vel_max_down_cms && !is_zero(_vel_max_down_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _vel_max_down_cms;
    }
    if (_vel_desired.z > _vel_max_up_cms && !is_zero(_vel_max_up_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _vel_max_up_cms;
    }

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_target.z, _vel_desired.z, _accel_desired.z, _dt, _limit_vector.z);

    // prevent altitude target from breeching altitude limits
    if (is_negative(_alt_min) && _alt_min < _alt_max && _alt_max < 100 && _pos_target.z < _alt_min) {
        _pos_target.z = constrain_float(_pos_target.z, _alt_min, _alt_max);
    }

    shape_vel_accel(vel, accel,
        _vel_desired.z, _accel_desired.z,
        -accel_z_cms, accel_z_cms,
        _jerk_xy_max, _dt, limit_output);

    update_vel_accel(vel, accel, _dt, _limit_vector.z);
}
