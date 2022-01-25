#include "AC_PosControl_Sub.h"

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

    // calculated increased maximum acceleration and jerk if over speed
    float accel_max_z_cmss = _accel_max_z_cmss * calculate_overspeed_gain();
    float jerk_max_xy_cmsss = _jerk_max_xy_cmsss * calculate_overspeed_gain();

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(_pos_target.z, _vel_desired.z, _accel_desired.z, _dt, _limit_vector.z, _p_pos_z.get_error(), _pid_vel_z.get_error());

    // prevent altitude target from breeching altitude limits
    if (is_negative(_alt_min) && _alt_min < _alt_max && _alt_max < 100 && _pos_target.z < _alt_min) {
        _pos_target.z = constrain_float(_pos_target.z, _alt_min, _alt_max);
    }

    shape_vel_accel(vel, accel,
        _vel_desired.z, _accel_desired.z,
        -accel_max_z_cmss, accel_max_z_cmss,
        jerk_max_xy_cmsss, _dt, limit_output);

    update_vel_accel(vel, accel, _dt, 0.0, 0.0);
}
