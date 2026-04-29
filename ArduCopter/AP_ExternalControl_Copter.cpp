/*
  external control library for copter
 */


#include "AP_ExternalControl_Copter.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Copter.h"

/*
  set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
  velocity is in earth frame, NED, m/s
*/
bool AP_ExternalControl_Copter::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity_ned_ms, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }
    const float checked_yaw_rate_rad = isnan(yaw_rate_rads)? 0: yaw_rate_rads;

    // Copter velocity is positive if aircraft is moving up which is opposite the incoming NED frame.
    copter.mode_guided.set_vel_NED_ms(linear_velocity_ned_ms, false, 0, !isnan(yaw_rate_rads), checked_yaw_rate_rad);
    return true;
}

bool AP_ExternalControl_Copter::set_global_position(const Location& loc)
{
    // Check if copter is ready for external control and returns false if it is not.
    if (!ready_for_external_control()) {
        return false;
    }
    return copter.set_target_location(loc);
}

bool AP_ExternalControl_Copter::set_local_position(const Vector3f &position_ned_m, const Vector3f &velocity_ned_ms,
        const Vector3f &acceleration_ned_mss, float yaw_ned_rad, float yaw_rate_ned_rads, bool yaw_relative)
{
    if (!ready_for_external_control()) {
        return false;
    }

    const bool pos_valid = !position_ned_m.is_nan();
    const bool vel_valid = !velocity_ned_ms.is_nan();
    const bool accel_valid = !acceleration_ned_mss.is_nan();
    const bool yaw_valid = !isnan(yaw_ned_rad);
    const bool yaw_rate_valid = !isnan(yaw_rate_ned_rads);

    Vector3p pos_ned_p;
    if (pos_valid) {
        pos_ned_p = position_ned_m.topostype();
    }

    const Vector3f acceleration_mss_checked = accel_valid ? acceleration_ned_mss : Vector3f();
    const float yaw_checked = yaw_valid ? yaw_ned_rad : 0.0f;
    const float yaw_rate_checked = yaw_rate_valid ? yaw_rate_ned_rads : 0.0f;
    if (pos_valid && vel_valid) {
        // Position & velocity valid
        return copter.mode_guided.set_pos_vel_accel_NED_m(
                   pos_ned_p, velocity_ned_ms, acceleration_mss_checked,
                   yaw_valid, yaw_checked,
                   yaw_rate_valid, yaw_rate_checked,
                   yaw_relative);
    } else if (!pos_valid && vel_valid) {
        // Velocity & accel only
        copter.mode_guided.set_vel_accel_NED_m(
            velocity_ned_ms, acceleration_mss_checked,
            yaw_valid, yaw_checked,
            yaw_rate_valid, yaw_rate_checked,
            yaw_relative);
        return true;
    } else if (!pos_valid && !vel_valid && accel_valid) {
        // Accel only
        copter.mode_guided.set_accel_NED_mss(
            acceleration_mss_checked,
            yaw_valid, yaw_checked,
            yaw_rate_valid, yaw_rate_checked,
            yaw_relative);
        return true;
    } else if (pos_valid && !vel_valid && !accel_valid) {
        // Position only
        return copter.mode_guided.set_pos_NED_m(
                   pos_ned_p,
                   yaw_valid, yaw_checked,
                   yaw_rate_valid, yaw_rate_checked,
                   yaw_relative, false);
    } else {
        return false;
    }
}

bool AP_ExternalControl_Copter::ready_for_external_control()
{
    return copter.flightmode->in_guided_mode() && copter.motors->armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
