/*
  external control library for copter
 */


#include "AP_ExternalControl_Copter.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Copter.h"

/*
  sets attitude commands
*/
bool AP_ExternalControl_Copter::set_attitude_target(Quaternion attitude_quat, Vector3f ang_vel_body, float thrust)
{
    if (!ready_for_external_control()) {
        return false;
    }

    // check if the message's thrust field should be interpreted as a climb rate or as thrust
    const bool use_thrust = copter.mode_guided.set_attitude_target_provides_thrust();

    float climb_rate_or_thrust;
    if (use_thrust) {
        // interpret thrust as thrust
        climb_rate_or_thrust = constrain_float(thrust, -1.0f, 1.0f);
    } else {
        // convert thrust to climb rate
        thrust = constrain_float(thrust, 0.0f, 1.0f);
        if (is_equal(thrust, 0.5f)) {
            climb_rate_or_thrust = 0.0f;
        } else if (thrust > 0.5f) {
            // climb at up to WPNAV_SPEED_UP
            climb_rate_or_thrust = (thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_or_thrust = (0.5f - thrust) * 2.0f * -copter.wp_nav->get_default_speed_down();
        }
    }

    copter.mode_guided.set_angle(attitude_quat, ang_vel_body,
                climb_rate_or_thrust, use_thrust);
                
    return true;
}

/*
  set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
  velocity is in earth frame, NED, m/s
*/
bool AP_ExternalControl_Copter::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }
    const float yaw_rate_cds = isnan(yaw_rate_rads)? 0: degrees(yaw_rate_rads)*100;

    // Copter velocity is positive if aircraft is moving up which is opposite the incoming NED frame.
    Vector3f velocity_NEU_ms {
        linear_velocity.x,
        linear_velocity.y,
        -linear_velocity.z };
    Vector3f velocity_up_cms = velocity_NEU_ms * 100;
    copter.mode_guided.set_velocity(velocity_up_cms, false, 0, !isnan(yaw_rate_rads), yaw_rate_cds);
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

bool AP_ExternalControl_Copter::ready_for_external_control()
{
    return copter.flightmode->in_guided_mode() && copter.motors->armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
