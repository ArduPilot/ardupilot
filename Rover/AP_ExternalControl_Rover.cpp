/*
  external control library for rover
 */


#include "AP_ExternalControl_Rover.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Rover.h"

/*
  set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw
  velocity is in earth frame, NED, m/s
*/
bool AP_ExternalControl_Rover::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }

    // rover is commanded in body-frame using FRD convention
    auto &ahrs = AP::ahrs();
    Vector3f linear_velocity_body = ahrs.earth_to_body(linear_velocity);

    const float target_speed = linear_velocity_body.x;
    const float turn_rate_cds = isnan(yaw_rate_rads)? 0: degrees(yaw_rate_rads)*100;

    rover.mode_guided.set_desired_turn_rate_and_speed(turn_rate_cds, target_speed);
    return true;
}

bool AP_ExternalControl_Rover::ready_for_external_control()
{
    return rover.control_mode->in_guided_mode() && rover.arming.is_armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
