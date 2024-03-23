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
bool AP_ExternalControl_Copter::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, const float yaw_rate_rads)
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

bool AP_ExternalControl_Copter::ready_for_external_control()
{
    return copter.flightmode->in_guided_mode() && copter.motors->armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
