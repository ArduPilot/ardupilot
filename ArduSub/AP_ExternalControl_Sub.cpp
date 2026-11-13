/*
  external control library for sub
 */

#include "AP_ExternalControl_Sub.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Sub.h"

/*
  set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
  velocity is in earth frame, NED, m/s
*/
bool AP_ExternalControl_Sub::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity_ned_ms, float yaw_rate_rads)
{
    if (!ready_for_external_control()) {
        return false;
    }

    // Sub's guided velocity controller expects NEU frame in cm/s.
    // Flip z (NED down-positive -> NEU up-positive) and scale m/s -> cm/s.
    const Vector3f velocity_neu_cms {
        linear_velocity_ned_ms.x * 100.0f,
        linear_velocity_ned_ms.y * 100.0f,
        -linear_velocity_ned_ms.z * 100.0f
    };

    const bool use_yaw_rate = !isnan(yaw_rate_rads);
    const float yaw_rate_cds = use_yaw_rate ? degrees(yaw_rate_rads) * 100.0f : 0.0f;

    sub.mode_guided.guided_set_velocity(velocity_neu_cms, false, 0, use_yaw_rate, yaw_rate_cds, false);
    return true;
}

bool AP_ExternalControl_Sub::set_global_position(const Location& loc)
{
    if (!ready_for_external_control()) {
        return false;
    }
    return sub.mode_guided.guided_set_destination(loc);
}

bool AP_ExternalControl_Sub::ready_for_external_control()
{
    return sub.flightmode->in_guided_mode() && sub.arming.is_armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
