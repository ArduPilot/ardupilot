/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  Sets the target global position for a loiter point.
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}

/*
    Sets only the target yaw rate
    Yaw is in earth frame, NED [rad/s]
*/
bool AP_ExternalControl_Plane::set_yaw_rate(const float yaw_rate_rads)
{
    // TODO validate yaw rate is feasible for current dynamics
    return plane.set_target_yaw_rate(yaw_rate_rads);
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
