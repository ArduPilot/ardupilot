/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"
#include "quadplane.h"

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
  Sets the target airspeed.
*/
bool AP_ExternalControl_Plane::set_airspeed(const float airspeed)
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // The command is only valid in guided mode.
    if (plane.control_mode != &plane.mode_guided) {
        return false;
    }

    // Assume the user wanted maximum acceleration.
    const float acc_instant = 0.0;
    return plane.mode_guided.handle_change_airspeed(airspeed, acc_instant);
#else 
  return false;
#endif
}

/*
    Request gliding
*/
bool AP_ExternalControl_Plane::request_gliding(bool gliding_requested)
{
    // The command is only valid in guided mode.
    if (plane.control_mode != &plane.mode_guided) {
        return false;
    }

    // Assume we always want motor to launch.
    if (!plane.is_flying()) {
        return false;
    }

    // Don't allow gliding when in a VTOL mode.
#if HAL_QUADPLANE_ENABLED
    if (!plane.quadplane.in_assisted_flight()) {
        return false;
    }
#endif // HAL_QUADPLANE_ENABLED

    plane.TECS_controller.set_gliding_requested_flag(gliding_requested);
    return true;
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
