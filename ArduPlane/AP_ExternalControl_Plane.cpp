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

bool AP_ExternalControl_Plane::set_path_position_tangent_and_curvature(const Location &position_on_path,
    const Vector2f unit_path_tangent, const float path_curvature, const bool direction_is_ccw)
{
    //! @todo(srmainwaring) consolidate duplicate code in GCS_MAVLINK_Plane::handle_set_position_target_global_int.

    // only accept position updates when in GUIDED mode.
    if (!plane.control_mode->is_guided_mode()) {
        return false;
    }

    // update guided path 
    plane.mode_guided.handle_guided_path_request(
        position_on_path, unit_path_tangent, path_curvature, direction_is_ccw);

    // update adjust_altitude_target immediately rather than wait for the scheduler.
    plane.adjust_altitude_target();

    return true;
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
