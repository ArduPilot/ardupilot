
/*
  external control library for plane
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Common/Location.h>

class AP_ExternalControl_Plane : public AP_ExternalControl {
public:
    /*
        Sets the target global position for a loiter point.
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;

    /*
        Set the target setpoint for path guidance: the closest position on the
        path, the unit tangent to the path, and path curvature and direction.
    */
    bool set_path_position_tangent_and_curvature(const Location &position_on_path,
        Vector2f unit_path_tangent, const float path_curvature, const bool direction_is_ccw) override WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
