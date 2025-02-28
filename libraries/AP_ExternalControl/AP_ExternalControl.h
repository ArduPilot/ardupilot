/*
  external control library for MAVLink, DDS and scripting
 */

#pragma once

#include "AP_ExternalControl_config.h"

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

class AP_ExternalControl
{
public:

    AP_ExternalControl();
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    virtual bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) WARN_IF_UNUSED {
        return false;
    }

    /*
        Sets the target global position with standard guided mode behavior.
    */
    virtual bool set_global_position(const Location& loc) WARN_IF_UNUSED {
        return false;
    }

    /*
        Arm the vehicle
    */
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks) WARN_IF_UNUSED;

    /*
        Disarm the vehicle
    */
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks) WARN_IF_UNUSED;

    /*
        Set the target setpoint for path guidance: the closest position on the
        path, the unit tangent to the path, and path curvature and direction.
    */
    virtual bool set_path_position_tangent_and_curvature(const Location &position_on_path,
            Vector2f unit_path_tangent, const float path_curvature, const bool direction_is_ccw) WARN_IF_UNUSED {
        return false;
    }

    static AP_ExternalControl *get_singleton(void) WARN_IF_UNUSED {
        return singleton;
    }
protected:
    ~AP_ExternalControl() {}

private:
    static AP_ExternalControl *singleton;
};

namespace AP
{
AP_ExternalControl *externalcontrol();
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
