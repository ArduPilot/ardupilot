#if AP_UROS_ENABLED

#include "AP_UROS_ExternalControl.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_ExternalControl/AP_ExternalControl.h>
#include <AP_ROS/AP_ROS_ExternalControl.h>

bool AP_UROS_External_Control::handle_velocity_control(const geometry_msgs__msg__TwistStamped& cmd_vel)
{
    return AP_ROS_External_Control::handle_velocity_control(cmd_vel);
}

#endif // AP_UROS_ENABLED
