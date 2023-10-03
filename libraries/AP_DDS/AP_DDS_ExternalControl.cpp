#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_ExternalControl/AP_ExternalControl.h>
#include <AP_ROS/AP_ROS_ExternalControl.h>

bool AP_DDS_External_Control::handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel)
{
    return AP_ROS_External_Control::handle_velocity_control(cmd_vel);
}

#endif // AP_DDS_ENABLED