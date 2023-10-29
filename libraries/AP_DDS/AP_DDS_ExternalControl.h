#pragma once

#if AP_DDS_ENABLED
#include "geometry_msgs/msg/TwistStamped.h"

class AP_DDS_External_Control
{
public:
    static bool handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel);
};
#endif // AP_DDS_ENABLED
