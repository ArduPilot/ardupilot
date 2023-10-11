#pragma once

#if AP_UROS_ENABLED
#include <geometry_msgs/msg/twist_stamped.h>

class AP_UROS_External_Control
{
public:
    static bool handle_velocity_control(const geometry_msgs__msg__TwistStamped& cmd_vel);
};
#endif // AP_UROS_ENABLED
