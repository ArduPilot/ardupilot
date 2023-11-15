// Class for handling type conversions for UROS.

#pragma once

#if AP_UROS_ENABLED

#include <builtin_interfaces/msg/time.h>

class AP_UROS_Type_Conversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    static uint64_t time_u64_micros(const builtin_interfaces__msg__Time& ros_time);
};

#endif // AP_UROS_ENABLED
