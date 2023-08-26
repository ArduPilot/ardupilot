// Class for handling type conversions for DDS.

#pragma once

#if AP_DDS_ENABLED

#include "builtin_interfaces/msg/Time.h"

class AP_DDS_Type_Conversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    static uint64_t time_u64_micros(const builtin_interfaces_msg_Time& ros_time);
};

#endif // AP_DDS_ENABLED
