#include "AP_DDS_Type_Conversions.h"
#if AP_DDS_ENABLED

#include "builtin_interfaces/msg/Time.h"


uint64_t AP_DDS_Type_Conversions::time_u64_micros(const builtin_interfaces_msg_Time& ros_time)
{
    return (uint64_t(ros_time.sec) * 1000000ULL) + (ros_time.nanosec / 1000ULL);
}


#endif // AP_DDS_ENABLED
