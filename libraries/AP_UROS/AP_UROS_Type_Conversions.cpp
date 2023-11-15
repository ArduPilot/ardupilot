#include "AP_UROS_Type_Conversions.h"
#if AP_UROS_ENABLED

#include <builtin_interfaces/msg/time.h>


uint64_t AP_UROS_Type_Conversions::time_u64_micros(const builtin_interfaces__msg__Time& ros_time)
{
    return (uint64_t(ros_time.sec) * 1000000ULL) + (ros_time.nanosec / 1000ULL);
}


#endif // AP_UROS_ENABLED
