#include <AP_gtest.h>

#include <AP_DDS/AP_DDS_Type_Conversions.h>
#include "builtin_interfaces/msg/Time.h"
#include <AP_HAL/AP_HAL.h>


const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(AP_DDS_TYPE_CONVERSIONS, test_time_u64_micros)
{
    builtin_interfaces_msg_Time ros_time {};
    ASSERT_EQ(AP_DDS_Type_Conversions::time_u64_micros(ros_time), 0UL);

    ros_time.sec = 5;
    ASSERT_EQ(AP_DDS_Type_Conversions::time_u64_micros(ros_time), uint64_t(5E6));

    ros_time.nanosec = 1000;
    const uint64_t expected5 = uint64_t(5E6) + 1; 
    ASSERT_EQ(AP_DDS_Type_Conversions::time_u64_micros(ros_time), expected5);

    ros_time.sec = 7 * 24 * 60 * 60; // 1 week of runtime
    ros_time.nanosec = 1000;
    const uint64_t expected_long_runtime = uint64_t(ros_time.sec) * 1000000 + 1; 
    ASSERT_EQ(AP_DDS_Type_Conversions::time_u64_micros(ros_time), expected_long_runtime);   
}

AP_GTEST_MAIN()
