#include <AP_gtest.h>

#include <AP_DDS/AP_DDS_config.h>

#include <AP_DDS/AP_DDS_External_Odom.h>
#include "geometry_msgs/msg/TransformStamped.h"
#include <AP_HAL/AP_HAL.h>

#if AP_DDS_VISUALODOM_ENABLED

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(AP_DDS_EXTERNAL_ODOM, test_is_odometry_success)
{
    geometry_msgs_msg_TransformStamped msg {};

    strncpy(msg.header.frame_id, "odom", strlen("odom") + 1);
    strncpy(msg.child_frame_id, "base_link", strlen("base_link") + 1);
    ASSERT_TRUE(AP_DDS_External_Odom::is_odometry_frame(msg));

    strncpy(msg.header.frame_id, "invalid", strlen("invalid") + 1);
    strncpy(msg.child_frame_id, "base_link", strlen("base_link") + 1);
    ASSERT_FALSE(AP_DDS_External_Odom::is_odometry_frame(msg));

    strncpy(msg.header.frame_id, "odom", strlen("odom") + 1);
    strncpy(msg.child_frame_id, "invalid", strlen("invalid") + 1);
    ASSERT_FALSE(AP_DDS_External_Odom::is_odometry_frame(msg));

    strncpy(msg.header.frame_id, "odom_with_invalid_extra", strlen("odom_with_invalid_extra") + 1);
    strncpy(msg.child_frame_id, "base_link", strlen("base_link") + 1);
    ASSERT_FALSE(AP_DDS_External_Odom::is_odometry_frame(msg));

    strncpy(msg.header.frame_id, "odom", strlen("odom") + 1);
    strncpy(msg.child_frame_id, "base_link_with_invalid_extra", strlen("base_link_with_invalid_extra") + 1);
    ASSERT_FALSE(AP_DDS_External_Odom::is_odometry_frame(msg));

    strncpy(msg.header.frame_id, "x", strlen("x") + 1);
    strncpy(msg.child_frame_id, "base_link", strlen("base_link") + 1);
    ASSERT_FALSE(AP_DDS_External_Odom::is_odometry_frame(msg));
}

#endif // AP_DDS_VISUALODOM_ENABLED

AP_GTEST_MAIN()
