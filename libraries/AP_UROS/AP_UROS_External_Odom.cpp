#include "AP_UROS_External_Odom.h"
#include "AP_UROS_Type_Conversions.h"

#if AP_UROS_VISUALODOM_ENABLED

#include <AP_ROS/AP_ROS_ExternalOdom.h>

void AP_UROS_External_Odom::handle_external_odom(const tf2_msgs__msg__TFMessage& msg)
{
    AP_ROS_External_Odom::handle_external_odom(msg);
}

bool AP_UROS_External_Odom::is_odometry_frame(const geometry_msgs__msg__TransformStamped& msg)
{
    return AP_ROS_External_Odom::is_odometry_frame(msg);
}

void AP_UROS_External_Odom::convert_transform(const geometry_msgs__msg__Transform& ros_transform, Vector3f& translation, Quaternion& rotation)
{
    AP_ROS_External_Odom::convert_transform(ros_transform, translation, rotation);
}

#endif // AP_UROS_VISUALODOM_ENABLED
