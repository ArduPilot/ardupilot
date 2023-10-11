#include "AP_DDS_External_Odom.h"
#include "AP_DDS_Type_Conversions.h"

#if AP_DDS_VISUALODOM_ENABLED

#include <AP_ROS/AP_ROS_ExternalOdom.h>

void AP_DDS_External_Odom::handle_external_odom(const tf2_msgs_msg_TFMessage& msg)
{
    AP_ROS_External_Odom::handle_external_odom(msg);
}

bool AP_DDS_External_Odom::is_odometry_frame(const geometry_msgs_msg_TransformStamped& msg)
{
    return AP_ROS_External_Odom::is_odometry_frame(msg);
}

void AP_DDS_External_Odom::convert_transform(const geometry_msgs_msg_Transform& ros_transform, Vector3f& translation, Quaternion& rotation)
{
    AP_ROS_External_Odom::convert_transform(ros_transform, translation, rotation);
}

#endif // AP_DDS_VISUALODOM_ENABLED
