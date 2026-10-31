// Class for handling external localization data.
// For historical reasons, it's called odometry to match AP_VisualOdom.

#pragma once

#include "AP_DDS_config.h"
#if AP_DDS_VISUALODOM_ENABLED

#include "geometry_msgs/msg/TransformStamped.h"
#include "tf2_msgs/msg/TFMessage.h"
#if AP_DDS_ODOM_SUB_ENABLED
#include "nav_msgs/msg/Odometry.h"
#endif // AP_DDS_ODOM_SUB_ENABLED
#include "AP_Math/vector3.h"
#include "AP_Math/quaternion.h"

class AP_DDS_External_Odom
{
public:

    // Handler for external position localization from TF
    static void handle_external_odom(const tf2_msgs_msg_TFMessage& msg);

#if AP_DDS_ODOM_SUB_ENABLED
    // Handler for external position and velocity localization from nav_msgs/Odometry
    static void handle_external_odom(const nav_msgs_msg_Odometry& msg);
#endif // AP_DDS_ODOM_SUB_ENABLED

    // Checks the child and parent frames match a set needed for external odom.
    // Since multiple different transforms can be sent, this validates the specific transform is
    // for odometry.
    static bool is_odometry_frame(const geometry_msgs_msg_TransformStamped& msg);

    // Helper to convert from ROS transform to AP datatypes
    // ros_transform is in ENU
    // translation is in NED
    static void convert_transform(const geometry_msgs_msg_Transform& ros_transform, Vector3f& translation, Quaternion& rotation);

};

#endif // AP_DDS_VISUALODOM_ENABLED
