// Class for handling external localization data.
// For historical reasons, it's called odometry to match AP_VisualOdom.

#pragma once

#include "AP_UROS_config.h"
#if AP_UROS_VISUALODOM_ENABLED

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

#include <AP_Math/vector3.h>
#include <AP_Math/quaternion.h>

class AP_UROS_External_Odom
{
public:

    // Handler for external position localization
    static void handle_external_odom(const tf2_msgs__msg__TFMessage& msg);

    // Checks the child and parent frames match a set needed for external odom.
    // Since multiple different transforms can be sent, this validates the specific transform is
    // for odometry.
    static bool is_odometry_frame(const geometry_msgs__msg__TransformStamped& msg);

    // Helper to convert from ROS transform to AP datatypes
    // ros_transform is in ENU
    // translation is in NED
    static void convert_transform(const geometry_msgs__msg__Transform& ros_transform, Vector3f& translation, Quaternion& rotation);

};

#endif // AP_UROS_VISUALODOM_ENABLED
