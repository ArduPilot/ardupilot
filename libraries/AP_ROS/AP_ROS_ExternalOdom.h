#pragma once

#include <AP_Math/quaternion.h>
#include <AP_Math/vector3.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_ROS_TypeConversions.h"

class AP_ROS_External_Odom
{
public:
    // Handler for external position localization
    template <typename TFMessage>
    static void handle_external_odom(const TFMessage& msg);

    // Checks the child and parent frames match a set needed for external odom.
    // Since multiple different transforms can be sent, this validates the specific transform is
    // for odometry.
    template <typename TransformStamped>
    static bool is_odometry_frame(const TransformStamped& msg);

    // Helper to convert from ROS transform to AP datatypes
    // ros_transform is in ENU
    // translation is in NED
    template <typename Transform>
    static void convert_transform(const Transform& ros_transform, Vector3f& translation, Quaternion& rotation);

};

template <typename TFMessage>
void AP_ROS_External_Odom::handle_external_odom(const TFMessage& msg)
{
    auto *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    for (size_t i = 0; i < transforms_size<TFMessage>(msg); i++) {
        const auto& ros_transform_stamped = transforms_data<TFMessage>(msg)[i];
        if (!is_odometry_frame(ros_transform_stamped)) {
            continue;
        }
        const uint64_t remote_time_us {AP_ROS_TypeConversions::time_u64_micros(ros_transform_stamped.header.stamp)};

        Vector3f ap_position;
        Quaternion ap_rotation;

        convert_transform(ros_transform_stamped.transform, ap_position, ap_rotation);
        // Although ROS convention states quaternions in ROS messages should be normalized, it's not guaranteed.
        // Before propogating a potentially inaccurate quaternion to the rest of AP, normalize it here.
        // TODO what if the quaternion is NaN?
        ap_rotation.normalize();

        // No error is available in TF, trust the data as-is
        const float posErr {0.0};
        const float angErr {0.0};
        // The odom to base_link transform used is locally consistent per ROS REP-105.
        // https://www.ros.org/reps/rep-0105.html#id16
        // Thus, there will not be any resets.
        const uint8_t reset_counter {0};
        // TODO imlement jitter correction similar to GCS_MAVLINK::correct_offboard_timestamp_usec_to_ms(remote_time_us, sizeof(msg));
        const uint32_t time_ms {static_cast<uint32_t>(remote_time_us * 1E-3)};
        visual_odom->handle_pose_estimate(remote_time_us, time_ms, ap_position.x, ap_position.y, ap_position.z, ap_rotation, posErr, angErr, reset_counter);

    }
}

template <typename TransformStamped>
bool AP_ROS_External_Odom::is_odometry_frame(const TransformStamped& msg)
{
    char odom_parent[] = "odom";
    char odom_child[] = "base_link";
    // Assume the frame ID's are null terminated.
    return (strcmp(string_data(msg.header.frame_id), odom_parent) == 0) &&
           (strcmp(string_data(msg.child_frame_id), odom_child) == 0);
}

template <typename Transform>
void AP_ROS_External_Odom::convert_transform(const Transform& ros_transform, Vector3f& translation, Quaternion& rotation)
{
    // convert from x-forward, y-left, z-up to NED
    // https://github.com/mavlink/mavros/issues/49#issuecomment-51614130
    translation = {
        static_cast<float>(ros_transform.translation.x),
        static_cast<float>(-ros_transform.translation.y),
        static_cast<float>(-ros_transform.translation.z)
    };

    // In AP, q1 is the quaternion's scalar component.
    // In ROS, w is the quaternion's scalar component.
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html#components-of-a-quaternion
    rotation.q1 = ros_transform.rotation.w;
    rotation.q2 = ros_transform.rotation.x;
    rotation.q3 = -ros_transform.rotation.y;
    rotation.q4 = -ros_transform.rotation.z;
}
