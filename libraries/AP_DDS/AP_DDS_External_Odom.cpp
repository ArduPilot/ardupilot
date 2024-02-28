

#include "AP_DDS_External_Odom.h"
#include "AP_DDS_Type_Conversions.h"

#if AP_DDS_VISUALODOM_ENABLED

#include <AP_VisualOdom/AP_VisualOdom.h>
#include <GCS_MAVLink/GCS.h>

void AP_DDS_External_Odom::handle_external_odom(const tf2_msgs_msg_TFMessage& msg)
{
    auto *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    for (size_t i = 0; i < msg.transforms_size; i++) {
        const auto& ros_transform_stamped = msg.transforms[i];
        if (!is_odometry_frame(ros_transform_stamped)) {
            continue;
        }
        const uint64_t remote_time_us {AP_DDS_Type_Conversions::time_u64_micros(ros_transform_stamped.header.stamp)};

        Vector3f ap_position;
        Quaternion ap_rotation;

        convert_transform(ros_transform_stamped.transform, ap_position, ap_rotation);
        // Although ROS convention states quaternions in ROS messages should be normalized, it's not guaranteed.
        // Before propagating a potentially inaccurate quaternion to the rest of AP, normalize it here.
        // TODO what if the quaternion is NaN?
        ap_rotation.normalize();

        // No error is available in TF, trust the data as-is
        const float posErr {0.0};
        const float angErr {0.0};
        // The odom to base_link transform used is locally consistent per ROS REP-105.
        // https://www.ros.org/reps/rep-0105.html#id16
        // Thus, there will not be any resets.
        const uint8_t reset_counter {0};
        // TODO implement jitter correction similar to GCS_MAVLINK::correct_offboard_timestamp_usec_to_ms(remote_time_us, sizeof(msg));
        const uint32_t time_ms {static_cast<uint32_t>(remote_time_us * 1E-3)};
        visual_odom->handle_pose_estimate(remote_time_us, time_ms, ap_position.x, ap_position.y, ap_position.z, ap_rotation, posErr, angErr, reset_counter, 0);

    }
}

bool AP_DDS_External_Odom::is_odometry_frame(const geometry_msgs_msg_TransformStamped& msg)
{
    char odom_parent[] = "odom";
    char odom_child[] = "base_link";
    // Assume the frame ID's are null terminated.
    return (strcmp(msg.header.frame_id, odom_parent) == 0) &&
           (strcmp(msg.child_frame_id, odom_child) == 0);
}

void AP_DDS_External_Odom::convert_transform(const geometry_msgs_msg_Transform& ros_transform, Vector3f& translation, Quaternion& rotation)
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

#endif // AP_DDS_VISUALODOM_ENABLED
