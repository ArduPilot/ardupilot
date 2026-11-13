

#include "AP_DDS_External_Odom.h"
#include "AP_DDS_Type_Conversions.h"

#if AP_DDS_VISUALODOM_ENABLED

#include <AP_HAL/AP_HAL.h>
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

#if AP_DDS_ODOM_SUB_ENABLED
void AP_DDS_External_Odom::handle_external_odom(const nav_msgs_msg_Odometry& msg)
{
    auto *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    // Use ArduPilot's internal clock for time_ms passed to the EKF.
    // The EKF compares this against imuSampleTime_ms (boot-relative),
    // so we must use the same epoch.  The ROS header stamp (Unix epoch
    // or zero from ros2 topic pub) is kept only for logging.
    const uint64_t remote_time_us {AP_DDS_Type_Conversions::time_u64_micros(msg.header.stamp)};
    const uint32_t time_ms {AP_HAL::millis()};

    // Extract position from pose (ENU to NED)
    const Vector3f ap_position {
        static_cast<float>(msg.pose.pose.position.x),
        static_cast<float>(-msg.pose.pose.position.y),
        static_cast<float>(-msg.pose.pose.position.z)
    };

    // Extract orientation from pose (ENU to NED)
    Quaternion ap_rotation;
    ap_rotation.q1 = msg.pose.pose.orientation.w;
    ap_rotation.q2 = msg.pose.pose.orientation.x;
    ap_rotation.q3 = -msg.pose.pose.orientation.y;
    ap_rotation.q4 = -msg.pose.pose.orientation.z;
    ap_rotation.normalize();

    // Extract velocity from twist (body-frame FLU → body-frame FRD)
    // twist.twist.linear is in the child_frame_id (base_link, FLU convention)
    Vector3f ap_velocity {
        static_cast<float>(msg.twist.twist.linear.x),
        static_cast<float>(-msg.twist.twist.linear.y),
        static_cast<float>(-msg.twist.twist.linear.z)
    };

    // Rotate velocity from body frame (FRD) to world frame (NED).
    // handle_vision_speed_estimate → EKF3::writeExtNavVelData expects NED.
    // This matches the MAVLink ODOMETRY handler which does: vel = q * vel;
    ap_velocity = ap_rotation * ap_velocity;

    // Use pose covariance diagonal as error estimates if available
    // covariance is row-major 6x6: [0]=xx, [7]=yy, [14]=zz
    float posErr {0.0};
    if (msg.pose.covariance[0] > 0) {
        posErr = sqrtf(static_cast<float>(msg.pose.covariance[0]));
    }
    const float angErr {0.0};

    const uint8_t reset_counter {0};

    // Send pose estimate to EKF (position is in local FRD, rotated internally by AP_VisualOdom)
    visual_odom->handle_pose_estimate(remote_time_us, time_ms, ap_position.x, ap_position.y, ap_position.z, ap_rotation, posErr, angErr, reset_counter, 0);

    // Send velocity estimate to EKF (velocity is now in NED world frame)
    visual_odom->handle_vision_speed_estimate(remote_time_us, time_ms, ap_velocity, reset_counter, 0);
}
#endif // AP_DDS_ODOM_SUB_ENABLED

#endif // AP_DDS_VISUALODOM_ENABLED
