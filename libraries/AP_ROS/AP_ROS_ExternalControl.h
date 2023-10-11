#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_ExternalControl/AP_ExternalControl.h>

#include "AP_ROS_Frames.h"
#include "AP_ROS_TypeConversions.h"

class AP_ROS_External_Control
{
public:
    template <typename TTwistStamped>
    static bool handle_velocity_control(const TTwistStamped& cmd_vel);

};

template <typename TTwistStamped>
bool AP_ROS_External_Control::handle_velocity_control(const TTwistStamped& cmd_vel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(string_data(cmd_vel.header.frame_id), BASE_LINK_FRAME_ID) == 0) {
        // Convert commands from body frame (x-forward, y-left, z-up) to NED.
        Vector3f linear_velocity;
        Vector3f linear_velocity_base_link {
            float(cmd_vel.twist.linear.x),
            float(cmd_vel.twist.linear.y),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;

        auto &ahrs = AP::ahrs();
        linear_velocity = ahrs.body_to_earth(linear_velocity_base_link);
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    else if (strcmp(string_data(cmd_vel.header.frame_id), MAP_FRAME) == 0) {
        // Convert commands from ENU to NED frame
        Vector3f linear_velocity {
            float(cmd_vel.twist.linear.y),
            float(cmd_vel.twist.linear.x),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }
    return false;
}
