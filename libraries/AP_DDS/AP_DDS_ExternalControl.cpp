#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"

#include <AP_ExternalControl/AP_ExternalControl.h>

bool AP_DDS_External_Control::handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }
    if (strcmp(cmd_vel.header.frame_id, MAP_FRAME) != 0) {
        // Although REP-147 says cmd_vel should be in body frame, all the AP math is done in earth frame.
        // This is because accounting for the gravity vector.
        // Although the ROS 2 interface could support body-frame velocity control in the future,
        // it is currently not supported.
        return false;
    }

    // Convert commands from ENU to NED frame
    Vector3f linear_velocity {
        float(cmd_vel.twist.linear.y),
        float(cmd_vel.twist.linear.x),
        float(-cmd_vel.twist.linear.z) };
    const float yaw_rate = -cmd_vel.twist.angular.z;
    return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
}


#endif // AP_DDS_ENABLED