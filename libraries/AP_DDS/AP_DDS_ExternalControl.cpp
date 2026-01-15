#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"
#include <AP_AHRS/AP_AHRS.h>

#include <AP_ExternalControl/AP_ExternalControl.h>

bool AP_DDS_External_Control::handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0) {
        // Narrow the altitude
        const int32_t alt_cm  = static_cast<int32_t>(cmd_pos.altitude * 100);

        Location::AltFrame alt_frame;
        if (!convert_alt_frame(cmd_pos.coordinate_frame, alt_frame)) {
            return false;
        }

        constexpr uint32_t MASK_POS_IGNORE =
            GlobalPosition::IGNORE_LATITUDE |
            GlobalPosition::IGNORE_LONGITUDE |
            GlobalPosition::IGNORE_ALTITUDE;

        if (!(cmd_pos.type_mask & MASK_POS_IGNORE)) {
            Location loc(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);
            if (!external_control->set_global_position(loc)) {
                return false; // Don't try sending other commands if this fails
            }
        }

        // TODO add velocity and accel handling

        return true;
    }

    return false;
}

bool AP_DDS_External_Control::handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_vel.header.frame_id, BASE_LINK_FRAME_ID) == 0) {
        // Convert commands from body frame (x-forward, y-left, z-up) to NED.
        Vector3f linear_velocity;
        Vector3f linear_velocity_base_link {
            float(cmd_vel.twist.linear.x),
            float(-cmd_vel.twist.linear.y),
            float(-cmd_vel.twist.linear.z) };

        if (isnan(linear_velocity_base_link.y) && isnan(linear_velocity_base_link.z)) {
            // Assume it's an airspeed command so ignore the angular data.
            // While MAV_CMD_GUIDED_CHANGE_SPEED supports commands of ground speed and airspeed,
            // ROS users likely care more about airspeed control for a low level velocity control interface like this.
            return external_control->set_airspeed(linear_velocity_base_link.x);
        }

        const float yaw_rate = -cmd_vel.twist.angular.z;

        auto &ahrs = AP::ahrs();
        linear_velocity = ahrs.body_to_earth(linear_velocity_base_link);
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    else if (strcmp(cmd_vel.header.frame_id, MAP_FRAME) == 0) {
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

#if AP_DDS_LOCAL_POSE_CTRL_ENABLED
bool AP_DDS_External_Control::handle_local_position_control(ardupilot_msgs_msg_LocalPosition& cmd_pos)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }
    // TODO: currently odom frame is considered fixed to map frame
    // TODO: DDS architecture should publish map to odom transform and odom to base_link transform

    // Handle MAP (ENU) or ODOM (ENU) frames
    if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0 || strcmp(cmd_pos.header.frame_id, ODOM_FRAME) == 0) {
        Vector3p pos_ned_m;
        // not allowing mixing of ignore flags
        const bool pos_ignore = (cmd_pos.type_mask & (LocalPosition::IGNORE_X | LocalPosition::IGNORE_Y | LocalPosition::IGNORE_Z)) == (LocalPosition::IGNORE_X | LocalPosition::IGNORE_Y | LocalPosition::IGNORE_Z);
        const bool vel_ignore = (cmd_pos.type_mask & (LocalPosition::IGNORE_VX | LocalPosition::IGNORE_VY | LocalPosition::IGNORE_VZ)) == (LocalPosition::IGNORE_VX | LocalPosition::IGNORE_VY | LocalPosition::IGNORE_VZ);
        const bool acc_ignore = (cmd_pos.type_mask & (LocalPosition::IGNORE_AFX | LocalPosition::IGNORE_AFY | LocalPosition::IGNORE_AFZ)) == (LocalPosition::IGNORE_AFX | LocalPosition::IGNORE_AFY | LocalPosition::IGNORE_AFZ);
        const bool yaw_ignore = cmd_pos.type_mask & LocalPosition::IGNORE_YAW;
        const bool yaw_rate_ignore = cmd_pos.type_mask & LocalPosition::IGNORE_YAW_RATE;

        if (!pos_ignore) {
            // Convert ENU to NED
            pos_ned_m = Vector3p{
                postype_t(cmd_pos.y),
                postype_t(cmd_pos.x),
                -postype_t(cmd_pos.z)
            };
        } else {
            pos_ned_m = Vector3p{NAN, NAN, NAN};
        }

        Vector3f velocity_ned;
        if (!vel_ignore) {
            velocity_ned = Vector3f{
                float(cmd_pos.velocity.linear.y),
                float(cmd_pos.velocity.linear.x),
                -float(cmd_pos.velocity.linear.z)
            };
        } else {
            velocity_ned = Vector3f{NAN, NAN, NAN};
        }

        Vector3f acceleration_ned;
        if (!acc_ignore) {
            acceleration_ned = Vector3f{
                float(cmd_pos.acceleration_or_force.linear.y),
                float(cmd_pos.acceleration_or_force.linear.x),
                -float(cmd_pos.acceleration_or_force.linear.z)
            };
        } else {
            acceleration_ned = Vector3f{NAN, NAN, NAN};
        }

        float yaw_ned = yaw_ignore ? NAN : (M_PI_2 - cmd_pos.yaw);
        float yaw_rate_ned = yaw_rate_ignore ? NAN : -cmd_pos.yaw_rate;

        return external_control->set_local_position(pos_ned_m.tofloat(), velocity_ned, acceleration_ned, yaw_ned, yaw_rate_ned, false);
    }
    // handle base_link frame (FLU) frame
    else if (strcmp(cmd_pos.header.frame_id, BASE_LINK_FRAME_ID) == 0) {
        Vector3p pos_ned_m;

        const bool pos_ignore = (cmd_pos.type_mask & (LocalPosition::IGNORE_X | LocalPosition::IGNORE_Y | LocalPosition::IGNORE_Z)) == (LocalPosition::IGNORE_X | LocalPosition::IGNORE_Y | LocalPosition::IGNORE_Z);
        const bool vel_ignore = (cmd_pos.type_mask & (LocalPosition::IGNORE_VX | LocalPosition::IGNORE_VY | LocalPosition::IGNORE_VZ)) == (LocalPosition::IGNORE_VX | LocalPosition::IGNORE_VY | LocalPosition::IGNORE_VZ);
        const bool acc_ignore = (cmd_pos.type_mask & (LocalPosition::IGNORE_AFX | LocalPosition::IGNORE_AFY | LocalPosition::IGNORE_AFZ)) == (LocalPosition::IGNORE_AFX | LocalPosition::IGNORE_AFY | LocalPosition::IGNORE_AFZ);
        const bool yaw_ignore = cmd_pos.type_mask & LocalPosition::IGNORE_YAW;
        const bool yaw_rate_ignore = cmd_pos.type_mask & LocalPosition::IGNORE_YAW_RATE;

        if (!pos_ignore) {
            // Convert FLU to FRD to NED
            Vector3f pos_body_frd(
                float(cmd_pos.x),
                -float(cmd_pos.y),
                -float(cmd_pos.z)
            );
            auto &ahrs = AP::ahrs();
            pos_ned_m = ahrs.body_to_earth(pos_body_frd).topostype();
            Vector3p rel_pos_ned_m;
            if (!ahrs.get_relative_position_NED_origin(rel_pos_ned_m)) {
                return false;
            }
            pos_ned_m += rel_pos_ned_m;
        } else {
            pos_ned_m = Vector3p{NAN, NAN, NAN};
        }

        Vector3f velocity_ned;
        if (!vel_ignore) {
            Vector3f velocity_frd(
                float(cmd_pos.velocity.linear.x),
                -float(cmd_pos.velocity.linear.y),
                -float(cmd_pos.velocity.linear.z)
            );
            velocity_ned = AP::ahrs().body_to_earth(velocity_frd);
        } else {
            velocity_ned = Vector3f{NAN, NAN, NAN};
        }

        Vector3f acceleration_ned;
        if (!acc_ignore) {
            Vector3f acceleration_frd(
                float(cmd_pos.acceleration_or_force.linear.x),
                -float(cmd_pos.acceleration_or_force.linear.y),
                -float(cmd_pos.acceleration_or_force.linear.z)
            );
            acceleration_ned = AP::ahrs().body_to_earth(acceleration_frd);
        } else {
            acceleration_ned = Vector3f{NAN, NAN, NAN};
        }

        // relative yaw
        float yaw_ned = yaw_ignore ? NAN : -cmd_pos.yaw;
        float yaw_rate_ned = yaw_rate_ignore ? NAN : -cmd_pos.yaw_rate;

        return external_control->set_local_position(pos_ned_m.tofloat(), velocity_ned, acceleration_ned, yaw_ned, yaw_rate_ned, true);
    }
    return false;
}
#endif // AP_DDS_LOCAL_POSE_CTRL_ENABLED

bool AP_DDS_External_Control::arm(AP_Arming::Method method, bool do_arming_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->arm(method, do_arming_checks);
}

bool AP_DDS_External_Control::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->disarm(method, do_disarm_checks);
}

bool AP_DDS_External_Control::convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out)
{

    // Specified in ROS REP-147; only some are supported.
    switch (frame_in) {
    case 5: // FRAME_GLOBAL_INT
        frame_out = Location::AltFrame::ABSOLUTE;
        break;
    case 6: // FRAME_GLOBAL_REL_ALT
        frame_out = Location::AltFrame::ABOVE_HOME;
        break;
    case 11: // FRAME_GLOBAL_TERRAIN_ALT
        frame_out = Location::AltFrame::ABOVE_TERRAIN;
        break;
    default:
        return false;
    }
    return true;
}

#endif // AP_DDS_ENABLED