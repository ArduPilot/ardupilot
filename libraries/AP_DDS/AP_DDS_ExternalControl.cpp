#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"
#include <AP_AHRS/AP_AHRS.h>

#include <AP_ExternalControl/AP_ExternalControl.h>

// These are the Goal Interface constants. Because microxrceddsgen does not expose
// them in the generated code, they are manually maintained.

//! @todo(srmainwaring) make an enum class and use templates to enable as bitfield
// see: https://accu.org/journals/overload/24/132/williams_2228/
typedef enum AP_DDS_PositionTargetTypeMask {
    TYPE_MASK_IGNORE_LATITUDE = 1,
    TYPE_MASK_IGNORE_LONGITUDE = 2,
    TYPE_MASK_IGNORE_ALTITUDE = 4,
    TYPE_MASK_IGNORE_VX = 8,
    TYPE_MASK_IGNORE_VY = 16,
    TYPE_MASK_IGNORE_VZ = 32,
    TYPE_MASK_IGNORE_AFX = 64,
    TYPE_MASK_IGNORE_AFY = 128,
    TYPE_MASK_IGNORE_AFZ = 256,
    TYPE_MASK_FORCE_SET = 512,
    TYPE_MASK_YAW_IGNORE = 1024,
    TYPE_MASK_YAW_RATE_IGNORE = 2048,
} AP_DDS_PositionTargetTypeMask;

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

        const uint16_t TYPE_MASK_LAST_BYTE = 0xF000;

        // position setpoint mask
        const uint16_t position_mask =
            (TYPE_MASK_IGNORE_VX | TYPE_MASK_IGNORE_VY | TYPE_MASK_IGNORE_VZ |
             TYPE_MASK_IGNORE_AFX | TYPE_MASK_IGNORE_AFY | TYPE_MASK_IGNORE_AFZ |
             TYPE_MASK_YAW_IGNORE | TYPE_MASK_YAW_RATE_IGNORE |
             TYPE_MASK_LAST_BYTE) ^ 0xFFFF;

        // path setpoint mask
        const uint16_t path_mask =
            (TYPE_MASK_YAW_IGNORE | TYPE_MASK_YAW_RATE_IGNORE |
             TYPE_MASK_LAST_BYTE) ^ 0xFFFF;

        // position control
        if (((cmd_pos.type_mask | TYPE_MASK_LAST_BYTE) ^ 0xFFFF) == position_mask) {
            Location loc(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);
            if (!external_control->set_global_position(loc)) {
                return false; // Don't try sending other commands if this fails
            }
        }

        // path guidance
        if (((cmd_pos.type_mask | TYPE_MASK_LAST_BYTE) ^ 0xFFFF) == path_mask) {
            Location position_on_path(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);

            // convert velocty and acceleration setpoints from ENU to NED
            Vector2f vel(
                float(cmd_pos.velocity.linear.y),
                float(cmd_pos.velocity.linear.x));
            Vector2f accel(
                float(cmd_pos.acceleration_or_force.linear.y),
                float(cmd_pos.acceleration_or_force.linear.x));
            Vector2f unit_vel;

            float path_curvature{0.0};
            bool dir_is_ccw{false};

            if (!vel.is_zero()) {
                unit_vel = vel.normalized();

                if (!accel.is_zero()) {
                    // curvature is determined from the acceleration normal
                    // to the planar velocity and the equation for uniform
                    // circular motion: a = v^2 / r.
                    float accel_proj = accel.dot(unit_vel);
                    Vector2f accel_lat = accel - unit_vel * accel_proj;
                    Vector2f accel_lat_unit = accel_lat.normalized();

                    path_curvature = accel_lat.length() / vel.length_squared();

                    // % is cross product, direction: cw:= 1, ccw:= -1
                    float dir = accel_lat_unit % unit_vel;
                    dir_is_ccw = dir < 0.0;
                }
            }

            if (external_control->set_path_position_tangent_and_curvature(
                    position_on_path, unit_vel, path_curvature, dir_is_ccw)) {
                return false;
            }
        }

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
            float(cmd_vel.twist.linear.y),
            float(-cmd_vel.twist.linear.z) };
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