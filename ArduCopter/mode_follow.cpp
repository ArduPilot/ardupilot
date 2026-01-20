#include "Copter.h"

#if MODE_FOLLOW_ENABLED

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AP_AVOIDANCE_ENABLED is true because we rely on it velocity limiting functions
 */

// initialise follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    // follow the lead vehicle using sysid
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    // initialise horizontal speed, acceleration
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

    // initialize vertical speeds and acceleration
    pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());

    // initialise velocity controller
    pos_control->D_init_controller();
    pos_control->NE_init_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollow::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // Initialize follow offset if not yet set.
    // Prevents vehicle from starting directly on top of the lead vehicle.
    g2.follow.init_offsets_if_required();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    Vector3p pos_ofs_ned_m;  // vector to lead vehicle + offset
    Vector3f vel_ofs_ned_ms;  // velocity of lead vehicle + offset
    Vector3f accel_ofs_ned_mss;  // accel of lead vehicle + offset
    if (g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_m, vel_ofs_ned_ms, accel_ofs_ned_mss)) {

        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        pos_control->input_pos_vel_accel_NE_m(pos_ofs_ned_m.xy(), vel_ofs_ned_ms.xy(), accel_ofs_ned_mss.xy(), false);

        float pos_ofs_d_m = pos_ofs_ned_m.z;
        pos_control->input_pos_vel_accel_D_m(pos_ofs_d_m, vel_ofs_ned_ms.z, accel_ofs_ned_mss.z, false);

        // Determine desired yaw behavior based on configured follow mode
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                // Face the target directly
                Vector3p pos_ned_m;  // vector to lead vehicle
                Vector3f vel_ned_ms;  // velocity of lead vehicle
                Vector3f accel_ned_mss;  // accel of lead vehicle
                if (g2.follow.get_target_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, accel_ned_mss))
                if (pos_ned_m.xy().length_squared() > 1.0) {
                    yaw_rad = (pos_ned_m.xy() - pos_control->get_pos_target_NED_m().xy()).tofloat().angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                // Match the heading of the lead vehicle
                yaw_rad = radians(target_heading_deg);
                yaw_rate_rads = radians(target_heading_rate_degs);
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                // Face the direction of travel
                if (vel_ofs_ned_ms.xy().length_squared() > 1.0) {
                    yaw_rad = vel_ofs_ned_ms.xy().angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    } else {
        // Target data is invalid; hold position using zero velocity and acceleration inputs
        Vector2f vel_ne_zero;
        Vector2f accel_ne_zero;
        pos_control->input_vel_accel_NE_m(vel_ne_zero, accel_ne_zero, false);
        float vel_d_zero = 0.0;
        pos_control->input_vel_accel_D_m(vel_d_zero, 0.0, false);
        yaw_rate_rads = 0.0f;
    }

    // update the position controller
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading_rad(pos_control->get_thrust_vector(), yaw_rad, yaw_rate_rads);
}

float ModeFollow::wp_distance_m() const
{
    return g2.follow.get_distance_to_target_m();
}

float ModeFollow::wp_bearing_deg() const
{
    return g2.follow.get_bearing_to_target_deg();
}

// Returns target location with offset applied, for MAVLink reporting
bool ModeFollow::get_wp(Location &loc) const
{
    Vector3f vel;
    return g2.follow.get_target_location_and_velocity_ofs(loc, vel);
}

#endif // MODE_FOLLOW_ENABLED
