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
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // initialise velocity controller
    pos_control->init_U_controller();
    pos_control->init_NE_controller();

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

    float yaw_cd = attitude_control->get_att_target_euler_cd().z;
    float yaw_rate_cds = 0.0f;

    Vector3p pos_ofs_ned_m;  // vector to lead vehicle + offset
    Vector3f vel_ofs_ned_ms;  // velocity of lead vehicle + offset
    Vector3f accel_ofs_ned_mss;  // accel of lead vehicle + offset
    if (g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_m, vel_ofs_ned_ms, accel_ofs_ned_mss)) {
        Vector2p pos_ofs_ne_cm = pos_ofs_ned_m.xy() * 100.0;
        Vector2f vel_ofs_ne_cms = vel_ofs_ned_ms.xy() * 100.0;
        Vector2f accel_ofs_ne_cmss = accel_ofs_ned_mss.xy() * 100.0;

        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        pos_control->input_pos_vel_accel_NE_cm(pos_ofs_ne_cm, vel_ofs_ne_cms, accel_ofs_ne_cmss, false);

        float pos_ofs_u_cm = -pos_ofs_ned_m.z * 100.0;
        float vel_ofs_u_cms = -vel_ofs_ned_ms.z * 100.0;
        float accel_ofs_u_cmss = -accel_ofs_ned_mss.z * 100.0;
        pos_control->input_pos_vel_accel_U_cm(pos_ofs_u_cm, vel_ofs_u_cms, accel_ofs_u_cmss, false);

        // Determine desired yaw behavior based on configured follow mode
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                // Face the target directly
                Vector3p pos_ned_m;  // vector to lead vehicle
                Vector3f vel_ned_ms;  // velocity of lead vehicle
                Vector3f accel_ned_mss;  // accel of lead vehicle
                if (g2.follow.get_target_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, accel_ned_mss))
                if (pos_ned_m.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(Vector2f{}, (pos_ned_m.xy() - pos_control->get_pos_target_NEU_cm().xy()).tofloat());
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                // Match the heading of the lead vehicle
                yaw_cd = target_heading_deg * 100.0;
                yaw_rate_cds = target_heading_rate_degs * 100.0;
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                // Face the direction of travel
                if (vel_ofs_ne_cms.length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, vel_ofs_ne_cms);
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
        Vector2f vel_zero;
        Vector2f accel_zero;
        pos_control->input_vel_accel_NE_cm(vel_zero, accel_zero, false);
        float velz = 0.0;
        pos_control->input_vel_accel_U_cm(velz, 0.0, false);
        yaw_rate_cds = 0.0f;
    }

    // update the position controller
    pos_control->update_NE_controller();
    pos_control->update_U_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading_cd(pos_control->get_thrust_vector(), yaw_cd, yaw_rate_cds);
}

float ModeFollow::wp_distance_m() const
{
    return g2.follow.get_distance_to_target_m();
}

int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target_deg() * 100;
}

// Returns target location with offset applied, for MAVLink reporting
bool ModeFollow::get_wp(Location &loc) const
{
    Vector3f vel;
    return g2.follow.get_target_location_and_velocity_ofs(loc, vel);
}

#endif // MODE_FOLLOW_ENABLED
