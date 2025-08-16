#include "Copter.h"

#if MODE_FOLLOW_ENABLED == ENABLED

#include<AP_Camera/AP_Camera_Tracking.h>
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

    // re-use guided mode
    return ModeGuided::init(ignore_checks);
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

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        // convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        // calculate desired relative velocity vector in cm/s in NEU
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_neu_cms = dist_vec_offs_neu * kp;

        // create horizontal desired velocity vector (required for slow down calculations)
        Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);

        // create horizontal unit vector towards target (required for slow down calculations)
        Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        if (!dir_to_target_xy.is_zero()) {
            dir_to_target_xy.normalize();
        }

        // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
        const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // copy horizontal velocity limits back to 3d vector
        desired_velocity_neu_cms.xy() = desired_velocity_xy_cms;

        // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // Add the target velocity baseline.
        desired_velocity_neu_cms.xy() += vel_of_target.xy() * 100.0f;
        desired_velocity_neu_cms.z += -vel_of_target.z * 100.0f;

        // scale desired velocity to stay within horizontal speed limit
        desired_velocity_neu_cms.xy().limit_length(pos_control->get_max_speed_xy_cms());

        // limit desired velocity to be between maximum climb and descent rates
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // limit the velocity for obstacle/fence avoidance
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // calculate vehicle heading
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                if (dist_vec.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                float target_hdg = 0.0f;
                if (g2.follow.get_target_heading_deg(target_hdg)) {
                    yaw_cd = target_hdg * 100.0f;
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (desired_velocity_neu_cms.xy().length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                    use_yaw = true;
                }
                break;
            }
#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
            case AP_Follow::YAW_BEHAVE_OBJECT_FOLLOW: {
                AP_Mount *mount = AP_Mount::get_singleton();
                float yaw_max, pitch_max;
                float yaw_min, pitch_min;
                mount->get_mount_yaw_limits(yaw_min,yaw_max);
                mount->get_mount_pitch_limits(pitch_min,pitch_max);
                // Get gimbal attitude
                float roll, pitch, yaw;
                if (!mount->get_attitude_euler(mount->get_primary_instance(), roll, pitch, yaw)) {
                    break; 
                }
                float last_yaw_tobe_pointed_to;
                bool last_yaw_tobe_pointed_to_isvalid = AP_Follow::get_singleton()->get_yaw_under_gimbal_limit_to_point_to(last_yaw_tobe_pointed_to);
                
                bool cond1 = yaw > (yaw_max - g2.follow.get_object_follow_yaw_reset());
                bool cond2 = yaw < (yaw_min + g2.follow.get_object_follow_yaw_reset());
                bool cond3 = !last_yaw_tobe_pointed_to_isvalid;
                bool cond4 = pitch < (pitch_min + g2.follow.get_object_follow_pitch_reset());
                
                if (cond1 || cond2 || cond3 || cond4) {
                    // gcs().send_text(MAV_SEVERITY_INFO, "Reset: y=%.1f p=%.1f c1=%d c2=%d c3=%d c4=%d", yaw, pitch, cond1, cond2, cond3, cond4);
                    
                    if (dist_vec.xy().length_squared() > 1.0) {
                        yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                        AP_Follow::get_singleton()->set_yaw_under_gimbal_limit_to_point_to(yaw_cd);
                        use_yaw = true;
                    } else {
                        // code this
                        // Its when your object is going (slowly or by some way) such that you can't get the distance vector less than 1 and
                        // You have achieved your yaw and pitch limits
                    }
                }
                last_yaw_tobe_pointed_to_isvalid = AP_Follow::get_singleton()->get_yaw_under_gimbal_limit_to_point_to(last_yaw_tobe_pointed_to);
                if (last_yaw_tobe_pointed_to_isvalid) {
                    yaw_cd = last_yaw_tobe_pointed_to;
                    use_yaw = true;
                } else {
                    // code this
                }
                break;
            }
#endif
            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    }

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}

uint32_t ModeFollow::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeFollow::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
