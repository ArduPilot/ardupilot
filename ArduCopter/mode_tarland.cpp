#include "Copter.h"

#if MODE_TARLAND_ENABLED

#define MIN_DESCENT_ALT_OFF 1
#define MIN_FOLLOW_DIST 1 

/**
 * mode_tarland.cpp - follow another mavlink enabled vehicle and land on it
 * Use AP_FOLLOW library to achieve landing
 */
bool ModeTarLand::init(bool ignore_checks)
{

    if(!ignore_checks){
        if(!AP::ahrs().home_is_set()){
            return false;
        }
    }

    if(!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

    // re use guided mode
    hal.console->printf("start tarland");
    copter.precland_statemachine.init();
    return ModeGuided::init(ignore_checks);
}

void ModeTarLand::run()
{   
    if(!motors->armed() || landed) {
        return;
    }

    const bool has_target_info = g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, target_vel);


    if(has_target_info){

        if(dist_vec.xy().length() > MIN_FOLLOW_DIST){
            // Follow the target
            g2.follow.set_offset({0,0,-15}, offset_type);
            follow();

        }else if(dist_vec.xy().length() <= MIN_FOLLOW_DIST){


                if(dist_vec.z > MIN_DESCENT_ALT_OFF){
                    // Initiate descent
                    g2.follow.set_offset({0,0,-MIN_DESCENT_ALT_OFF}, offset_type);
                    follow();

                }else if(dist_vec.z <= MIN_DESCENT_ALT_OFF){

                    perform_landing();
                }
        }


    }
}

void ModeTarLand::follow()
{

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    const bool has_target_info = g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, target_vel);

    if(has_target_info){

        // convert pos_err_off to cm in NEU frame
        const Vector3f pos_err_off_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);
        get_desired_vel_neu_cms(desired_velocity_neu_cms, pos_err_off_neu, dist_vec, dist_vec_offs);

        limit_desired_velocity_xy(desired_velocity_neu_cms, pos_err_off_neu);
        add_feedforward_velocity_xy(desired_velocity_neu_cms);

        limit_desired_velocity_z(desired_velocity_neu_cms, pos_err_off_neu);
        add_feedforward_velocity_z(desired_velocity_neu_cms);
        
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

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    }

        // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 1000) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
        Location current_loc;
        AP::ahrs().get_location(current_loc);
        hal.console->printf("Mode Tarland");
        hal.console->printf("Distance to target: %f", dist_vec.xy().length());
        hal.console->printf("Dist x: %f\n", dist_vec.x);
         hal.console->printf("Dist y: %f\n", dist_vec.y);
         hal.console->printf("Dist z: %f\n", dist_vec.z);
         hal.console->printf("Drone z pos: %f \n", current_loc.alt*0.01f);
    }

    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}

void ModeTarLand::perform_landing()
{
    copter.set_land_complete(true);
    copter.pos_control->set_vel_desired_cms({0,0,0});

    copter.pos_control->update_xy_controller();
    copter.pos_control->update_z_controller();

    if (copter.motors->armed()) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        gcs().send_text(MAV_SEVERITY_INFO, "Ship landing complete. Motors disarmed.");
        landed = true;
    }
}

/**
 * Calculates the desired velocity from position error using P controller
 */
void ModeTarLand::get_desired_vel_neu_cms(Vector3f &desired_vel_neu_cms, const Vector3f pos_err_off_neu, Vector3f pos_err, Vector3f pos_err_off)
{

    // calculate the desired relative velocity vector in cm/s in NEU frame
    const float kp = g2.follow.get_pos_p().kP();
    desired_vel_neu_cms = pos_err_off_neu*kp;

    limit_desired_velocity_xy(desired_vel_neu_cms, pos_err_off_neu);
    // add_feedforward_velocity_xy(desired_vel_neu_cms, target_vel);

}

void ModeTarLand::limit_desired_velocity_xy(Vector3f &desired_vel_neu_cms, Vector3f pos_err_off_neu)
{
    // create horizontal unit vector towards target (required for slow down calculations)
    Vector2f dir_to_target_xy(desired_vel_neu_cms.x, desired_vel_neu_cms.y);
    if (!dir_to_target_xy.is_zero()) {
        dir_to_target_xy.normalize();
    }

    // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
    const float dist_to_target_xy = Vector2f(pos_err_off_neu.x, pos_err_off_neu.y).length();
    // apply sqrt controller to limit velocity
    Vector2f desired_vel_xy_cms(desired_vel_neu_cms.x, desired_vel_neu_cms.y);
    copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_vel_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
    desired_vel_neu_cms.xy() = desired_vel_xy_cms;
}

void ModeTarLand::limit_desired_velocity_z(Vector3f &desired_vel_neu_cms, Vector3f pos_err_off_neu)
{
    // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
    const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(pos_err_off_neu.z), copter.G_Dt);
    desired_vel_neu_cms.z = constrain_float(desired_vel_neu_cms.z, -des_vel_z_max, des_vel_z_max);
}

void ModeTarLand::add_feedforward_velocity_xy(Vector3f &desired_vel_neu_cms)
{
    desired_vel_neu_cms.xy() += target_vel.xy()*100.0f;
    desired_vel_neu_cms.xy().limit_length(pos_control->get_max_speed_xy_cms());
}

void ModeTarLand::add_feedforward_velocity_z(Vector3f &desired_vel_neu_cms)
{
    desired_vel_neu_cms.z += -target_vel.z * 100.0f;
    // limit desired velocity to be between maximum climb and descent rates
    desired_vel_neu_cms.z = constrain_float(desired_vel_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

}
#endif