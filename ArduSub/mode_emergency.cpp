#include "Sub.h"


bool ModeEmergency::init(bool ignore_checks)
{
    has_surfaced = false;
    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cmss(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_U_controller();
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"Emergencey surface!");
    return true;
    
    has_surfaced = false;

}

void ModeEmergency::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(.5,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->init_U_controller();
        return;
    }

    // Already at surface, hold depth at surface
    if (sub.ap.at_surface || has_surfaced) {
        if (!has_surfaced) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"At Surface");
            has_surfaced = true;
        }
        motors.set_throttle(.55);  //hold at surface
        return;
    }


    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(0,0,0);

    // set target climb rate
    float cmb_rate_cms = constrain_float(fabsf(sub.wp_nav.get_default_speed_up_cms()), 1, position_control->get_max_speed_up_cms());

    // update altitude target and call position controller
    position_control->set_pos_target_U_from_climb_rate_cm(cmb_rate_cms);
    position_control->update_U_controller();

    // maximum climb to surface
    motors.set_throttle(1.1); //over 1 to clear current throttle limit on surface detection
}
