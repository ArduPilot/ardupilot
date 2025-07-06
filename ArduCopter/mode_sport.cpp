#include "Copter.h"

#if MODE_SPORT_ENABLED

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool ModeSport::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void ModeSport::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform
    update_simple_mode();

    // get pilot's desired roll and pitch rates
    float target_roll_rads = channel_roll->norm_input_dz() * radians(g2.command_model_acro_rp.get_rate());
    float target_pitch_rads = channel_pitch->norm_input_dz() * radians(g2.command_model_acro_rp.get_rate());

    // get pilot's desired yaw rate
    float target_yaw_rads = get_pilot_desired_yaw_rate_rads();

    // get attitude targets
    const Vector3f att_target_euler_rad = attitude_control->get_att_target_euler_rad();

    // Calculate trainer mode earth frame rate command for roll
    float roll_angle_rad = wrap_PI(att_target_euler_rad.x);
    target_roll_rads -= constrain_float(roll_angle_rad, -cd_to_rad(ACRO_LEVEL_MAX_ANGLE), cd_to_rad(ACRO_LEVEL_MAX_ANGLE)) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    float pitch_angle_rad = wrap_PI(att_target_euler_rad.y);
    target_pitch_rads -= constrain_float(pitch_angle_rad, -cd_to_rad(ACRO_LEVEL_MAX_ANGLE), cd_to_rad(ACRO_LEVEL_MAX_ANGLE)) * g.acro_balance_pitch;

    const float angle_max_rad = attitude_control->lean_angle_max_rad();

    if (roll_angle_rad > angle_max_rad){
        target_roll_rads +=  sqrt_controller(angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / cd_to_rad(ACRO_LEVEL_MAX_OVERSHOOT), attitude_control->get_accel_roll_max_radss(), G_Dt);
    }else if (roll_angle_rad < -angle_max_rad) {
        target_roll_rads +=  sqrt_controller(-angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / cd_to_rad(ACRO_LEVEL_MAX_OVERSHOOT), attitude_control->get_accel_roll_max_radss(), G_Dt);
    }

    if (pitch_angle_rad > angle_max_rad){
        target_pitch_rads +=  sqrt_controller(angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / cd_to_rad(ACRO_LEVEL_MAX_OVERSHOOT), attitude_control->get_accel_pitch_max_radss(), G_Dt);
    }else if (pitch_angle_rad < -angle_max_rad) {
        target_pitch_rads +=  sqrt_controller(-angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / cd_to_rad(ACRO_LEVEL_MAX_OVERSHOOT), attitude_control->get_accel_pitch_max_radss(), G_Dt);
    }

    // get pilot desired climb rate
    float target_climb_rate_cms = get_pilot_desired_climb_rate();
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Sport State Machine Determination
    AltHoldModeState sport_state = get_alt_hold_state(target_climb_rate_cms);

    // State Machine
    switch (sport_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate_cms);
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_rate_roll_pitch_yaw_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();
}

#endif
