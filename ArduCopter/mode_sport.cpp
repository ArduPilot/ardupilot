#include "Copter.h"

#if MODE_SPORT_ENABLED

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool ModeSport::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void ModeSport::run()
{
    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

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
    target_roll_rads -= constrain_float(roll_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    float pitch_angle_rad = wrap_PI(att_target_euler_rad.y);
    target_pitch_rads -= constrain_float(pitch_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_pitch;

    const float angle_max_rad = attitude_control->lean_angle_max_rad();

    if (roll_angle_rad > angle_max_rad){
        target_roll_rads +=  sqrt_controller(angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
    }else if (roll_angle_rad < -angle_max_rad) {
        target_roll_rads +=  sqrt_controller(-angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
    }

    if (pitch_angle_rad > angle_max_rad){
        target_pitch_rads +=  sqrt_controller(angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
    }else if (pitch_angle_rad < -angle_max_rad) {
        target_pitch_rads +=  sqrt_controller(-angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
    }

    // get pilot desired climb rate
    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    // Sport State Machine Determination
    AltHoldModeState sport_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    // State Machine
    switch (sport_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01, 0.0, 10.0));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_rate_roll_pitch_yaw_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);

    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();
}

#endif
