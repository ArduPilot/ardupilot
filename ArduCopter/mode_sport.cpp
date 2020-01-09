#include "Copter.h"

#if MODE_SPORT_ENABLED == ENABLED

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool ModeSport::init(bool ignore_checks)
{
    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void ModeSport::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform
    update_simple_mode();

    // get pilot's desired roll and pitch rates

    // calculate rate requests
    float target_roll_rate = channel_roll->get_control_in() * g.acro_rp_p;
    float target_pitch_rate = channel_pitch->get_control_in() * g.acro_rp_p;

    // get attitude targets
    const Vector3f att_target = attitude_control->get_att_target_euler_cd();

    // Calculate trainer mode earth frame rate command for roll
    int32_t roll_angle = wrap_180_cd(att_target.x);
    target_roll_rate -= constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(att_target.y);
    target_pitch_rate -= constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

    const float angle_max = copter.aparm.angle_max;
    if (roll_angle > angle_max){
        target_roll_rate +=  AC_AttitudeControl::sqrt_controller(angle_max - roll_angle, g.acro_rp_p * 4.5, attitude_control->get_accel_roll_max(), G_Dt);
    }else if (roll_angle < -angle_max) {
        target_roll_rate +=  AC_AttitudeControl::sqrt_controller(-angle_max - roll_angle, g.acro_rp_p * 4.5, attitude_control->get_accel_roll_max(), G_Dt);
    }

    if (pitch_angle > angle_max){
        target_pitch_rate +=  AC_AttitudeControl::sqrt_controller(angle_max - pitch_angle, g.acro_rp_p * 4.5, attitude_control->get_accel_pitch_max(), G_Dt);
    }else if (pitch_angle < -angle_max) {
        target_pitch_rate +=  AC_AttitudeControl::sqrt_controller(-angle_max - pitch_angle, g.acro_rp_p * 4.5, attitude_control->get_accel_pitch_max(), G_Dt);
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Sport State Machine Determination
    AltHoldModeState sport_state = get_alt_hold_state(target_climb_rate);

    // State Machine
    switch (sport_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_rate_roll_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();
}

#endif
