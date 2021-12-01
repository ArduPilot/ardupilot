#include "Copter.h"

#if MODE_ACRO_ENABLED == ENABLED

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for acro flight mode for trad heli
 */

// heli_acro_init - initialise acro controller
bool ModeAcro_Heli::init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    attitude_control->use_flybar_passthrough(motors->has_flybar(), motors->supports_yaw_passthrough());

    motors->set_acro_tail(true);
    
    // set stab collective false to use full collective pitch range
    copter.input_manager.set_use_stab_col(false);

    // always successfully enter acro
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
void ModeAcro_Heli::run()
{
    float target_roll, target_pitch, target_yaw;
    float pilot_throttle_scaled;

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup while flying, because
    // we may be in autorotation flight.  This is so that the servos move in a realistic fashion while disarmed
    // for operational checks. Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero
    // so the swash servos move.

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // If aircraft is landed, set target heading to current and reset the integrator
        // Otherwise motors could be at ground idle for practice autorotation
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    if (!motors->has_flybar()){
        // convert the input to the desired body frame rate
        get_pilot_desired_angle_rates(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll, target_pitch, target_yaw);
        // only mimic flybar response when trainer mode is disabled
        if ((Trainer)g.acro_trainer.get() == Trainer::OFF) {
            // while landed always leak off target attitude to current attitude
            if (copter.ap.land_complete) {
                virtual_flybar(target_roll, target_pitch, target_yaw, 3.0f, 3.0f);
            // while flying use acro balance parameters for leak rate
            } else {
                virtual_flybar(target_roll, target_pitch, target_yaw, g.acro_balance_pitch, g.acro_balance_roll);
            }
        }
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            target_yaw = channel_yaw->get_control_in_zero_dz();
        }

        // run attitude controller
        if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
        }
    }else{
        /*
          for fly-bar passthrough use control_in values with no
          deadzone. This gives true pass-through.
         */
        float roll_in = channel_roll->get_control_in_zero_dz();
        float pitch_in = channel_pitch->get_control_in_zero_dz();
        float yaw_in;
        
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            yaw_in = channel_yaw->get_control_in_zero_dz();
        } else {
            // if there is no external gyro then run the usual
            // ACRO_YAW_P gain on the input control, including
            // deadzone
            yaw_in = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        }

        // run attitude controller
        attitude_control->passthrough_bf_roll_pitch_rate_yaw(roll_in, pitch_in, yaw_in);
    }

    // get pilot's desired throttle
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}


// virtual_flybar - acts like a flybar by leaking target atttitude back to current attitude
void ModeAcro_Heli::virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak)
{
    Vector3f rate_ef_level, rate_bf_level;

    // get attitude targets
    const Vector3f att_target = attitude_control->get_att_target_euler_cd();

    // Calculate earth frame rate command for roll leak to current attitude
    rate_ef_level.x = -wrap_180_cd(att_target.x - ahrs.roll_sensor) * roll_leak;

    // Calculate earth frame rate command for pitch leak to current attitude
    rate_ef_level.y = -wrap_180_cd(att_target.y - ahrs.pitch_sensor) * pitch_leak;

    // Calculate earth frame rate command for yaw
    rate_ef_level.z = 0;

    // convert earth-frame leak rates to body-frame leak rates
    attitude_control->euler_rate_to_ang_vel(attitude_control->get_att_target_euler_cd()*radians(0.01f), rate_ef_level, rate_bf_level);

    // combine earth frame rate corrections with rate requests
    roll_out += rate_bf_level.x;
    pitch_out += rate_bf_level.y;
    yaw_out += rate_bf_level.z;

}
#endif  //HELI_FRAME
#endif  //MODE_ACRO_ENABLED == ENABLED
