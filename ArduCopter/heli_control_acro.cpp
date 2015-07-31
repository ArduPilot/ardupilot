/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * heli_control_acro.pde - init and run calls for acro flight mode for trad heli
 */

// heli_acro_init - initialise acro controller
bool Copter::heli_acro_init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    attitude_control.use_flybar_passthrough(motors.has_flybar(), motors.tail_type() == AP_MOTORS_HELI_TAILTYPE_SERVO_EXTGYRO);

    // always successfully enter acro
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
void Copter::heli_acro_run()
{
    float target_roll, target_pitch, target_yaw;
    
    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup, because
    // we may be in autorotation flight.  These should be reset only when transitioning from disarmed
    // to armed, because the pilot will have placed the helicopter down on the landing pad.  This is so
    // that the servos move in a realistic fashion while disarmed for operational checks.
    // Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero so the swash servos move
    
    if(!motors.armed()) {
        heli_flags.init_targets_on_arming=true;
        attitude_control.set_yaw_target_to_current_heading();
    }
    
    if(motors.armed() && heli_flags.init_targets_on_arming) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        if (motors.rotor_speed_above_critical()) {
            heli_flags.init_targets_on_arming=false;
        }
    }   

    // send RC inputs direct into motors library for use during manual passthrough for helicopter setup
    heli_radio_passthrough();

    if (!motors.has_flybar()){
        // convert the input to the desired body frame rate
        get_pilot_desired_angle_rates(channel_roll->control_in, channel_pitch->control_in, channel_yaw->control_in, target_roll, target_pitch, target_yaw);
        // run attitude controller
        attitude_control.rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
    }else{
        // flybar helis only need yaw rate control
        get_pilot_desired_yaw_rate(channel_yaw->control_in, target_yaw);
        // run attitude controller
        attitude_control.passthrough_bf_roll_pitch_rate_yaw(channel_roll->control_in, channel_pitch->control_in, target_yaw);
    }

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(channel_throttle->control_in, false, g.throttle_filt);
}

// get_pilot_desired_yaw_rate - transform pilot's yaw input into a desired yaw angle rate
// returns desired yaw rate in centi-degrees-per-second
void Copter::get_pilot_desired_yaw_rate(int16_t yaw_in, float &yaw_out)
{
    // calculate rate request
    float rate_bf_yaw_request = yaw_in * g.acro_yaw_p;

    // hand back rate request
    yaw_out = rate_bf_yaw_request;
}

#endif  //HELI_FRAME
