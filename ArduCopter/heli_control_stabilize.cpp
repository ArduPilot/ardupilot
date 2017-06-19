#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for stabilize flight mode for trad heli
 */

// stabilize_init - initialise stabilize controller
bool Copter::heli_stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control->set_alt_target(0);

    // set stab collective true to use stabilize scaled collective pitch range
    input_manager.set_use_stab_col(true);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::heli_stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup, because
    // we may be in autorotation flight.  These should be reset only when transitioning from disarmed
    // to armed, because the pilot will have placed the helicopter down on the landing pad.  This is so
    // that the servos move in a realistic fashion while disarmed for operational checks.
    // Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero so the swash servos move
    
    if(!motors->armed()) {
        heli_flags.init_targets_on_arming=true;
        attitude_control->set_yaw_target_to_current_heading();
    }
    
    if(motors->armed() && heli_flags.init_targets_on_arming) {
        attitude_control->set_yaw_target_to_current_heading();
        if (motors->rotor_speed_above_critical()) {
            heli_flags.init_targets_on_arming=false;
        }
    }

    // clear landing flag above zero throttle
    if (motors->armed() && motors->get_interlock() && motors->rotor_runup_complete() && !ap.throttle_zero) {
        set_land_complete(false);
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle - note that TradHeli does not used angle-boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}

#endif  //HELI_FRAME
