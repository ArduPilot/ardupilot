/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG == HELI_FRAME
/*
 * heli_control_acro.pde - init and run calls for acro flight mode for trad heli
 */

// heli_acro_init - initialise acro controller
static bool heli_acro_init(bool ignore_checks)
{
    // clear stabilized rate errors
    attitude_control.init_targets();
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
static void heli_acro_run()
{
    int16_t target_roll, target_pitch, target_yaw;

    // if not armed or main rotor not up to full speed clear stabilized rate errors
    // unlike multicopters we do not set throttle (i.e. collective pitch) to zero so the swash servos move
    if(!motors.armed() || !motors.motor_runup_complete()) {
        attitude_control.init_targets();
    }

    // To-Do: add support for flybarred helis

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in, target_roll, target_pitch, target_yaw);

    attitude_control.ratebf_rpy(target_roll, target_pitch, target_yaw);
}

#endif  //HELI_FRAME
