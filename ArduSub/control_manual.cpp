/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Sub.h"

// manual_init - initialise manual controller
bool Sub::manual_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::manual_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input_dz()*0.67f);
    motors.set_pitch(channel_pitch->norm_input_dz()*0.67f);
    motors.set_yaw(channel_yaw->norm_input_dz()*0.67f);
    motors.set_throttle(channel_throttle->norm_input());
    motors.set_forward(channel_forward->norm_input_dz()*0.67f);
    motors.set_lateral(channel_lateral->norm_input_dz()*0.67f);
}
