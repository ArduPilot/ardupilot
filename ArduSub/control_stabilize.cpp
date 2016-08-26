/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

namespace {
	uint32_t last_pilot_heading;
	uint32_t last_pilot_yaw_input_ms = 0;
}

// stabilize_init - initialise stabilize controller
bool Sub::stabilize_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->control_in) > get_non_takeoff_throttle())) { 
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);
    last_pilot_heading = ahrs.yaw_sensor;


    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::stabilize_run()
{
	uint32_t tnow = millis();
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);

    // call attitude controller
	// update attitude controller targets

	if (target_yaw_rate != 0) { // call attitude controller with rate yaw determined by pilot input
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
		last_pilot_heading = ahrs.yaw_sensor;
		last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

	} else { // hold current heading

		// this check is required to prevent bounce back after very fast yaw maneuvers
		// the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
		if(tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
			target_yaw_rate = 0;

			// call attitude controller with target yaw rate = 0 to decelerate on yaw axis
			attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
			last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

		} else { // call attitude controller holding absolute absolute bearing
			attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true, get_smoothing_gain());
		}
	}

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
