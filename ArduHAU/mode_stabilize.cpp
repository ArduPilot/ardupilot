#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ModeStabilize::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeStabilize::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;
	float target_roll_internal = 0;
	float target_pitch_internal = 0;

    // if not armed set throttle to zero and exit immediately //ap.throttle_zero || only need when throttle act as flight
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    //update_simple_mode();

    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // convert pilot input to lean angles
	// remove for HAU letting for stabilizer control only
    //get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

	switch(get_control_mode()){
		case 0:
			target_roll_internal = 0;
			target_pitch_internal = 0;
			//pilot input
			motors->set_forward(channel_roll->norm_input());
			motors->set_updown(channel_pitch->norm_input()); 
			break;
		case 1:
		default:
			get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);
			target_roll_internal = target_roll;
			target_pitch_internal = target_pitch;
			break;
			
	}
    // call attitude controller
	// original function call for attitude control
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
	// for HAU let it be 0 for pitch and roll
	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_internal, target_pitch_internal, target_yaw_rate);

	
    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

int8_t Copter::ModeStabilize::get_control_mode(){
    int8_t switch_position;
    uint16_t mode_in = RC_Channels::rc_channel(g.control_mode_chan-1)->get_radio_in();
    if      (mode_in < 1100+800/3*1) switch_position = 0;
    else if (mode_in < 1100+800/3*2) switch_position = 1;
    else switch_position = 2;
	
	return switch_position;
}