#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Semaphore.h>

#define LEAN_CENTIDEGREES_TO_CORRECT 20
#define Throttle_TO_CORRECT 0,02f

extern const AP_HAL::HAL& hal;

// stabilize_init - initialise stabilize controller
bool Copter::ModeHoldUltra::init(bool ignore_checks)
{
	// Define here, what kind of modes are nessesary to start this flightmode
	// Give back false, if cannot start this mode
	// if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    /*if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
	*/
	
	// Warning, when the RANGEFINDER is disabled in precomp
	#ifndef RANGEFINDER_ENABLED
		gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder is not enabled (pre compiler) %5.3f", (double)3.142f);
		return false;
	#endif
	
	// ... or in the stack
	if (copter.rangefinder_state.enabled)
	{(
		gcs().send_text(MAV_SEVERITY_CRITICAL, "Rangefinder is not enabled! (runtime) %5.3f", (double)3.142f);
		return false;
	}
	
	hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &Copter::ModeHoldUltra::_timer, void));
	throttle = motors->get_throttle();
    return true;
}

// hold_run - runs the main hold controller
// should be called at 100hz or more
void Copter::ModeHoldUltra::run()
{
    float target_roll = 0;
	float target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;
	float left_distance_from_wall;
	
	alt_target = 40;  // 40 cm over ground
	left_target = 20;  // 20 cm from left

	// holds a fixed height
	 if (copter.rangefinder_alt_ok()) {
        alt_above_ground = copter.rangefinder_state.alt_cm_filt.get();
	
	    if (alt_above_ground > alt_target) {
		    throttle =- Throttle_TO_CORRECT;
			if (throttle < 0) {
				throttle = 0;
			}
	    }
	    if (alt_above_ground < alt_target) {
		    throttle =+ Throttle_TO_CORRECT;
			if (throttle > 1) {
				throttle = 1;
			}
	    }
	 }
    attitude_control->set_throttle_out(throttle, true, g.throttle_filt);
	  
	// keeps a fixed distance to the left obstacle
	 if (copter.rangefinder_left_ok()) {
        left_distance_from_wall = copter.rangefinder_left_state.left_cm_filt.get();
	
	    if (left_distance_from_wall > left_target) {
		    target_roll =- LEAN_CENTIDEGREES_TO_CORRECT;
	    }
	    if (left_distance_from_wall < left_target) {
		    target_roll =+ LEAN_CENTIDEGREES_TO_CORRECT;
	    }
	 }
	
	// set attitude
	target_pitch = 100;  // 100 centidegrees = 10°
	target_yaw_rate = 0; // rate 0: hold the direction
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}

void Copter::ModeHoldUltra::_timer()
{
	// Sampled at 100Hz
    uint32_t now = AP_HAL::millis();
    if ((now - _last_sample_time) < 10) {
        return;
    }
    _last_sample_time = now;
}
