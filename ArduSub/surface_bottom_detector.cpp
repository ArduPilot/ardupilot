/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// Code by Jacob Walser: jwalser90@gmail.com

#include "Sub.h"

// counter to verify contact with bottom
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;
static float start_depth = 0; // the depth when we first hit downward throttle limit

// checks if we have have hit bottom or surface and updates the ap.at_bottom and ap.at_surface flags
// called at MAIN_LOOP_RATE
void Sub::update_surface_and_bottom_detector()
{
	Vector3f velocity;
	ahrs.get_velocity_NED(velocity);

	// check that we are not moving up or down
	bool vel_stationary = velocity.z > -0.05 && velocity.z < 0.05;

	if (ap.depth_sensor_present) { // we can use the external pressure sensor for a very accurate and current measure of our z axis position
		current_depth = barometer.get_altitude();

		set_surfaced(current_depth > SURFACE_DEPTH); // If we are above surface depth, we are surfaced

		if(motors.limit.throttle_lower && vel_stationary) {
			// bottom criteria met - increment the counter and check if we've triggered
			if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				bottom_detector_count++;
			} else {
				set_bottomed(true);
			}

		} else {
			set_bottomed(false);
		}

	// with no external baro, the only thing we have to go by is a vertical velocity estimate
	} else if (vel_stationary) {

		if(motors.limit.throttle_upper) {
			// surface criteria met, increment counter and see if we've triggered
			if( surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				surface_detector_count++;
			} else {
				set_surfaced(true);
			}

		} else if(motors.limit.throttle_lower) {
			// bottom criteria met, increment counter and see if we've triggered
			if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				bottom_detector_count++;
			} else {
				set_bottomed(true);
			}

		} else { // we're not at the limits of throttle, so reset both detectors
			set_surfaced(false);
			set_bottomed(false);
		}

	} else { // we're moving up or down, so reset both detectors
		set_surfaced(false);
		set_bottomed(false);
	}
}

void Sub::set_surfaced(bool at_surface) {
	if(ap.at_surface == at_surface) { // do nothing if state unchanged
		return;
	}

	ap.at_surface = at_surface;

	if(!ap.at_surface) {
		surface_detector_count = 0;
	    Log_Write_Event(DATA_SURFACED);
		gcs_send_text(MAV_SEVERITY_CRITICAL, "Off Surface");
	} else {
		Log_Write_Event(DATA_NOT_SURFACED);
		gcs_send_text(MAV_SEVERITY_CRITICAL, "Surfaced");
	}
}

void Sub::set_bottomed(bool at_bottom) {
	if(ap.at_bottom == at_bottom) { // do nothing if state unchanged
		return;
	}

	ap.at_bottom = at_bottom;

	if(!ap.at_bottom) {
		bottom_detector_count = 0;
		Log_Write_Event(DATA_BOTTOMED);
		gcs_send_text(MAV_SEVERITY_CRITICAL, "Off Bottom");
	} else {
		Log_Write_Event(DATA_NOT_BOTTOMED);
		gcs_send_text(MAV_SEVERITY_CRITICAL, "Bottomed");
	}
}
