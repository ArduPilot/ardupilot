/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// Code by Jacob Walser: jwalser90@gmail.com

#include "Sub.h"

#define SURFACE_DEPTH -0.05 // 5cm depends on where the external depth sensor is mounted
#define DIVE_DEPTH_CONSTRAINT 0.10 // 10cm if we cannot achieve this target, we are trying to dig into the bottom
#define BOTTOM_DETECTOR_TRIGGER_SEC 1.0
#define SURFACE_DETECTOR_TRIGGER_SEC 1.0

// counter to verify contact with bottom
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;
static float start_depth = 0; // the depth when we first hit downward throttle limit

// checks if we have have hit bottom or surface and updates the ap.at_bottom and ap.at_surface flags
// called at MAIN_LOOP_RATE
void Sub::update_surface_and_bottom_detector()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    depth_accel_ef_filter.apply(accel_ef, MAIN_LOOP_SECONDS);

	// check that the airframe is not accelerating (not falling or breaking after fast forward flight)
	bool accel_stationary = (depth_accel_ef_filter.get().length() <= 1.0f);

	if (ap.depth_sensor_present) { // we can use the external pressure sensor for a very accurate and current measure of our z axis position
		current_depth = barometer.get_altitude();


		set_surfaced(current_depth > SURFACE_DEPTH); // If we are above surface depth, we are surfaced

		// ToDo maybe we can lighten the throttle check to a less extreme value, will depend on buoyancy and effective motor force
		// it won't make a difference beyond informational purposes for non-auto control modes ie stabilize
		if (motors.limit.throttle_lower) { // We can't predict where the bottom is, unless we have a sounder
			if (bottom_detector_count == 0) {
				start_depth = current_depth;
			}

			if (current_depth > start_depth - DIVE_DEPTH_CONSTRAINT) { // If we can't descend a short distance at full throttle, we are at the bottom
				if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
					bottom_detector_count++;
				} else {
					set_bottomed(true);
				}
			} else {
				set_bottomed(false); // We are still moving down
			}
		} else {
			set_bottomed(false); // If we are not trying to descend at lower throttle limit, we are not at the bottom
		}

	}
//	else if (accel_stationary) {
//		if(motors.limit.throttle_upper) {
//			if( surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
//				surface_detector_count++;
//			} else {
//				set_surfaced(true);
//			}
//		} else if(motors.limit.throttle_lower) {
//			// landed criteria met - increment the counter and check if we've triggered
//			if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
//				bottom_detector_count++;
//			} else {
//				set_bottomed(true);
//			}
//		} else {
//			set_surfaced(false);
//			set_bottomed(false);
//		}
//	} else {
//		// we've sensed movement up or down so reset land_detector
//		set_surfaced(false);
//		set_bottomed(false);
//	}


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
