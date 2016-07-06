/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"
#define LAND_END_DEPTH -20 //Landed at 20cm depth

// Code to detect a crash main ArduCopter code
#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity

// counter to verify landings
static uint32_t land_detector_count = 0;

// run land and crash detectors
// called at MAIN_LOOP_RATE
void Sub::update_land_and_crash_detectors()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, MAIN_LOOP_SECONDS);

    update_land_detector();

    crash_check();
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
void Sub::update_land_detector()
{
//	if(barometer.num_instances() > 1 && barometer.get_altitude() > LAND_END_DEPTH && ap.throttle_zero) {
//		set_land_complete(true);
//	} else {
		set_land_complete(false);
//	}
}

void Sub::set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    land_detector_count = 0;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    } else {
        Log_Write_Event(DATA_NOT_LANDED);
    }
    ap.land_complete = b;

    // trigger disarm-on-land if configured
    bool disarm_on_land_configured = (g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) != 0;
    bool mode_disarms_on_land = mode_allows_arming(control_mode,false) && !mode_has_manual_throttle(control_mode);

    if (ap.land_complete && motors.armed() && disarm_on_land_configured && mode_disarms_on_land) {
        init_disarm_motors();
    }
}

// update_throttle_thr_mix - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
void Sub::update_throttle_thr_mix()
{
	return; // placeholder, was used by heli only
}
