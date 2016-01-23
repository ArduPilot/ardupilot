/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"
#define LAND_END_DEPTH -20 //Landed at 20cm depth

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
	if(barometer.get_altitude() > SURFACE_END_DEPTH && ap.throttle_zero) {
		set_land_complete(true);
	} else {
		set_land_complete(false);
	}
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
}
