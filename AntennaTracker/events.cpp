// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

// failsafe_gcs_check - check for ground station failsafe
void Tracker::vehicle_mavlink_check()
{
	uint32_t last_vehicle_update_ms;

	// return immediately if gcs has never been connected
	if (vehicle.last_heartbeat_ms == 0) 
	{
		//sets flag to start scanning for vehicle
		set_vehicle_mavlink_lost(true);
		return;
	}

	// calc time since last vehicle update
	// note: this only looks at the heartbeat from the device id set by g.sysid_target
	last_vehicle_update_ms = AP_HAL::millis() - vehicle.last_heartbeat_ms;

	// check if connection to vehicle is recovered.
	if (last_vehicle_update_ms < TRACKING_TIMEOUT_MS) {
		// check for recovery from vehicle lost
		if (vehicle.mavlink_lost) {
			set_vehicle_mavlink_lost(false);
		}
		return;
	}

	// do nothing if vehicle lost already triggered
	if (vehicle.mavlink_lost) {
		return;
	}

	// Vehicle Lost event has occured
	set_vehicle_mavlink_lost(true);

	// This is how to handle a lost vehicle
	switch (control_mode) {
	case AUTO:
		set_mode(SCAN);
		break;
	case MANUAL:
		break;
	case SCAN:
		break;
	case SERVO_TEST:
		break;
	case STOP:
		break;
	case INITIALISING:
		break;
	}
}

void Tracker::set_vehicle_mavlink_lost(bool b)
{
	vehicle.mavlink_lost = b;
}