#include "Tracker.h"

/**
  update_vehicle_position_estimate - updates estimate of vehicle positions
  should be called at 50hz
 */
void Tracker::update_vehicle_pos_estimate()
{
    // calculate time since last actual position update
    float dt = (AP_HAL::micros() - vehicle.last_update_us) * 1.0e-6f;

    // if less than 5 seconds since last position update estimate the position
    if (dt < TRACKING_TIMEOUT_SEC) {
        // project the vehicle position to take account of lost radio packets
        vehicle.location_estimate = vehicle.location;
        float north_offset = vehicle.vel.x * dt;
        float east_offset = vehicle.vel.y * dt;
        location_offset(vehicle.location_estimate, north_offset, east_offset);
    	vehicle.location_estimate.alt += vehicle.vel.z * 100.0f * dt;
        // set valid_location flag
        vehicle.location_valid = true;
    } else {
        // vehicle has been lost, set lost flag
        vehicle.location_valid = false;
    }
}

/**
  update_tracker_position - updates antenna tracker position from GPS location
  should be called at 50hz
 */
void Tracker::update_tracker_position()
{
    // update our position if we have at least a 2D fix
    // REVISIT: what if we lose lock during a mission and the antenna is moving?
    if (!ahrs.get_position(current_loc) && (gps.status() >= AP_GPS::GPS_OK_FIX_2D)) {
        current_loc = gps.location();
    }
}

/**
  update_bearing_and_distance - updates bearing and distance to the vehicle
  should be called at 50hz
 */
void Tracker::update_bearing_and_distance()
{
    // exit immediately if we do not have a valid vehicle position
    if (!vehicle.location_valid) {
        return;
    }

    // calculate bearing to vehicle
    // To-Do: remove need for check of control_mode
    if (control_mode != SCAN && !nav_status.manual_control_yaw) {
        nav_status.bearing  = get_bearing_cd(current_loc, vehicle.location_estimate) * 0.01f;
    }

    // calculate distance to vehicle
    nav_status.distance = get_distance(current_loc, vehicle.location_estimate);

    // calculate altitude difference to vehicle using gps
    if (g.alt_source == ALT_SOURCE_GPS){
        nav_status.alt_difference_gps = (vehicle.location_estimate.alt - current_loc.alt) / 100.0f;
    } else {
        // g.alt_source == ALT_SOURCE_GPS_VEH_ONLY
        nav_status.alt_difference_gps = vehicle.relative_alt / 100.0f;
    }

    // calculate pitch to vehicle
    // To-Do: remove need for check of control_mode
    if (control_mode != SCAN && !nav_status.manual_control_pitch) {
    	if (g.alt_source == ALT_SOURCE_BARO) {
    	    nav_status.pitch = degrees(atan2f(nav_status.alt_difference_baro, nav_status.distance));
    	} else {
            nav_status.pitch = degrees(atan2f(nav_status.alt_difference_gps, nav_status.distance));
    	}
    }
}

/**
  main antenna tracking code, called at 50Hz
 */
void Tracker::update_tracking(void)
{
    // update vehicle position estimate
    update_vehicle_pos_estimate();

    // update antenna tracker position from GPS
    update_tracker_position();

    // update bearing and distance to vehicle
    update_bearing_and_distance();

    // do not perform any servo updates until startup delay has passed
    if (g.startup_delay > 0 &&
        AP_HAL::millis() - start_time_ms < g.startup_delay*1000) {
        return;
    }

    // do not perform updates if safety switch is disarmed (i.e. servos can't be moved)
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        return;
    }

    switch (control_mode) {
    case AUTO:
        update_auto();
        break;

    case MANUAL:
        update_manual();
        break;

    case SCAN:
        update_scan();
        break;

    case SERVO_TEST:
    case STOP:
    case INITIALISING:
        break;
    }
}

/**
   handle an updated position from the aircraft
 */
void Tracker::tracking_update_position(const mavlink_global_position_int_t &msg)
{
    vehicle.location.lat = msg.lat;
    vehicle.location.lng = msg.lon;
    vehicle.location.alt = msg.alt/10;
    vehicle.relative_alt = msg.relative_alt/10;
    vehicle.vel = Vector3f(msg.vx/100.0f, msg.vy/100.0f, msg.vz/100.0f);
    vehicle.last_update_us = AP_HAL::micros();
    vehicle.last_update_ms = AP_HAL::millis();
    // log vehicle as GPS2
    if (should_log(MASK_LOG_GPS)) {
        Log_Write_Vehicle_Pos(vehicle.location.lat, vehicle.location.lng, vehicle.location.alt, vehicle.vel);
    }
}


/**
   handle an updated pressure reading from the aircraft
 */
void Tracker::tracking_update_pressure(const mavlink_scaled_pressure_t &msg)
{
    float local_pressure = barometer.get_pressure();
    float aircraft_pressure = msg.press_abs*100.0f;

    // calculate altitude difference based on difference in barometric pressure
    float alt_diff = barometer.get_altitude_difference(local_pressure, aircraft_pressure);
    if (!isnan(alt_diff) && !isinf(alt_diff)) {
        nav_status.alt_difference_baro = alt_diff + nav_status.altitude_offset;

		if (nav_status.need_altitude_calibration) {
			// we have done a baro calibration - zero the altitude
			// difference to the aircraft
			nav_status.altitude_offset = -alt_diff;
			nav_status.alt_difference_baro = 0;
			nav_status.need_altitude_calibration = false;
		}
    }

    // log vehicle baro data
    Log_Write_Vehicle_Baro(aircraft_pressure, alt_diff);
}

/**
   handle a manual control message by using the data to command yaw and pitch
 */
void Tracker::tracking_manual_control(const mavlink_manual_control_t &msg)
{
    nav_status.bearing = msg.x;
    nav_status.pitch   = msg.y;
    nav_status.distance = 0.0;
    nav_status.manual_control_yaw   = (msg.x != 0x7FFF);
    nav_status.manual_control_pitch = (msg.y != 0x7FFF);
    // z, r and buttons are not used
}

/**
   update_armed_disarmed - set armed LED if we have received a position update within the last 5 seconds
 */
void Tracker::update_armed_disarmed()
{
    if (vehicle.last_update_ms != 0 && (AP_HAL::millis() - vehicle.last_update_ms) < TRACKING_TIMEOUT_MS) {
        AP_Notify::flags.armed = true;
    } else {
        AP_Notify::flags.armed = false;
    }
}
