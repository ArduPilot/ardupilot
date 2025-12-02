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
        vehicle.location_estimate.offset(north_offset, east_offset);
    	vehicle.location_estimate.offset_up_m(vehicle.vel.z * dt);
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
    Location temp_loc;

    // REVISIT: what if we lose lock during a mission and the antenna is moving?
    if (ahrs.get_location(temp_loc)) {
        stationary = false;
        current_loc = temp_loc;
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
    if (mode != &mode_scan && !nav_status.manual_control_yaw) {
        nav_status.bearing  = current_loc.get_bearing_to(vehicle.location_estimate) * 0.01f;
    }

    // calculate distance to vehicle
    nav_status.distance = current_loc.get_distance(vehicle.location_estimate);

    // calculate altitude difference to vehicle using gps
    if (g.alt_source == ALT_SOURCE_GPS){
        nav_status.alt_difference_gps = (vehicle.location_estimate.alt - current_loc.alt) * 0.01f;
    } else {
        // g.alt_source == ALT_SOURCE_GPS_VEH_ONLY
        nav_status.alt_difference_gps = vehicle.relative_alt * 0.01f;
    }

    // calculate pitch to vehicle
    // To-Do: remove need for check of control_mode
    if (mode->number() != Mode::Number::SCAN && !nav_status.manual_control_pitch) {
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
    // do not move if we are not armed:
    if (!hal.util->get_soft_armed()) {
        switch ((PWMDisarmed)g.disarm_pwm.get()) {
        case PWMDisarmed::TRIM:
            SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
            break;
        default:
        case PWMDisarmed::ZERO:
            SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, 0);
            break;
        }
    } else {
        mode->update();
    }

    // convert servo_out to radio_out and send to servo
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    return;
}

/**
   handle an updated position from the aircraft
 */
void Tracker::tracking_update_position(const mavlink_global_position_int_t &msg)
{
    // reject (0;0) coordinates
    if (!msg.lat && !msg.lon) {
        return;
    }

    vehicle.location.lat = msg.lat;
    vehicle.location.lng = msg.lon;
    vehicle.location.alt = msg.alt/10;
    vehicle.relative_alt = msg.relative_alt/10;
    vehicle.vel = Vector3f(msg.vx*0.01f, msg.vy*0.01f, msg.vz*0.01f);
    vehicle.last_update_us = AP_HAL::micros();
    vehicle.last_update_ms = AP_HAL::millis();
#if HAL_LOGGING_ENABLED
    // log vehicle as VPOS
    if (should_log(MASK_LOG_GPS)) {
        Log_Write_Vehicle_Pos(vehicle.location.lat, vehicle.location.lng, vehicle.location.alt, vehicle.vel);
    }
#endif
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

#if HAL_LOGGING_ENABLED
    // log vehicle baro data
    Log_Write_Vehicle_Baro(aircraft_pressure, alt_diff);
#endif
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
void Tracker::update_armed_disarmed() const
{
    if (vehicle.last_update_ms != 0 && (AP_HAL::millis() - vehicle.last_update_ms) < TRACKING_TIMEOUT_MS) {
        AP_Notify::flags.armed = true;
    } else {
        AP_Notify::flags.armed = false;
    }
}

/*
  Returns the pan and tilt for use by onvif camera in scripting
  the output will be mapped to -1..1 from limits specified by PITCH_MIN
  and PITCH_MAX for tilt, and YAW_RANGE for pan
*/
bool Tracker::get_pan_tilt_norm(float &pan_norm, float &tilt_norm) const
{
    float pitch = nav_status.pitch;
    float bearing = nav_status.bearing;
    // set tilt value
    tilt_norm = (((constrain_float(pitch+g.pitch_trim, g.pitch_min, g.pitch_max) - g.pitch_min)*2.0f)/(g.pitch_max - g.pitch_min)) - 1;
    // set yaw value
    pan_norm = (wrap_360(bearing+g.yaw_trim)*2.0f/(g.yaw_range)) - 1;
    return true;
}
