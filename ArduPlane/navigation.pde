// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (g_gps->fix == 0)
	{
		g_gps->new_data = false;
		return;
	}

	if(next_WP.lat == 0){
		return;
	}

	// waypoint distance from plane
	// ----------------------------
	wp_distance = get_distance(&current_loc, &next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		//Serial.println(wp_distance,DEC);
		return;
	}

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 	= get_bearing(&current_loc, &next_WP);

	// nav_bearing will includes xtrac correction
	// ------------------------------------------
	nav_bearing = target_bearing;

	// check if we have missed the WP
	loiter_delta = (target_bearing - old_target_bearing)/100;

	// reset the old value
	old_target_bearing = target_bearing;

	// wrap values
	if (loiter_delta > 180) loiter_delta -= 360;
	if (loiter_delta < -180) loiter_delta += 360;
	loiter_sum += abs(loiter_delta);

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


#if 0
// Disabled for now
void calc_distance_error()
{
	distance_estimate 	+= (float)g_gps->ground_speed * .0002 * cos(radians(bearing_error * .01));
	distance_estimate 	-= DST_EST_GAIN * (float)(distance_estimate - GPS_wp_distance);
	wp_distance  		= max(distance_estimate,10);
}
#endif

static void calc_airspeed_errors()
{
    // Normal airspeed target
    target_airspeed = g.airspeed_cruise;

    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B) {
        target_airspeed = ((int)(g.flybywire_airspeed_max -
                                 g.flybywire_airspeed_min) *
                           g.channel_throttle.servo_out) +
                          ((int)g.flybywire_airspeed_min * 100);
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode >= FLY_BY_WIRE_B && (g.min_gndspeed > 0)) {
        long min_gnd_target_airspeed = airspeed + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed)
            target_airspeed = min_gnd_target_airspeed;
    }

    // Bump up the target airspeed based on throttle nudging
    if (control_mode >= AUTO && airspeed_nudge > 0) {
        target_airspeed += airspeed_nudge;
    }

    // Apply airspeed limit
    if (target_airspeed > (g.flybywire_airspeed_max * 100))
        target_airspeed = (g.flybywire_airspeed_max * 100);

    airspeed_error = target_airspeed - airspeed;
    airspeed_energy_error = ((target_airspeed * target_airspeed) - ((long)airspeed * (long)airspeed))/20000; //Changed 0.00005f * to / 20000 to avoid floating point calculation
}

static void calc_gndspeed_undershoot()
{
    // Function is overkill, but here in case we want to add filtering later
    groundspeed_undershoot = (g.min_gndspeed > 0) ? (g.min_gndspeed - g_gps->ground_speed) : 0;
}

static void calc_bearing_error()
{
	if(takeoff_complete == true  || g.compass_enabled == true) {
        /*
          most of the time we use the yaw sensor for heading, even if
          we don't have a compass. The yaw sensor is drift corrected
          in the DCM library. We only use the gps ground course
          directly if we haven't completed takeoff, as the yaw drift
          correction won't have had a chance to kick in. Drift
          correction using the GPS typically takes 10 seconds or so
          for a 180 degree correction.
         */
		bearing_error = nav_bearing - dcm.yaw_sensor;
	} else {

		// TODO: we need to use the Yaw gyro for in between GPS reads,
		// maybe as an offset from a saved gryo value.
		bearing_error = nav_bearing - g_gps->ground_course;
	}

	bearing_error = wrap_180(bearing_error);
}

static void calc_altitude_error()
{
	if(control_mode == AUTO && offset_altitude != 0) {
		// limit climb rates
		target_altitude = next_WP.alt - ((float)((wp_distance -30) * offset_altitude) / (float)(wp_totalDistance - 30));

		// stay within a certain range
		if(prev_WP.alt > next_WP.alt){
			target_altitude = constrain(target_altitude, next_WP.alt, prev_WP.alt);
		}else{
			target_altitude = constrain(target_altitude, prev_WP.alt, next_WP.alt);
		}
	} else if (non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) {
		target_altitude = next_WP.alt;
	}

	/*
	// Disabled for now
	#if AIRSPEED_SENSOR == 1
		long    altitude_estimate;                  // for smoothing GPS output

		// special thanks to Ryan Beall for this one
		float pitch_angle 	= pitch_sensor - g.pitch_trim; // pitch_angle = pitch sensor - angle of attack of your plane at level *100 (50 = .5°)
		pitch_angle			= constrain(pitch_angle, -2000, 2000);
		float scale			= sin(radians(pitch_angle * .01));
		altitude_estimate 	+= (float)airspeed * .0002 * scale;
		altitude_estimate 	-= ALT_EST_GAIN * (float)(altitude_estimate - current_loc.alt);

		// compute altitude error for throttle control
		altitude_error  = target_altitude - altitude_estimate;
	#else
		altitude_error 	= target_altitude - current_loc.alt;
	#endif
	*/

	altitude_error 	= target_altitude - current_loc.alt;
}

static long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

static long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

static void update_loiter()
{
	float power;

	if(wp_distance <= g.loiter_radius){
		power = float(wp_distance) / float(g.loiter_radius);
		power = constrain(power, 0.5, 1);
		nav_bearing += (int)(9000.0 * (2.0 + power));
	}else if(wp_distance < (g.loiter_radius + LOITER_RANGE)){
		power = -((float)(wp_distance - g.loiter_radius - LOITER_RANGE) / LOITER_RANGE);
		power = constrain(power, 0.5, 1);			//power = constrain(power, 0, 1);
		nav_bearing -= power * 9000;

	}else{
		update_crosstrack();
		loiter_time = millis();			// keep start time for loiter updating till we get within LOITER_RANGE of orbit

	}
/*
	if (wp_distance < g.loiter_radius){
		nav_bearing += 9000;
	}else{
		nav_bearing -= 100 * M_PI / 180 * asin(g.loiter_radius / wp_distance);
	}

	update_crosstrack();
*/
	nav_bearing = wrap_360(nav_bearing);
}

static void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (abs(wrap_180(target_bearing - crosstrack_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
		crosstrack_error = sin(radians((target_bearing - crosstrack_bearing) / (float)100)) * (float)wp_distance;	 // Meters we are off track line
		nav_bearing += constrain(crosstrack_error * g.crosstrack_gain, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
		nav_bearing = wrap_360(nav_bearing);
	}
}

static void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&prev_WP, &next_WP);	// Used for track following
}

static long get_distance(struct Location *loc1, struct Location *loc2)
{
	if(loc1->lat == 0 || loc1->lng == 0)
		return -1;
	if(loc2->lat == 0 || loc2->lng == 0)
		return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

static long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
