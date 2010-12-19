// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (GPS.fix == 0)
	{
		GPS.new_data = false;
		return;
	}
	if(next_WP.lat == 0){
		return;
	}
	
	// We only perform most nav computations if we have new gps data to work with
	// --------------------------------------------------------------------------
	if(GPS.new_data){
		GPS.new_data = false;
		
		// waypoint distance from plane
		// ----------------------------
		GPS_wp_distance = getDistance(&current_loc, &next_WP);

		if (GPS_wp_distance < 0){
			send_message(SEVERITY_HIGH,"<navigate> WP error - distance < 0");
			//Serial.println(wp_distance,DEC);
			//print_current_waypoints();
			return;
		}

		// target_bearing is where we should be heading 
		// --------------------------------------------
		target_bearing 	= get_bearing(&current_loc, &next_WP);
	
		// nav_bearing will includes xtrack correction
		// -------------------------------------------
		nav_bearing = target_bearing;

		// look for a big angle change
		// ---------------------------
		//verify_missed_wp();

		// control mode specific updates to nav_bearing
		// --------------------------------------------
		update_navigation();
	}
}

void verify_missed_wp()
{
	// check if we have missed the WP
	loiter_delta = (target_bearing - old_target_bearing) / 100;
	
	// reset the old value
	old_target_bearing = target_bearing;
	
	// wrap values
	if (loiter_delta > 170) loiter_delta -= 360;
	if (loiter_delta < -170) loiter_delta += 360;
	loiter_sum 		+= abs(loiter_delta);
}

void calc_bearing_error()
{
	bearing_error 	= nav_bearing - yaw_sensor;
	bearing_error 	= wrap_180(bearing_error);
}

void calc_yaw_error()
{
	yaw_error = nav_yaw - yaw_sensor;
	yaw_error = wrap_180(yaw_error);
}

void calc_distance_error()
{
	wp_distance = GPS_wp_distance;
	
	// this wants to work only while moving, but it should filter out jumpy GPS reads
	//						scale gs to whole deg		(50hz / 100)	scale bearing error down to whole deg
	//distance_estimate 	+= (float)GPS.ground_speed * 	.0002 * 		cos(radians(bearing_error / 100));
	//distance_estimate 	-= distance_gain * (float)(distance_estimate - GPS_wp_distance);
	//wp_distance			=  distance_estimate;
}

/*void calc_airspeed_errors()
{
	//airspeed_error = airspeed_cruise - airspeed;
	//airspeed_energy_error = (long)(((long)airspeed_cruise * (long)airspeed_cruise) - ((long)airspeed * (long)airspeed))/20000; //Changed 0.00005f * to / 20000 to avoid floating point calculation
} */

// calculated at 50 hz
void calc_altitude_error() 
{
	altitude_error 		= next_WP.alt - current_loc.alt;

	// limit climb rates
	//target_altitude = next_WP.alt - ((float)((wp_distance -30) * offset_altitude) / (float)(wp_totalDistance - 30));
	
	//if(prev_WP.alt > next_WP.alt){
//		target_altitude = constrain(target_altitude, next_WP.alt, prev_WP.alt);
//	}else{
//		target_altitude = constrain(target_altitude, prev_WP.alt, next_WP.alt);
//	}
	
	// calc the GPS/Abs pressure altitude
	//if(GPS.fix)
	//	pressure_altitude 	+= altitude_gain * (float)(GPS.altitude - pressure_altitude);
	//current_loc.alt		= pressure_altitude;
//	altitude_error 		= target_altitude - current_loc.alt;

	//Serial.printf("s: %d %d t_alt %d\n", (int)current_loc.alt, (int)altitude_error, (int)target_altitude);
}

long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

/*
// disabled for now
void update_loiter()
{
	loiter_delta = (target_bearing - old_target_bearing) / 100;
	// reset the old value
	old_target_bearing = target_bearing;
	// wrap values
	if (loiter_delta > 170) loiter_delta -= 360;
	if (loiter_delta < -170) loiter_delta += 360;
	loiter_sum += loiter_delta;
} */

void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (abs(target_bearing - crosstrack_bearing) < 4500) {	 // If we are too far off or too close we don't do track following
		crosstrack_error = sin(radians((target_bearing - crosstrack_bearing) / 100)) * wp_distance;	 // Meters we are off track line
		nav_bearing += constrain(crosstrack_error * x_track_gain, -x_track_angle, x_track_angle);
		nav_bearing = wrap_360(nav_bearing);
	}
}

void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&current_loc, &next_WP);	// Used for track following
}

int get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}

long getDistance(struct Location *loc1, struct Location *loc2)
{
	if(loc1->lat == 0 || loc1->lng == 0) 
		return -1;
	if(loc2->lat == 0 || loc2->lng == 0) 
		return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

long get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}

long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
