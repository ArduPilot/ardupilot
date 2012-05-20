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

#if HIL_MODE != HIL_MODE_ATTITUDE
	if((next_WP.lat == 0)||(home_is_set==false)){
#else
	if(next_WP.lat == 0){
#endif
		return;
	}

         if(control_mode < INITIALISING) {

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
                }    

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


#if 0
// Disabled for now
void calc_distance_error()
{
	distance_estimate 	+= (float)ground_speed * .0002 * cos(radians(bearing_error * .01));
	distance_estimate 	-= DST_EST_GAIN * (float)(distance_estimate - GPS_wp_distance);
	wp_distance  		= max(distance_estimate,10);
}
#endif

static void calc_gndspeed_undershoot()
{
    // Function is overkill, but here in case we want to add filtering later
    groundspeed_undershoot = (g.min_gndspeed > 0) ? (g.min_gndspeed - ground_speed) : 0;
}

static void calc_bearing_error()
{    
#if LITE == DISABLED  
      #if CONFIG_SONAR == ENABLED
	if((g.sonar_enabled) && (sonar_dist < g.sonar_trigger)) {
	    nav_bearing += 9000;    // if obstacle in front turn 90° right	 
	}
      #endif
#endif
	bearing_error = nav_bearing - ground_course;
	bearing_error = wrap_180(bearing_error);
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

static void calc_turn_radius()    // JLN update - adjut automaticaly the wp_radius Vs the speed and the turn angle
{
  wp_radius = ground_speed * 150 / g.roll_limit.get();
  //Serial.println(wp_radius, DEC);
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

void reached_waypoint()
{       

}

