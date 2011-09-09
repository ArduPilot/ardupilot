// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

void waypoint_check()
{
	if(g.waypoint_index > 1 && g.waypoint_index <=18){	// Between these waypoints it will do what you want
		if(wp_distance < 10){				// Get as close as it can for you
			servo_pic();
		}		// DO SOMETHIMNG
	}
	if(g.waypoint_index == 20){				// When here do whats underneath
		servo_pic();
	}
}

void picture_time_check()
{
	if (picture_time == 1){
		if (wp_distance < 10){
		servo_pic();			// or any camera activation command
		}
	}
}

void egg_waypoint()
{
	float temp  = (float)(current_loc.alt - home.alt) * .01;
	float egg_dist = sqrt(temp / 4.903) * (float)g_gps->ground_speed *.01;

	if(g.waypoint_index == 3){
		if(wp_distance < egg_dist){
			APM_RC.OutputCh(CH_RUDDER,1500 + (-45*10.31));
		}
	}else{
		APM_RC.OutputCh(CH_RUDDER,1500 + (45*10.31));
	}
}

void johann_check()	// if you aren't Johann it doesn't really matter :D
{
	APM_RC.OutputCh(CH_7,1500 + (350));
	if(g.waypoint_index > 1 && g.waypoint_index <=18){	// Between these waypoints it will do what you want
		if(wp_distance < 10){				// Get as close as it can for you
			servo_pic();
		}
	}
	if(g.waypoint_index == 20){				// When here do whats underneath
		servo_pic();
	}
}
