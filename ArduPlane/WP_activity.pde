// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CAMERA == ENABLED
void waypoint_check()
{
	if(g.waypoint_index > 1 && g.waypoint_index <=18){	// Between these waypoints it will do what you want
		if(wp_distance < g.camera.wp_distance_min){		// Get as close as it can for you
			g.camera.trigger_pic();
		}		// DO SOMETHIMNG
	}
	if(g.waypoint_index == 20){				// When here do whats underneath
		g.camera.trigger_pic();
	}
}

void picture_time_check()
{
	if (g.camera.picture_time == 1){
		if (wp_distance < g.camera.wp_distance_min){
			g.camera.trigger_pic();			// or any camera activation command
		}
	}
}
#endif

void egg_waypoint()
{
	if (g_rc_function[RC_Channel_aux::k_egg_drop])
	{
		float temp  = (float)(current_loc.alt - home.alt) * .01;
		float egg_dist = sqrt(temp / 4.903) * (float)g_gps->ground_speed *.01;

		if(g.waypoint_index == 3){
			if(wp_distance < egg_dist){
				g_rc_function[RC_Channel_aux::k_egg_drop]->servo_out = 100;
			}
		}else{
			g_rc_function[RC_Channel_aux::k_egg_drop]->servo_out = 0;
		}
	}
}

#if CAMERA == ENABLED
void johann_check()	// if you aren't Johann it doesn't really matter :D
{
	APM_RC.OutputCh(CH_7,1500 + (350));
	if(g.waypoint_index > 1 && g.waypoint_index <=18){	// Between these waypoints it will do what you want
		if(wp_distance < g.camera.wp_distance_min){		// Get as close as it can for you
			g.camera.trigger_pic();
		}
	}
	if(g.waypoint_index == 20){				// When here do whats underneath
		g.camera.trigger_pic();
	}
}
#endif
