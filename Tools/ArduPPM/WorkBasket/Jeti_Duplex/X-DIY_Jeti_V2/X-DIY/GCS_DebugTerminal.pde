// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#if GCS_PROTOCOL == GCS_PROTOCOL_DEBUGTERMINAL

/* This GCS protocol sends text-base information over the telemetry port
*/

#include <avr/pgmspace.h>	//Allow us to store string constants (there are a lot!) in program memory to avoid using up valuable RAM

#define ERR(a) ((DEBUGTERMINAL_VERBOSE)>0?(PSTR(a)):(0))

#if GCS_PORT == 3
# define DTSTREAM	Serial3
#else
# define DTSTREAM	Serial
#endif

//Read buffer
#define DEBUGTERM_BUFFERSIZE (120)
char gcsinputbuffer[DEBUGTERM_BUFFERSIZE];
byte bufferidx;

//Reporting flags
byte report_heartbeat = 1;
byte report_attitude = 0;
byte report_location = 0;
byte report_command = 1;
byte report_severity = 0;
byte first_location = 0;

void readgcsinput() {
	byte numc, i, c;
	
	numc = DTSTREAM.available();
	for (i=0;i<numc;i++) {
		c = DTSTREAM.read();   
		processgcsinput(c);
	}
}

void processgcsinput(char c) {
	if (c==8) {
		//Received a backspace character; move the buffer index backwards
		if (bufferidx > 0) bufferidx--;
	} else if (c==10) {
		//Received a linefeed; do nothing
	} else if (c==13) {
		//Received a carriage return; process command
		gcsinputbuffer[bufferidx] = 0;
		run_debugt_command(gcsinputbuffer);
		bufferidx = 0;
	} else {
		gcsinputbuffer[bufferidx++] = c;
		if (bufferidx >= DEBUGTERM_BUFFERSIZE) bufferidx = 0;
	}
}
		
void run_debugt_command(char *buf) {
	//*********** Ignore comments ***********
	if (strmatch(buf, PSTR("//"))) {
		
		//*********** Process 'show' commands ***********
	} else if (strmatch(buf, PSTR("show "))) {
		if (strmatch(buf+5, PSTR("heartbeat")))
			report_heartbeat = 1;
		else if (strmatch(buf+5, PSTR("attitude")))
			report_attitude = 1;
		else if (strmatch(buf+5, PSTR("location")))
			report_location = 1;
		else if (strmatch(buf+5, PSTR("command")))
			report_command = 1;
		else if (strmatch(buf+5, PSTR("severity")))
			report_severity = atoi(buf+14);
		else
			print_error(ERR("USAGE: show {heartbeat|attitude|location|command|severity <X>}"));
				
		//*********** Process 'hide' commands ***********
	} else if (strmatch(buf, PSTR("hide "))) {
		if (strmatch(buf+5, PSTR("heartbeat")))
			report_heartbeat = 0;
		else if (strmatch(buf+5, PSTR("attitude")))
			report_attitude = 0;
		else if (strmatch(buf+5, PSTR("location")))
			report_location = 0;
		else if (strmatch(buf+5, PSTR("command")))
			report_command = 0;
		else if (strmatch(buf+5, PSTR("all"))) {
			report_heartbeat = 0;
			report_attitude = 0;
			report_location = 0;
			report_command = 0;
		} else
			print_error(ERR("USAGE: hide {heartbeat|attitude|location|command|all}"));

		//*********** Process 'echo' command ***********
	} else if (strmatch(buf, PSTR("echo "))) {
		DTSTREAM.println(buf+5);
			
		//*********** Process 'groundstart' command ***********
	} else if (strmatch(buf, PSTR("groundstart"))) {
		startup_ground();
				
		//*********** Process 'inithome' command ***********
	} else if (strmatch(buf, PSTR("inithome"))) {
		init_home();
		DTSTREAM.println_P("Home set.");
  				
		//*********** Process 'print' commands ***********
	} else if (strmatch(buf, PSTR("print "))) {
		//------- print altitude -------
		if (strmatch(buf+6, PSTR("altitude"))) {
			DTSTREAM.println_P("Altitude:");
			DTSTREAM.print_P("  Pressure: ");  DTSTREAM.print(((float)press_alt)/100,2);  DTSTREAM.println_P(" m");
			DTSTREAM.print_P("  GPS: ");  DTSTREAM.print(((float)GPS.altitude)/100,2);  DTSTREAM.println_P(" m");
			DTSTREAM.print_P("  Mix ratio: ");  DTSTREAM.println(altitude_mix,3);
			DTSTREAM.print_P("  Mix: ");  DTSTREAM.print((float)(((1 - altitude_mix) * GPS.altitude) + (altitude_mix * press_alt))/100,2);  DTSTREAM.println_P(" m");
				
			//------- print attitude -------
		} else if (strmatch(buf+6, PSTR("attitude"))) {
			print_attitude();
				
			//------- print commands -------
		} else if (strmatch(buf+6, PSTR("commands"))) {
			print_waypoints();
				
			//------- print ctrlmode -------
		} else if (strmatch(buf+6, PSTR("ctrlmode"))) {
			print_control_mode();
				
			//------- print curwaypts -------
		} else if (strmatch(buf+6, PSTR("curwaypts"))) {
			print_current_waypoints();

			//------- print flightmodes -------
		} else if (strmatch(buf+6, PSTR("flightmodes"))) {
			int i;
			DTSTREAM.print_P("EEPROM read: ");
			for (i=0; i<6; i++) {
				DTSTREAM.print_P("Ch ");  DTSTREAM.print(i,DEC);  DTSTREAM.print_P(" = ");  DTSTREAM.print(flight_modes[i],DEC);  DTSTREAM.print_P(", ");
			}
			DTSTREAM.println(" ");
				
			//------- print k -? -------
		} else if (strmatch(buf+6, PSTR("k -?"))) {
			print_error(ERR("USAGE: print k{p|i|d} {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt}"));
			print_error(ERR("USAGE: print kff {pitchcomp|ruddermix|pitchtothrottle}"));
				
			//------- print kp -------
		} else if (strmatch(buf+6, PSTR("kp "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				DTSTREAM.print_P("P gain for ");
				DTSTREAM.print(buf+9);
				DTSTREAM.print_P(" = ");
				DTSTREAM.println(kp[i],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize P keyword; type print k -? for more information"));
					
			//------- print ki -------
		} else if (strmatch(buf+6, PSTR("ki "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				DTSTREAM.print_P("I gain for ");
				DTSTREAM.print(buf+9);
				DTSTREAM.print_P(" = ");
				DTSTREAM.println(ki[i],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize I keyword; type print k -? for more information"));
					
			//------- print kd -------
		} else if (strmatch(buf+6, PSTR("kd "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				DTSTREAM.print_P("D gain for ");
				DTSTREAM.print(buf+9);
				DTSTREAM.print_P(" = ");
				DTSTREAM.println(kd[i],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize D keyword; type print k -? for more information"));
					
			//------- print kff -------
		} else if (strmatch(buf+6, PSTR("kff "))) {
			if (strmatch(buf+10, PSTR("pitchcomp"))) {
				DTSTREAM.print_P("FF gain for pitchcomp = ");
				DTSTREAM.println(kff[CASE_PITCH_COMP],DEC);
			} else if (strmatch(buf+10, PSTR("ruddermix"))) {
				DTSTREAM.print_P("FF gain for ruddermix = ");
				DTSTREAM.println(kff[CASE_RUDDER_MIX],DEC);
			} else if (strmatch(buf+10, PSTR("pitchtothrottle"))) {
				DTSTREAM.print_P("FF gain for pitchtothrottle = ");
				DTSTREAM.println(kff[CASE_P_TO_T],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize FF keyword; type print k -? for more information"));
				
			//------- print location -------
		} else if (strmatch(buf+6, PSTR("location"))) {
			print_position();
				
			//------- print navsettings -------
		} else if (strmatch(buf+6, PSTR("navsettings"))) {
			DTSTREAM.println_P("Navigation settings:");
#if AIRSPEED_SENSOR == 1
			DTSTREAM.print_P("  Cruise airspeed: ");  DTSTREAM.println((float)airspeed_cruise/100.0,2);
#else
			DTSTREAM.print_P("  Cruise throttle: ");  DTSTREAM.println(throttle_cruise,DEC);
#endif
			DTSTREAM.print_P("  Hold altitude above home: ");  DTSTREAM.println(read_alt_to_hold(),DEC);
			DTSTREAM.print_P("  Loiter radius: ");  DTSTREAM.println(loiter_radius,DEC);
			DTSTREAM.print_P("  Waypoint radius: ");  DTSTREAM.println(wp_radius,DEC);
				
			//------- print pressure -------
		} else if (strmatch(buf+6, PSTR("pressure"))) {
			DTSTREAM.println_P("BMP085 pressure sensor:");
			DTSTREAM.print_P("  Temperature: ");  DTSTREAM.println(APM_BMP085.Temp,DEC);
			DTSTREAM.print_P("  Pressure: ");  DTSTREAM.println(APM_BMP085.Press,DEC);
				
			//------- print rlocation home -------
		} else if (strmatch(buf+6, PSTR("rlocation home"))) {
			print_rlocation(&home);
				
			//------- print rlocation -------
			//(implication is "relative to next waypoint")
		} else if (strmatch(buf+6, PSTR("rlocation"))) {
			print_rlocation(&next_WP);
				
			//------- print speed -------
		} else if (strmatch(buf+6, PSTR("speed"))) {
			DTSTREAM.println_P("Speed:");
			DTSTREAM.print_P("  Ground: ");  DTSTREAM.println((float)GPS.ground_speed/100.0,2);
#if AIRSPEED_SENSOR == 1
			DTSTREAM.print_P("  Air: ");  DTSTREAM.println((float)airspeed/100.0,2);
			DTSTREAM.print_P("  Cruise: ");  DTSTREAM.println((float)airspeed_cruise/100.0,2);
#endif
				
			//------- print tuning -------
		} else if (strmatch(buf+6, PSTR("tuning"))) {
			print_tuning();
				
		} else
			print_error(ERR("USAGE: print {altitude|attitude|commands|ctrlmode|curwaypts|flightmodes|k -?|kp|ki|kd|kff|location|navsettings|pressure|rlocation [home]|speed|tuning|}"));
		
		//*********** Process 'reset' commands ***********
	} else if (strmatch(buf, PSTR("reset "))) {
		//------- reset commands -------
		if (strmatch(buf+6, PSTR("commands"))) {
			reload_commands();
			DTSTREAM.println_P("Commands reloaded.");
		} else
			print_error(ERR("USAGE: reset commands"));
				
		//*********** Process 'rtl' command ***********
	} else if (strmatch(buf, PSTR("rtl"))) {
		return_to_launch();
		DTSTREAM.println_P("Returning to launch...");
				
		//*********** Process 'set' commands ***********
	} else if (strmatch(buf, PSTR("set "))) {
		//------- set cmd -------
		if (strmatch(buf+4, PSTR("cmd "))) {
			process_set_cmd(buf+8, bufferidx-8);

			//------- set cmdcount -------
		} else if (strmatch(buf+4, PSTR("cmdcount "))) {
			wp_total = atoi(buf+13);
			save_EEPROM_waypoint_info();
			DTSTREAM.print_P("wp_total = ");  DTSTREAM.println(wp_total,DEC);
			
			//------- set cmdindex -------
		} else if (strmatch(buf+4, PSTR("cmdindex "))) {
			wp_index = atoi(buf+13);
			decrement_WP_index();
			next_command = get_wp_with_index(wp_index+1);
			DTSTREAM.print_P("Command set to index ");  DTSTREAM.print(wp_index,DEC);
			if (next_command.id >= 0x10 && next_command.id <= 0x1F) { //TODO: create a function the defines what type of command each command ID is
				command_must_index = 0;
				DTSTREAM.println_P(" (must)");
			} else if (next_command.id >= 0x20 && next_command.id <= 0x2F) {
				command_may_index = 0;
				DTSTREAM.println_P(" (may)");
			} else
				DTSTREAM.println_P(" (now)");

			next_command.id = CMD_BLANK;
			if (wp_index > wp_total) {
				wp_total = wp_index;
				DTSTREAM.print_P("  The total number of commands is now ");
				DTSTREAM.println(wp_total,DEC);
			}
			update_commands();
			
			//------- set cruise -------
		} else if (strmatch(buf+4, PSTR("cruise "))) {
			unsigned char j = 4+7;
#if AIRSPEED_SENSOR == 1
			DTSTREAM.print_P("airspeed_cruise changed from ");
			DTSTREAM.print((float)airspeed_cruise/100,2);
			DTSTREAM.print_P(" to ");
			airspeed_cruise = (int)(readfloat(buf, &j)/100000);
			airspeed_cruise = constrain(airspeed_cruise,0,9000); //TODO: constrain minimum as stall speed, maximum as Vne
			DTSTREAM.println(((float)airspeed_cruise)/100,2);
#else
			DTSTREAM.print_P("throttle_cruise changed from ");
			DTSTREAM.print(throttle_cruise,DEC);
			DTSTREAM.print_P(" to ");
			throttle_cruise = constrain((int)(readfloat(buf, &j)/10000000),0,200);
			DTSTREAM.println(throttle_cruise,DEC);
#endif

			//------- set holdalt -------
		} else if (strmatch(buf+4, PSTR("holdalt "))) {
			int tempalt = atoi(buf+12)*100;
			save_alt_to_hold((int32_t)tempalt);
			DTSTREAM.println_P("Hold altitude above home set.");
			
			//------- set k -? -------
		} else if (strmatch(buf+4, PSTR("k -?"))) {
			print_error(ERR("USAGE: set k{p|i|d} {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} <X>"));
			print_error(ERR("USAGE: set kff {pitchcomp|ruddermix|pitchtothrottle} <X>"));
				
			//------- set kp -------
		} else if (strmatch(buf+4, PSTR("kp "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+7, &i, &j)) {
				buf[7+j] = 0;
				DTSTREAM.print_P("P gain for ");
				DTSTREAM.print(buf+7);
				DTSTREAM.print_P(" changed from ");
				DTSTREAM.print(kp[i],DEC);
				DTSTREAM.print_P(" to ");
				j += 7+1;
				kp[i] = ((float)readfloat(buf, &j))/10000000;
				DTSTREAM.println(kp[i],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize P keyword; type set k -? for more information"));
					
			//------- set ki -------
		} else if (strmatch(buf+4, PSTR("ki "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+7, &i, &j)) {
				buf[7+j] = 0;
				DTSTREAM.print_P("I gain for ");
				DTSTREAM.print(buf+7);
				DTSTREAM.print_P(" changed from ");
				DTSTREAM.print(ki[i],DEC);
				DTSTREAM.print_P(" to ");
				j += 7+1;
				ki[i] = ((float)readfloat(buf, &j))/10000000;
				DTSTREAM.println(ki[i],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize I keyword; type set k -? for more information"));
					
			//------- set kd -------
		} else if (strmatch(buf+4, PSTR("kd "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+7, &i, &j)) {
				buf[7+j] = 0;
				DTSTREAM.print_P("D gain for ");
				DTSTREAM.print(buf+7);
				DTSTREAM.print_P(" changed from ");
				DTSTREAM.print(kd[i],DEC);
				DTSTREAM.print_P(" to ");
				j += 7+1;
				kd[i] = ((float)readfloat(buf, &j))/10000000;
				DTSTREAM.println(kd[i],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize D keyword; type set k -? for more information"));
					
			//------- set kff -------
		} else if (strmatch(buf+4, PSTR("kff "))) {
			unsigned char j = 0;
			if (strmatch(buf+8, PSTR("pitchcomp"))) {
				buf[8+9] = 0;
				DTSTREAM.print_P("FF gain for ");
				DTSTREAM.print(buf+8);
				DTSTREAM.print_P(" changed from ");
				DTSTREAM.print(kff[CASE_PITCH_COMP],DEC);
				DTSTREAM.print_P(" to ");
				j = 8+9+1;
				kff[CASE_PITCH_COMP] = ((float)readfloat(buf, &j))/10000000;
				DTSTREAM.println(kff[CASE_PITCH_COMP],DEC);
			} else if (strmatch(buf+8, PSTR("ruddermix"))) {
				buf[8+9] = 0;
				DTSTREAM.print_P("FF gain for ");
				DTSTREAM.print(buf+8);
				DTSTREAM.print_P(" changed from ");
				DTSTREAM.print(kff[CASE_RUDDER_MIX],DEC);
				DTSTREAM.print_P(" to ");
				j = 8+9+1;
				kff[CASE_RUDDER_MIX] = ((float)readfloat(buf, &j))/10000000;
				DTSTREAM.println(kff[CASE_RUDDER_MIX],DEC);
			} else if (strmatch(buf+8, PSTR("pitchtothrottle"))) {
				buf[8+15] = 0;
				DTSTREAM.print_P("FF gain for ");
				DTSTREAM.print(buf+8);
				DTSTREAM.print_P(" changed from ");
				DTSTREAM.print(kff[CASE_P_TO_T],DEC);
				DTSTREAM.print_P(" to ");
				j = 8+15+1;
				kff[CASE_P_TO_T] = ((float)readfloat(buf, &j))/10000000;
				DTSTREAM.println(kff[CASE_P_TO_T],DEC);
			} else
				print_error(ERR("ERROR: Did not recognize FF keyword; type print k -? for more information"));
				
			//------- set loiterradius -------
		} else if (strmatch(buf+4, PSTR("loiterradius "))) {
			loiter_radius = atoi(buf+17);
			save_EEPROM_waypoint_info();
			DTSTREAM.print_P("Set loiter radius to ");  DTSTREAM.print(loiter_radius,DEC);  DTSTREAM.println_P(" meters");

			//------- set rcin -------
		} else if (strmatch(buf+4, PSTR("rcin"))) {
			unsigned char index = buf[8]-'1';
			if (index < 8) {
				radio_in[index] = atoi(buf+9);
			} else
				print_error(ERR("USAGE: set rcin<N> <X>"));
				
			//------- set rcout -------
		} else if (strmatch(buf+4, PSTR("rcout"))) {
			unsigned char index = buf[9]-'1';
			if (index < 8) {
				radio_out[index] = atoi(buf+10);
				APM_RC.OutputCh(index, radio_out[index]);
			} else
				print_error(ERR("USAGE: set rcout<N> <X>"));

			//------- set wpradius -------
		} else if (strmatch(buf+4, PSTR("wpradius "))) {
			wp_radius = atoi(buf+13);
			save_EEPROM_waypoint_info();
			DTSTREAM.print_P("Set waypoint radius to ");  DTSTREAM.print(wp_radius,DEC);  DTSTREAM.println_P(" meters");

			//------- set xtrackentryangle -------
		} else if (strmatch(buf+4, PSTR("xtrackentryangle "))) {
			unsigned char j = 21;
			x_track_angle = readfloat(buf, &j)/100000;
			DTSTREAM.print_P("Crosstrack entry angle set to ");  DTSTREAM.println((float)x_track_angle/100,2);

			//------- set xtrackgain -------
		} else if (strmatch(buf+4, PSTR("xtrackgain "))) {
			unsigned char j = 15;
			x_track_gain = ((float)readfloat(buf, &j))/10000000;
			DTSTREAM.print_P("Crosstrack gain set to ");  DTSTREAM.println(x_track_gain,2);
				
		} else
			print_error(ERR("USAGE: set {cmd|cmdcount|cmdindex|cruise|holdalt|kp|ki|kd|kff|loiterradius|rcin|rcout|wpradius}"));
			
		//*********** Process 'status' commands ***********
	} else if (strmatch(buf, PSTR("status "))) {				
		//------- status control -------
		if (strmatch(buf+7, PSTR("control"))) {
			DTSTREAM.println_P("Control status:");
			DTSTREAM.print_P("  Roll: nav=");  DTSTREAM.print(nav_roll/100.0,2);  DTSTREAM.print_P(", servo_out=");  DTSTREAM.println((float)servo_out[CH_ROLL]/100.0,2);
			DTSTREAM.print_P("  Pitch: nav=");  DTSTREAM.print(nav_pitch/100.0,2);  DTSTREAM.print_P(", servo_out=");  DTSTREAM.println((float)servo_out[CH_PITCH]/100.0,2);
			DTSTREAM.print_P("  Throttle: nav=");  DTSTREAM.print(throttle_cruise,DEC);  DTSTREAM.print_P(", servo_out=");  DTSTREAM.println(servo_out[CH_THROTTLE],DEC);
			
			//------- status gps -------
		} else if (strmatch(buf+7, PSTR("gps"))) {
			DTSTREAM.println_P("GPS status:");
			DTSTREAM.print_P("  Fix: ");
			if (GPS.fix == 0)
				DTSTREAM.print_P("INVALID (");
			else
				DTSTREAM.print_P("Valid (");
			DTSTREAM.print(GPS.fix,DEC);
			DTSTREAM.println_P(")");
			DTSTREAM.print_P("  Satellites: ");  DTSTREAM.println(GPS.num_sats,DEC);
			DTSTREAM.print_P("  Fix count: "); DTSTREAM.println(gps_fix_count,DEC);
			
			//------- status landing -------	
		} else if (strmatch(buf+7, PSTR("landing"))) {
			DTSTREAM.println_P("Landing status:");
			DTSTREAM.print_P("  land_complete = ");  DTSTREAM.println(land_complete,DEC);
			DTSTREAM.print_P("  landing_pitch = ");  DTSTREAM.println(landing_pitch,DEC);
			DTSTREAM.print_P("  nav_pitch = ");  DTSTREAM.println(nav_pitch,DEC);
			DTSTREAM.print_P("  airspeed_cruise = ");  DTSTREAM.println(airspeed_cruise,DEC);
			DTSTREAM.print_P("  throttle_cruise = ");  DTSTREAM.println(throttle_cruise,DEC);
			DTSTREAM.print_P("  hold_course = ");  DTSTREAM.println(hold_course,DEC);
			DTSTREAM.print_P("  nav_bearing = ");  DTSTREAM.println(nav_bearing,DEC);
			DTSTREAM.print_P("  bearing_error = ");  DTSTREAM.println(bearing_error,DEC);
				
			//------- status loiter -------
		} else if (strmatch(buf+7, PSTR("loiter"))) {
			DTSTREAM.println_P("Loiter status:");
			DTSTREAM.print_P("  Loiter radius: ");  DTSTREAM.println(loiter_radius,DEC);
			DTSTREAM.print_P("  Progress: ");  DTSTREAM.print(loiter_sum,DEC);  DTSTREAM.print_P("°/");  DTSTREAM.print(loiter_total,DEC);  DTSTREAM.println_P("°");
			DTSTREAM.print_P("  Time: ");  DTSTREAM.print(loiter_time,DEC);  DTSTREAM.print_P("ms/");  DTSTREAM.print(loiter_time_max,DEC);  DTSTREAM.println_P("ms");
			
			//------- status navigation -------
		} else if (strmatch(buf+7, PSTR("navigation"))) {
			DTSTREAM.println_P("Navigation status:");
			DTSTREAM.print_P("  From <");  DTSTREAM.print((float)prev_WP.lat/10000000.0,6);  DTSTREAM.print_P(", ");  DTSTREAM.print((float)prev_WP.lng/10000000.0,6);  DTSTREAM.print_P(", ");  DTSTREAM.print((float)prev_WP.alt/100.0,1);  DTSTREAM.print_P(">: ");  print_rlocation(&prev_WP);
			DTSTREAM.print_P("  At <");  DTSTREAM.print((float)current_loc.lat/10000000.0,6);  DTSTREAM.print_P(", ");  DTSTREAM.print((float)current_loc.lng/10000000.0,6);  DTSTREAM.print_P(", ");  DTSTREAM.print((float)current_loc.alt/100.0,1);  DTSTREAM.println_P(">");
			DTSTREAM.print_P("  To <");  DTSTREAM.print((float)next_WP.lat/10000000.0,6);  DTSTREAM.print_P(", ");  DTSTREAM.print((float)next_WP.lng/10000000.0,6);  DTSTREAM.print_P(", ");  DTSTREAM.print((float)next_WP.alt/100.0,1);  DTSTREAM.print_P(">: ");  print_rlocation(&next_WP);
			DTSTREAM.print_P("  Distance: ");  DTSTREAM.print(100.0*(float)(wp_totalDistance-wp_distance)/(float)wp_totalDistance,1);  DTSTREAM.print_P("% ");  DTSTREAM.print(wp_totalDistance-wp_distance,DEC);  DTSTREAM.print_P("m / ");  DTSTREAM.print(wp_totalDistance,DEC);  DTSTREAM.print_P("m; ");  DTSTREAM.print(altitude_error/100.0,2);  DTSTREAM.println_P("m vertically");
			DTSTREAM.print_P("  Nav bearing: ");  DTSTREAM.print(nav_bearing/100.0,2);  DTSTREAM.print_P("; error = ");  DTSTREAM.println(bearing_error/100.0,2);
			DTSTREAM.print_P("  Ground course: ");  DTSTREAM.print(GPS.ground_course/100.0,1);  DTSTREAM.print_P(" (current), ");  DTSTREAM.print(target_bearing/100.0,1);  DTSTREAM.println_P(" (target)");
			if (hold_course >= 0) {
				DTSTREAM.print_P("  HOLD COURSE: ");  DTSTREAM.println(hold_course/100.0,2);
			}
				
			/*DTSTREAM.print_P("  Crosstrack bearing: ");  DTSTREAM.println(crosstrack_bearing/100.0,DEC);
			  DTSTREAM.print_P("  Crosstrack error: ");  DTSTREAM.println(crosstrack_error/100.0,DEC);
			  DTSTREAM.print_P("  Climb rate: ");  DTSTREAM.println(climb_rate/100.0,DEC);*/
				

			//------- status navpitch -------
		} else if (strmatch(buf+7, PSTR("navpitch"))) {
#if AIRSPEED_SENSOR == 1
			DTSTREAM.print_P("nav_pitch = PID[airspeed_error (");  DTSTREAM.print(airspeed_error/100.0,2);  DTSTREAM.print_P(") = airspeed_cruise (");  DTSTREAM.print((float)airspeed_cruise/100.0,2);  DTSTREAM.print_P(") - airspeed (");  DTSTREAM.print((float)airspeed/100.0,2);  DTSTREAM.println_P(")]");
#else
			DTSTREAM.print_P("nav_pitch = PID[altitude_error (");  DTSTREAM.print(altitude_error,DEC);  DTSTREAM.print_P(") = target_altitude (");  DTSTREAM.print(target_altitude,DEC);  DTSTREAM.print_P(") - current_loc.alt (");  DTSTREAM.print(current_loc.alt,DEC);  DTSTREAM.println_P(")]");
#endif
			DTSTREAM.print_P("nav_pitch (");  DTSTREAM.print((float)nav_pitch/100.0,2);  DTSTREAM.print_P(") - pitch_sensor (");  DTSTREAM.print((float)pitch_sensor/100.0,2);  DTSTREAM.print_P(") + pitch_comp (");  DTSTREAM.print(abs(roll_sensor*kff[CASE_PITCH_COMP])/100.0,2);  DTSTREAM.print_P(") = ");  DTSTREAM.println((float)(nav_pitch-pitch_sensor+abs(roll_sensor*kff[CASE_PITCH_COMP]))/100.0,2);
			DTSTREAM.print_P("servo_out[CH_PITCH] (");  DTSTREAM.print((float)servo_out[CH_PITCH]/100.0,2);  DTSTREAM.println_P(") = PID[nav_pitch + pitch_comp - pitch_sensor]");
				
			//------- status navroll -------
		} else if (strmatch(buf+7, PSTR("navroll"))) {
			print_rlocation(&next_WP);
			DTSTREAM.print_P("bearing_error (");  DTSTREAM.print(bearing_error,DEC);  DTSTREAM.print_P(") = nav_bearing (");  DTSTREAM.print(nav_bearing,DEC);  DTSTREAM.print_P(") - yaw_sensor (");  DTSTREAM.print(yaw_sensor,DEC);  DTSTREAM.println_P(")");
			DTSTREAM.print_P("nav_roll (");  DTSTREAM.print(nav_roll,DEC);  DTSTREAM.print_P(") - roll_sensor (");  DTSTREAM.print(roll_sensor,DEC);  DTSTREAM.print_P(") = ");  DTSTREAM.print(nav_roll-roll_sensor,DEC);  DTSTREAM.println_P(")");
			DTSTREAM.print_P("servo_out[CH_ROLL] = "); DTSTREAM.println(servo_out[CH_ROLL],DEC);
				
			//------- status rcinputch -------
		} else if (strmatch(buf+7, PSTR("rcinputch"))) {
			unsigned char i;
			DTSTREAM.println_P("RC hardware input:");
			for (i=0; i<8; i++) {
				DTSTREAM.print_P("  Ch");
				DTSTREAM.print(i+1,DEC);
				DTSTREAM.print_P(": ");
				DTSTREAM.println(APM_RC.InputCh(i));
			}

			//------- status rcin -------
		} else if (strmatch(buf+7, PSTR("rcin"))) {
			unsigned char i;
			DTSTREAM.println_P("RC software input:");
			for (i=0; i<8; i++) {
				DTSTREAM.print_P("  Ch");
				DTSTREAM.print(i+1,DEC);
				DTSTREAM.print_P(": ");
				DTSTREAM.println(radio_in[i]);
			}
				
			//------- status rcout -------
		} else if (strmatch(buf+7, PSTR("rcout"))) {
			unsigned char i;
			DTSTREAM.println_P("RC software output:");
			for (i=0; i<8; i++) {
				DTSTREAM.print_P("  Ch");
				DTSTREAM.print(i+1,DEC);
				DTSTREAM.print_P(": ");
				DTSTREAM.println(radio_out[i]);
			}

			//------- status rcpwm -------
		} else if (strmatch(buf+7, PSTR("rcpwm"))) {
			unsigned char i;
			DTSTREAM.println_P("RC hardware output:");
			for (i=0; i<8; i++) {
				DTSTREAM.print_P("  Ch");
				DTSTREAM.print(i+1,DEC);
				DTSTREAM.print_P(": ");
				DTSTREAM.println(readOutputCh(i));
			}
				
			//------- status rctrim -------
		} else if (strmatch(buf+7, PSTR("rctrim"))) {
			unsigned char i;
			DTSTREAM.println_P("RC trim:");
			for (i=0; i<8; i++) {
				DTSTREAM.print_P("  Ch");
				DTSTREAM.print(i+1,DEC);
				DTSTREAM.print_P(": ");
				DTSTREAM.println(radio_trim[i]);
			}
			DTSTREAM.print_P("  elevon1_trim = ");  DTSTREAM.println(elevon1_trim,DEC);
			DTSTREAM.print_P("  elevon2_trim = ");  DTSTREAM.println(elevon2_trim,DEC);

			//------- status rc -------
		} else if (strmatch(buf+7, PSTR("rc"))) {
			unsigned char i;
			DTSTREAM.println_P("RC:\tCh\tHWin\tSWtrim\tSWin\tServo\tSWout\tHWout\t");
			for (i=0; i<8; i++) {
				DTSTREAM.print_P("\t"); DTSTREAM.print(i+1,DEC);
				DTSTREAM.print_P("\t"); DTSTREAM.print(APM_RC.InputCh(i),DEC);
				DTSTREAM.print_P("\t"); DTSTREAM.print(radio_trim[i],DEC);
				DTSTREAM.print_P("\t"); DTSTREAM.print(radio_in[i],DEC);
				DTSTREAM.print_P("\t"); DTSTREAM.print(((float)servo_out[i])/100,2);
				DTSTREAM.print_P("\t"); DTSTREAM.print(radio_out[i],DEC);
				DTSTREAM.print_P("\t"); DTSTREAM.println(readOutputCh(i),DEC);
			}
				
			//------- status system -------	
		} else if (strmatch(buf+7, PSTR("system"))) {
			DTSTREAM.println_P("System status:");
			DTSTREAM.print_P("  Ground start: ");  if (ground_start) DTSTREAM.print_P("YES ("); else DTSTREAM.print_P("NO (");
			DTSTREAM.print(ground_start,DEC);  DTSTREAM.println_P(")");
			DTSTREAM.print_P("  Home: ");  if (home_is_set) DTSTREAM.print_P(" SET("); else DTSTREAM.print_P("NOT SET (");
			DTSTREAM.print(home_is_set,DEC);  DTSTREAM.println_P(")");
				
			//------- status takeoff -------	
		} else if (strmatch(buf+7, PSTR("takeoff"))) {
			DTSTREAM.println_P("Takeoff status:");
			DTSTREAM.print_P("  takeoff_pitch = ");  DTSTREAM.println((float)takeoff_pitch/100.0,2);
			DTSTREAM.print_P("  scaler = ");  DTSTREAM.println((float)airspeed/(float)airspeed_cruise,3);
			DTSTREAM.print_P("  nav_pitch = ");  DTSTREAM.println((float)nav_pitch/100.0,2);
			DTSTREAM.print_P("  throttle = ");  DTSTREAM.println(servo_out[CH_THROTTLE],DEC);
			DTSTREAM.print_P("  hold_course = ");  DTSTREAM.println((float)hold_course/100.0,2);
			DTSTREAM.print_P("  nav_bearing = ");  DTSTREAM.println(nav_bearing,DEC);
			DTSTREAM.print_P("  bearing_error = ");  DTSTREAM.println(bearing_error,DEC);
			DTSTREAM.print_P("  current_loc.alt = ");  DTSTREAM.println(current_loc.alt,DEC);
			DTSTREAM.print_P("  home.alt + takeoff_altitude = ");  DTSTREAM.println(home.alt + takeoff_altitude,DEC);
				
			//nav_pitch = constrain(nav_pitch, 0, takeoff_pitch * 100);
			//calc_nav_pitch();
			nav_pitch = constrain(nav_pitch, 0, takeoff_pitch * 100);
				
			//------- status telemetry -------
		} else if (strmatch(buf+7, PSTR("telemetry"))) {
			DTSTREAM.println_P("Telemetry status:");
			if (report_heartbeat) DTSTREAM.print_P("  Show "); else DTSTREAM.print_P("  Hide ");  DTSTREAM.println_P("heartbeat");
			if (report_location) DTSTREAM.print_P("  Show "); else DTSTREAM.print_P("  Hide ");  DTSTREAM.println_P("location");
			if (report_attitude) DTSTREAM.print_P("  Show "); else DTSTREAM.print_P("  Hide ");  DTSTREAM.println_P("attitude");
			if (report_command) DTSTREAM.print_P("  Show "); else DTSTREAM.print_P("  Hide ");  DTSTREAM.println_P("command");
			DTSTREAM.print_P("  Severity report level ");  DTSTREAM.println(report_severity,DEC);

			//------- status throttle -------
		} else if (strmatch(buf+7, PSTR("throttle"))) {
#if AIRSPEED_SENSOR == 1
			DTSTREAM.print_P("airspeed_energy_error (");  DTSTREAM.print(airspeed_energy_error,DEC);  DTSTREAM.print_P(") = 0.5 * (airspeed_cruise (");  DTSTREAM.print((float)airspeed_cruise/100.0,2);  DTSTREAM.print_P(")^2 - airspeed (");  DTSTREAM.print((float)airspeed/100.0,2);  DTSTREAM.println_P(")^2)");
			DTSTREAM.print_P("energy_error (");  DTSTREAM.print(energy_error,DEC);  DTSTREAM.print_P(") = airspeed_energy_error (");  DTSTREAM.print(airspeed_energy_error,DEC);  DTSTREAM.print_P(") + altitude_error*0.098 (");  DTSTREAM.print((long)((float)altitude_error*0.098),DEC);  DTSTREAM.println_P(")");
			DTSTREAM.print_P("servo_out[CH_THROTTLE] (");  DTSTREAM.print(servo_out[CH_THROTTLE],DEC);  DTSTREAM.println_P(") = PID[energy_error]");
#else
			DTSTREAM.print_P("altitude_error (");  DTSTREAM.print(altitude_error,DEC);  DTSTREAM.print_P(") = target_altitude (");  DTSTREAM.print(target_altitude,DEC);  DTSTREAM.print_P(") - current_loc.alt (");  DTSTREAM.print(current_loc.alt,DEC);  DTSTREAM.println_P(")");
			DTSTREAM.print_P("servo_out[CH_THROTTLE] (");  DTSTREAM.print(servo_out[CH_THROTTLE],DEC);  DTSTREAM.println_P(") = PID[altitude_error]");
#endif

			//------- status waypoints -------
		} else if (strmatch(buf+7, PSTR("waypoints"))) {
			DTSTREAM.println_P("Waypoints status:");
			DTSTREAM.print_P("  Distance: ");  DTSTREAM.print(wp_distance,DEC);  DTSTREAM.print_P("/");  DTSTREAM.println(wp_totalDistance,DEC);
			DTSTREAM.print_P("  Index: ");  DTSTREAM.print(wp_index,DEC);  DTSTREAM.print_P("/");  DTSTREAM.println(wp_total,DEC);
			DTSTREAM.print_P("  Next: ");  DTSTREAM.println(next_wp_index,DEC);
				
		} else if (strmatch(buf+7, PSTR("james"))) {
			DTSTREAM.println_P("James is a monkey");
		} else {
			print_error(ERR("USAGE: status {control|gps|landing|loiter|mixing|navigation|navpitch|navroll|rc|rcinputch|rcin|rcout|rcpwm|rctrim|system|takeoff|telemetry|throttle|waypoints}"));
		}
	} else {
		print_error(ERR("USAGE: {echo <text>|groundstart|inithome|show|hide|print|status|set|reset|rtl}"));
		print_error(ERR("Type <command> -? for specific usage guidance"));
	}
}

/* strmatch compares two strings and returns 1 if they match and 0 if they don't
   s2 must be stored in program memory (via PSTR) rather than RAM (like standard strings)
   s1 must be at least as long as s2 for a valid match
   if s1 is longer than s2, then only the beginning of s1 (the length of s2) must match s2 for a valid match */
int strmatch(char* s1, const char* s2) {
	int i = 0;
	char c1 = s1[0], c2 = pgm_read_byte(s2);

	while (c1 != 0 && c2 != 0) {
		if (c1 < c2)
			return 0;
		if (c1 > c2)
			return 0;
		i++;
		c1 = s1[i];
		c2 = pgm_read_byte(s2+i);
	}

	if (c2==0)
		return 1;
	else
		return 0;
}

/* readfloat parses a string written as a float starting at the offset in *i and updates *i as it parses
   numbers without a decimal place are read as integers
   numbers with a decimial place are multiplied by 10,000,000 and decimals beyond 7 places are discarded
   parsing is terminated with either a space or a null, other characters are ignored */
long readfloat(char* s, unsigned char* i) {
	long result = 0, multiplier = 0;
	char c, neg = 0;
	
	for (;;(*i)++) {
		c = s[*i];
		if (c == ' ')
			break;
		else if (c == 0)
			break;
		else if (c == '-')
			neg = 1-neg;
		else if (c == '.') {
			result *= 10000000;
			multiplier = 1000000;
		} else if (c >= '0' && c <= '9') {
			if (multiplier == 0)
				result = (result*10) + c-'0';
			else {
				result += (c-'0')*multiplier;
				multiplier /= 10;
			}
		}
	}

	if (multiplier == 0)
		result *= 10000000;
	
	if (neg)
		return -result;
	else
		return result;
}

/* process_set_cmd processing the parameters of a 'set cmd' command and takes the appropriate action
   *buffer is the buffer containing the parameters of the command; it should start after the space after 'set cmd'
   bufferlen is the length of the buffer; the routine will stop looking for parameters after the offset index reaches this value
*/
#define SETPARAM_NONE (0)
#define SETPARAM_ID (1)
#define SETPARAM_P1 (2)
#define SETPARAM_LAT (3)
#define SETPARAM_LNG (4)
#define SETPARAM_ALT (5)
#define SETPARAM_P2 (6)
#define SETPARAM_P3 (7)
#define SETPARAM_P4 (8)

void process_set_cmd(char *buffer, int bufferlen) {
	unsigned char i, j, err=1, setparam=SETPARAM_NONE;
	unsigned char cmdindex=0, p1=0, cmdid;
	long lat, lng, alt;
	Location temp;

	//Parse the command index
	for (i=0; i<bufferlen; i++)
		if (buffer[i] >= '0' && buffer[i] <= '9')
			cmdindex = (cmdindex*10) + buffer[i]-'0';
		else
			break;
	
	if (buffer[i] == ' ') {
		//Find the end of the command-type string
		i++;
		for (j=i; j<bufferlen; j++) {
			if (buffer[j] == ' ' || buffer[j] == 0)
				break;
		}
		if (buffer[j] == ' ') {
			//Process the command-type string
			buffer[j] = 0; //Null-terminate the command-type string so strmatch can figure out when to stop comparing, and so atoi can work
			if (strmatch(buffer+i, PSTR("waypoint")))
				cmdid = CMD_WAYPOINT;
			else if (strmatch(buffer+i, PSTR("takeoff")))
				cmdid = CMD_TAKEOFF;
			else if (strmatch(buffer+i, PSTR("landoptions")))
				cmdid = CMD_LAND_OPTIONS;
			else if (strmatch(buffer+i, PSTR("land")))
				cmdid = CMD_LAND;
			else if (strmatch(buffer+i, PSTR("loiternturns")))
				cmdid = CMD_LOITER_N_TURNS;
			else if (strmatch(buffer+i, PSTR("loitertime")))
				cmdid = CMD_LOITER_TIME;
			else if (strmatch(buffer+i, PSTR("loiter")))
				cmdid = CMD_LOITER;
			else if (strmatch(buffer+i, PSTR("delay")))
				cmdid = CMD_DELAY;
			else if (strmatch(buffer+i, PSTR("resetindex")))
				cmdid = CMD_RESET_INDEX;
			else if (strmatch(buffer+i, PSTR("throttlecruise")))
				cmdid = CMD_THROTTLE_CRUISE;
			else if (strmatch(buffer+i, PSTR("resethome")))
				cmdid = CMD_RESET_HOME;
			else if (strmatch(buffer+i, PSTR("index")))
				cmdid = CMD_INDEX;
			else if (strmatch(buffer+i, PSTR("rtl")))
				cmdid = CMD_RTL;
			else if (strmatch(buffer+i, PSTR("id")))
				setparam = SETPARAM_ID;
			else if (strmatch(buffer+i, PSTR("p1")))
				setparam = SETPARAM_P1;
			else if (strmatch(buffer+i, PSTR("alt")))
				setparam = SETPARAM_ALT;
			else if (strmatch(buffer+i, PSTR("lat")))
				setparam = SETPARAM_LAT;
			else if (strmatch(buffer+i, PSTR("lng")))
				setparam = SETPARAM_LNG;
			else if (strmatch(buffer+i, PSTR("p2")))
				setparam = SETPARAM_P2;
			else if (strmatch(buffer+i, PSTR("p3")))
				setparam = SETPARAM_P3;
			else if (strmatch(buffer+i, PSTR("p4")))
				setparam = SETPARAM_P4;
			else
				cmdid = atoi(buffer+i);
			
			if (setparam > SETPARAM_NONE) {
				//Process new parameter value
				i = j+1;
				lat = readfloat(buffer, &i);
				temp = get_wp_with_index(cmdindex);
				if (setparam == SETPARAM_ID)
					temp.id = lat/10000000;
				else if (setparam == SETPARAM_P1)
					temp.p1 = lat/10000000;
				else if (setparam == SETPARAM_ALT)
					temp.alt = lat/100000;
				else if (setparam == SETPARAM_LAT)
					temp.lat = lat;
				else if (setparam == SETPARAM_LNG)
					temp.lng = lat;
				else if (setparam == SETPARAM_P2)
					temp.alt = lat/10000000;
				else if (setparam == SETPARAM_P3)
					temp.lat = lat/10000000;
				else if (setparam == SETPARAM_P4)
					temp.lng = lat/10000000;
				cmdid = temp.id;
				p1 = temp.p1;
				lat = temp.lat;
				lng = temp.lng;
				alt = temp.alt;
				err = 0;
			} else {
				//Process param 1
				for (i=j+1; i<bufferlen; i++) {
					if (buffer[i] >= '0' && buffer[i] <= '9')
						p1 = (p1*10) + buffer[i]-'0';
					else
						break;
				}

				if (buffer[i] == ' ') {
					//Process altitude
					i++;
					if (strmatch(buffer+i, PSTR("here"))) {
						lat = GPS.latitude;
						lng = GPS.longitude;
						alt = get_altitude_above_home(); //GPS.altitude;
						err = 0;
					} else {
						alt = readfloat(buffer, &i)/100000;

						if (buffer[i] == ' ') {
							//Process latitude
							i++;
							lat = readfloat(buffer, &i);
							if (strmatch(buffer+i, PSTR("here"))) {
								lat = GPS.latitude;
								lng = GPS.longitude;
								err = 0;
							} else {
								if (buffer[i] == ' ') {
									//Process longitude
									i++;
									lng = readfloat(buffer, &i);
									err = 0;
								} else
									print_error(ERR("Error processing set cmd: Could not find longitude parameter"));
							}
						} else
							print_error(ERR("Error processing set cmd: Could not find latitude parameter"));
					}
				} else
					print_error(ERR("Error processing set cmd: Could not find altitude parameter"));
			}
		} else
			print_error(ERR("Error processing set cmd: Could not find command type"));
	} else
		print_error(ERR("Error processing set cmd: Could not find command index"));
		
	if (err == 0) {
		temp.id = cmdid;
		if (cmdid == CMD_LAND_OPTIONS) {
			temp.p1 = p1;
			temp.alt = alt;
			temp.lat = lat / 10000000;
			temp.lng = lng / 10000000;
		} else {
			temp.p1 = p1;
			temp.lat = lat;
			temp.lng = lng;
			temp.alt = alt;
		}
		if (cmdindex >= wp_total) {
			wp_total = cmdindex+1;
			save_EEPROM_waypoint_info();
		}
		set_wp_with_index(temp, cmdindex);

		DTSTREAM.print_P("Set command ");
		DTSTREAM.print((int)cmdindex);
		DTSTREAM.print_P(" to ");
		DTSTREAM.print((int)temp.id);
		DTSTREAM.print_P(" with p1=");
		DTSTREAM.print((int)temp.p1);
		DTSTREAM.print_P(", lat=");
		DTSTREAM.print(temp.lat);
		DTSTREAM.print_P(", lng=");
		DTSTREAM.print(temp.lng);
		DTSTREAM.print_P(", alt=");
		DTSTREAM.println(temp.alt);
	}
}

/* get_pid_offset_name matches a string expressed in *buffer with a pid keyword and returns the k-array
   gain offset in *offset, and the length of that string expression in *length.  If a good keyword
   match is found, 1 is returned; 0 otherwise
*/
int get_pid_offset_name(char *buffer, int *offset, unsigned char *length) {
	if (strmatch(buffer, PSTR("servoroll"))) {
		*length += 9;  *offset = CASE_SERVO_ROLL;
	} else if (strmatch(buffer, PSTR("servopitch"))) {
		*length += 10;  *offset = CASE_SERVO_PITCH;
	} else if (strmatch(buffer, PSTR("servorudder"))) {
		*length += 11;  *offset = CASE_SERVO_RUDDER;
	} else if (strmatch(buffer, PSTR("navroll"))) {
		*length += 7;  *offset = CASE_NAV_ROLL;
	} else if (strmatch(buffer, PSTR("navpitchasp"))) {
		*length += 11;  *offset = CASE_NAV_PITCH_ASP;
	} else if (strmatch(buffer, PSTR("navpitchalt"))) {
		*length += 11;  *offset = CASE_NAV_PITCH_ALT;
	} else if (strmatch(buffer, PSTR("throttlete"))) {
		*length += 10;  *offset = CASE_TE_THROTTLE;
	} else if (strmatch(buffer, PSTR("throttlealt"))) {
		*length += 11;  *offset = CASE_ALT_THROTTLE;
	} else {
		return 0;
	}
	
	return 1;
}

/* print_rlocation prints the relative location of the specified waypoint from the plane in easy-to-read cartesian format
*/
void print_rlocation(Location *wp) {
	float x, y;
	y = (float)(wp->lat - current_loc.lat) * 0.0111194927;
	x = (float)(wp->lng - current_loc.lng) * cos(radians(current_loc.lat)) * 0.0111194927;
	DTSTREAM.print_P("dP = <");
	DTSTREAM.print(abs((int)y),DEC);
	if (y >= 0) DTSTREAM.print_P("N, "); else DTSTREAM.print_P("S, ");
	DTSTREAM.print(abs((int)x),DEC);
	if (x >= 0) DTSTREAM.print_P("E, "); else DTSTREAM.print_P("W, ");
	DTSTREAM.print((float)(wp->alt - current_loc.alt)/100,1);
	DTSTREAM.println_P("> meters");
}

/* print_error prints an error message if the user sends an invalid command
*/
void print_error(const char *msg) {
	if (msg == 0)
		DTSTREAM.println_P(PSTR("ERROR: Invalid command"));
	else
		DTSTREAM.println_P(msg);
}

void send_message(byte severity, const char *str)		// This is the instance of send_message for message 0x05
{
	if(severity >= DEBUG_LEVEL){
		DTSTREAM.print_P("MSG: ");
		DTSTREAM.println(str);
	}
}

void send_message(byte id) {
	send_message(id,(long)0);
}

void send_message(byte id, long param) {

	switch(id) {
		case MSG_HEARTBEAT:
			if (report_heartbeat)
				print_control_mode();
			break;
		
		case MSG_ATTITUDE:
			if (report_attitude)
				print_attitude();
			break;
			
		case MSG_LOCATION:
			if (report_location)
				print_position();
			if (first_location == 0) {
				send_message(0,"First location received");
				first_location = 1;
			}
			break;
		
		case MSG_COMMAND:
			struct Location cmd = get_wp_with_index(param);
			print_waypoint(&cmd, param);
			break;
		
        }
}
			
void pipe()
{
	DTSTREAM.print_P("|");
}

void print_current_waypoints()
{
	DTSTREAM.println_P("Current waypoints:");
	DTSTREAM.print_P("  Prev:");
	DTSTREAM.print_P("\t");
	DTSTREAM.print(prev_WP.lat,DEC);
	DTSTREAM.print_P(",\t");
	DTSTREAM.print(prev_WP.lng,DEC);
	DTSTREAM.print_P(",\t");
	DTSTREAM.println(prev_WP.alt,DEC);

	DTSTREAM.print_P("  Cur: ");
	DTSTREAM.print_P("\t");
	DTSTREAM.print(current_loc.lat,DEC);					
	DTSTREAM.print_P(",\t");
	DTSTREAM.print(current_loc.lng,DEC);					
	DTSTREAM.print_P(",\t");
	DTSTREAM.println(current_loc.alt,DEC);					
	
	DTSTREAM.print_P("  Next:");
	DTSTREAM.print(wp_index,DEC);
	DTSTREAM.print_P(",\t");
	DTSTREAM.print(next_WP.lat,DEC);
	DTSTREAM.print_P(",\t");
	DTSTREAM.print(next_WP.lng,DEC);
	DTSTREAM.print_P(",\t");
	DTSTREAM.println(next_WP.alt,DEC);
}

void print_position(void)
{
	DTSTREAM.print_P("Pos: ");
	DTSTREAM.print(current_loc.lat,DEC);					// 0
	DTSTREAM.print_P(", "); //°
	DTSTREAM.print(current_loc.lng,DEC);					// 1
	DTSTREAM.print_P(", ");
	DTSTREAM.print(current_loc.alt,DEC);					// 2
	DTSTREAM.print_P("cm, ");
	DTSTREAM.print(GPS.ground_speed,DEC);						// 3
	DTSTREAM.print_P("cm/s GS, ");
	DTSTREAM.print(airspeed,DEC);					// 4
	DTSTREAM.print_P("cm/s AS, ");
	DTSTREAM.print(get_altitude_above_home(),DEC);		// 5
	DTSTREAM.print_P("cm above home, ");
	DTSTREAM.print(climb_rate,DEC);						// 6
	DTSTREAM.print_P("? climb, ");
	DTSTREAM.print(wp_distance,DEC);						// 7
	DTSTREAM.print_P("m from wp, ");
	DTSTREAM.print(throttle_cruise,DEC);					// 8
	DTSTREAM.print_P("% throttle, ");
	DTSTREAM.print(altitude_error,DEC);					// 9
	DTSTREAM.println_P("alt err");
}

void print_attitude(void)
{
	DTSTREAM.print_P("Att: ");
	DTSTREAM.print(radio_in[CH_ROLL],DEC);					// 0
	DTSTREAM.print_P(" roll in, ");
	DTSTREAM.print(radio_in[CH_PITCH],DEC);					// 1
	DTSTREAM.print_P(" pitch in, ");
	DTSTREAM.print(radio_in[CH_THROTTLE],DEC);		// 2
	DTSTREAM.print_P(" throttle in, ");
	DTSTREAM.print(roll_sensor,DEC);					// 3
	DTSTREAM.print_P(" roll sensor, ");
	DTSTREAM.print(pitch_sensor,DEC);					// 4
	DTSTREAM.print_P(" pitch sensor, ");
	DTSTREAM.print(GPS.ground_course,DEC);				// 6
	DTSTREAM.print_P(" ground course, ");
	DTSTREAM.print(target_bearing,DEC);				// 7
	DTSTREAM.print_P(" target bearing, ");
	DTSTREAM.print(nav_roll,DEC);					// 8
	DTSTREAM.print_P(" nav roll, ");
	DTSTREAM.print(loiter_sum,DEC);					// 8
	DTSTREAM.print_P(" loiter sum, ");
	DTSTREAM.print(servo_out[CH_ROLL],DEC);
	DTSTREAM.print_P(" roll servo_out, ");
	DTSTREAM.print(servo_out[CH_PITCH],DEC);
	DTSTREAM.println_P(" pitch servo_out");
}

// required by Groundstation to plot lateral tracking course 
void print_new_wp_info()
{
	DTSTREAM.print_P("New waypt (");
	DTSTREAM.print(wp_index,DEC);	//0
	DTSTREAM.print_P("): ");
	DTSTREAM.print(prev_WP.lat,DEC);		//1
	DTSTREAM.print_P(", "); //°
	DTSTREAM.print(prev_WP.lng,DEC);		//2
	DTSTREAM.print_P(", ");
	DTSTREAM.print(prev_WP.alt,DEC);		//3
	DTSTREAM.print_P("m -> ");
	DTSTREAM.print(next_WP.lat,DEC);		//4
	DTSTREAM.print_P("°, ");
	DTSTREAM.print(next_WP.lng,DEC);		//5
	DTSTREAM.print_P("°, ");
	DTSTREAM.print(next_WP.alt,DEC);		//6
	DTSTREAM.print_P("m; ");
	DTSTREAM.print(wp_totalDistance,DEC);	//7
	DTSTREAM.print_P("m dist, ");
	DTSTREAM.print(radio_trim[CH_ROLL],DEC);	//8
	DTSTREAM.print_P(" roll trim, ");
	DTSTREAM.print(radio_trim[CH_PITCH],DEC);	//9
	DTSTREAM.println_P(" pitch trim");
}

void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			DTSTREAM.println_P("##MANUAL");
			break;
		case STABILIZE:
			DTSTREAM.println_P("##STABILIZE");
			break;
		case FLY_BY_WIRE_A:
			DTSTREAM.println_P("##FBW A");
			break;
		case FLY_BY_WIRE_B:
			DTSTREAM.println_P("##FBW B");
			break;
		case AUTO:
			DTSTREAM.println_P("##AUTO");
			break;
		case RTL:
			DTSTREAM.println_P("##RTL");
			break;
		case LOITER:
			DTSTREAM.println_P("##LOITER");
			break;
		case 98:
			DTSTREAM.println_P("##Air Start Complete");
			break;
		case 99:
			DTSTREAM.println_P("##Ground Start Complete");
			break;
	}
}

void print_tuning(void) {
	DTSTREAM.print_P("TUN:");
	DTSTREAM.print(servo_out[CH_ROLL]);
	DTSTREAM.print_P(",	 ");
	DTSTREAM.print(nav_roll/100,DEC);
	DTSTREAM.print_P(",	 ");
	DTSTREAM.print(roll_sensor/100,DEC);
	DTSTREAM.print_P(",	 ");
	DTSTREAM.print(servo_out[CH_PITCH]);
	DTSTREAM.print_P(",	 ");
	DTSTREAM.print(nav_pitch/100,DEC);
	DTSTREAM.print_P(",	 ");
	DTSTREAM.println(pitch_sensor/100,DEC);
}

void printPerfData(void)
{
}


void print_waypoint(struct Location *cmd,byte index)
{
	//TODO: Why does this stop printing in the middle?? --BJP
	DTSTREAM.print_P("    command #: ");
	DTSTREAM.print(index, DEC);
	DTSTREAM.print_P(" id: ");
	switch (cmd->id) {
		// Command IDs - Must
		case CMD_BLANK: DTSTREAM.print_P("CMD_BLANK");  break;
		case CMD_WAYPOINT: DTSTREAM.print_P("CMD_WAYPOINT");  break;
		case CMD_LOITER: DTSTREAM.print_P("CMD_LOITER");  break;
		case CMD_LOITER_N_TURNS: DTSTREAM.print_P("CMD_LOITER_N_TURNS");  break;
		case CMD_LOITER_TIME: DTSTREAM.print_P("CMD_LOITER_TIME");  break;
		case CMD_RTL: DTSTREAM.print_P("CMD_RTL");  break;
		case CMD_LAND: DTSTREAM.print_P("CMD_LAND");  break;
		case CMD_TAKEOFF: DTSTREAM.print_P("CMD_TAKEOFF");  break;

		// Command IDs - May
		case CMD_DELAY: DTSTREAM.print_P("CMD_DELAY");  break;
		case CMD_CLIMB: DTSTREAM.print_P("CMD_CLIMB");  break;
		case CMD_LAND_OPTIONS: DTSTREAM.print_P("CMD_LAND_OPTIONS");  break;

		// Command IDs - Now
		case CMD_RESET_INDEX: DTSTREAM.print_P("CMD_RESET_INDEX");  break;
		case CMD_GOTO_INDEX: DTSTREAM.print_P("CMD_GOTO_INDEX");  break;
		case CMD_GETVAR_INDEX: DTSTREAM.print_P("CMD_GETVAR_INDEX");  break;
		case CMD_SENDVAR_INDEX: DTSTREAM.print_P("CMD_SENDVAR_INDEX");  break;
		case CMD_TELEMETRY: DTSTREAM.print_P("CMD_TELEMETRY");  break;

		case CMD_THROTTLE_CRUISE: DTSTREAM.print_P("CMD_THROTTLE_CRUISE");  break;
		case CMD_AIRSPEED_CRUISE: DTSTREAM.print_P("CMD_AIRSPEED_CRUISE");  break;
		case CMD_RESET_HOME: DTSTREAM.print_P("CMD_RESET_HOME");  break;

		case CMD_KP_GAIN: DTSTREAM.print_P("CMD_KP_GAIN");  break;
		case CMD_KI_GAIN: DTSTREAM.print_P("CMD_KI_GAIN");  break;
		case CMD_KD_GAIN: DTSTREAM.print_P("CMD_KD_GAIN");  break;
		case CMD_KI_MAX: DTSTREAM.print_P("CMD_KI_MAX");  break;
		case CMD_KFF_GAIN: DTSTREAM.print_P("CMD_KFF_GAIN");  break;

		case CMD_RADIO_TRIM: DTSTREAM.print_P("CMD_RADIO_TRIM");  break;
		case CMD_RADIO_MAX: DTSTREAM.print_P("CMD_RADIO_MAX");  break;
		case CMD_RADIO_MIN: DTSTREAM.print_P("CMD_RADIO_MIN");  break;
		case CMD_ELEVON_TRIM: DTSTREAM.print_P("CMD_ELEVON_TRIM");  break;

		case CMD_INDEX: DTSTREAM.print_P("CMD_INDEX");  break;
		case CMD_REPEAT: DTSTREAM.print_P("CMD_REPEAT");  break;
		case CMD_RELAY: DTSTREAM.print_P("CMD_RELAY");  break;
		case CMD_SERVO: DTSTREAM.print_P("CMD_SERVO");  break;
		
		default: DTSTREAM.print(cmd->id,DEC);
	}
	DTSTREAM.print_P(" p1: ");
	DTSTREAM.print(cmd->p1,DEC);
	DTSTREAM.print_P(" p2/alt: ");
	DTSTREAM.print(cmd->alt,DEC);
	DTSTREAM.print_P(" p3/lat: ");
	DTSTREAM.print(cmd->lat,DEC);
	DTSTREAM.print_P(" p4/lng: ");
	DTSTREAM.println(cmd->lng,DEC);
}

void print_waypoints()
{
	DTSTREAM.println_P("Commands in memory:");
	DTSTREAM.print_P("  ");
	DTSTREAM.print(wp_total, DEC);
	DTSTREAM.println_P(" commands total");
	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	DTSTREAM.println_P("  Home: ");
	struct Location cmd = get_wp_with_index(0);
	print_waypoint(&cmd, 0);
	DTSTREAM.println_P("  Commands: ");
	
	for (int i=1; i<wp_total; i++){
		cmd = get_wp_with_index(i);
		print_waypoint(&cmd, i);
		delay(10);
	}
}

#endif

