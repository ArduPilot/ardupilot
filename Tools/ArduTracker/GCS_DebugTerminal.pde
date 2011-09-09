// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/* This GCS protocol sends text-based information over the GCS port
*/

#if GCS_PROTOCOL == GCS_PROTOCOL_DEBUGTERMINAL

#define ERR(a) ((DEBUGTERMINAL_VERBOSE)>0?(PSTR(a)):(0))

void 
GCS_DEBUGTERMINAL::update() 
{
	byte numc, i, c;

	numc = _port->available();
	for (i=0;i<numc;i++) {
		c = _port->read();   
		processgcsinput(c);
	}
}

void 
GCS_DEBUGTERMINAL::processgcsinput(char c) 
{
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
		if (bufferidx >= sizeof(gcsinputbuffer)) bufferidx = 0;
	}
}
		
void 
GCS_DEBUGTERMINAL::run_debugt_command(char *buf) 
{
	BetterStream *port = _port;

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
		else if (strmatch(buf+5, PSTR("cpuload")))
			report_cpu_load = 1;
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
		else if (strmatch(buf+5, PSTR("cpuload")))
			report_cpu_load = 0;
		else if (strmatch(buf+5, PSTR("all"))) {
			report_heartbeat = 0;
			report_attitude = 0;
			report_location = 0;
			report_command = 0;
			report_cpu_load = 0;
		} else
			print_error(ERR("USAGE: hide {heartbeat|attitude|location|command|all}"));

	//*********** Process 'copy' command ***********
	} else if (strmatch(buf, PSTR("copy "))) {
		//------- copy cmd <N1> <N2> -------
		if (strmatch(buf+5, PSTR("cmd "))) {
			unsigned char i = 9, index1, index2;
			while (buf[i] != 0) {
				i++;
				if (buf[i] == ' ') break;
			}
			if (buf[i] == ' ') {
				buf[i] = 0;
				index1 = atoi(buf+9);
				index2 = atoi(buf+i+1);
				Location temp = get_wp_with_index(index1);
				set_wp_with_index(temp, index2);
				port->print_P(PSTR("Copied command index "));  port->print(index1,DEC);  port->print_P(PSTR(" to "));  port->println(index2,DEC);
			} else {
				print_error(ERR("USAGE: copy cmd <srcindex> <targetindex>"));
			}
		}

	//*********** Process 'echo' command ***********
	} else if (strmatch(buf, PSTR("echo "))) {
		port->println(buf+5);
			
	//*********** Process 'groundstart' command ***********
	} else if (strmatch(buf, PSTR("groundstart"))) {
		startup_ground();
				
	//*********** Process 'inithome' command ***********
	} else if (strmatch(buf, PSTR("inithome"))) {
		init_home();
		port->println_P(PSTR("Home set."));
  				
	//------- k -? -------
	} else if (strmatch(buf, PSTR("k -?"))) {
		print_error(ERR("USAGE: {print|set} {k{p|i|d}|imax} {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt}"));
		print_error(ERR("USAGE: {print|set} kff {pitchcomp|ruddermix|pitchtothrottle}"));

	//*********** Process 'print' commands ***********
	} else if (strmatch(buf, PSTR("print "))) {
		//------- print airspeedtrim -------
		if (strmatch(buf+6, PSTR("airspeedtrim"))) {
			port->printf_P("Trim airspeed = %f\n", (float)get(PARAM_TRIM_AIRSPEED)/100);
			
		//------- print airspeednudge -------
		} else if (strmatch(buf+6, PSTR("airspeednudge"))) {
			port->printf_P("Airspeed nudge = %f\n", (float)airspeed_nudge/100);
		
		//------- print altitude -------
		} else if (strmatch(buf+6, PSTR("altitude"))) {
			recalc_climb_rate();
			port->printf_P(PSTR("Altitude:\n"
								"  Pressure: %.2fm\n"
								"  GPS: %.2fm\n"
								"  Mix ratio: %.3f\n"
								"  Mix: %.2fm\n"
								"  Above home: %.1fm\n"
								"  Climb rate: %.2fm/s\n"),
							(float)press_alt / 100,
							(float)gps.altitude / 100,
							get(PARAM_ALT_MIX),
							(((1.0 - get(PARAM_ALT_MIX)) * gps.altitude) + (get(PARAM_ALT_MIX) * press_alt)) / 100,
							(float)get_altitude_above_home()/100,
							(float)climb_rate/100);

		//------- print attitude -------
		} else if (strmatch(buf+6, PSTR("attitude"))) {
			print_attitude();

		//------- print commands[ <N1>-<N2>] -------
		} else if (strmatch(buf+6, PSTR("commands"))) {
			unsigned char dash, index1, index2;
			for (dash=14; dash<sizeof(gcsinputbuffer); dash++) {
				if (buf[dash] == 0) break;
				if (buf[dash] == '-') break;
			}
			if (buf[dash] == 0) {
				print_commands();
			} else {
				buf[dash] = 0;
				index1 = atoi(buf+14);
				index2 = atoi(buf+dash+1);
				if (index2 < index1) index2 = get(PARAM_WP_TOTAL);
				print_commands(index1, index2);
			}

		//------- print ctrlmode -------
		} else if (strmatch(buf+6, PSTR("ctrlmode"))) {
			print_control_mode();

		//------- print curwaypts -------
		} else if (strmatch(buf+6, PSTR("curwaypts"))) {
			print_current_waypoints();

		//------- print gains -------
		} else if (strmatch(buf+6, PSTR("gains"))) {
			print_gains();

		//------- print flightmodes -------
		} else if (strmatch(buf+6, PSTR("flightmodes"))) {
			int i;
			port->print_P(PSTR("EEPROM read: "));
			for (i=0; i<6; i++) {
				port->print_P(PSTR("Ch "));  port->print(i,DEC);  port->print_P(PSTR(" = "));  port->print(get(uint8_param_t(PARAM_FLIGHT_MODE_1+i)),DEC);  port->print_P(PSTR(", "));
			}
			port->println(" ");

		//------- print holdalt -------
		} else if (strmatch(buf+6, PSTR("holdalt"))) {
			port->print_P(PSTR("Altitude above home set to "));  port->println(get(PARAM_ALT_HOLD_HOME),2);

		//------- print imax {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} -------
		} else if (strmatch(buf+6, PSTR("imax "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+11, &i, &j)) {
				port->print_P(PSTR("Integrator maximum for "));
				port->print(buf+9);
				port->print_P(PSTR(" = "));
				port->println(pid_index[i]->imax(),DEC);
			} else
				print_gain_keyword_error();

		//------- print kp {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} -------
		} else if (strmatch(buf+6, PSTR("kp "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				port->print_P(PSTR("P gain for "));
				port->print(buf+9);
				port->print_P(PSTR(" = "));
				port->println(pid_index[i]->kP(),DEC);
			} else
				print_gain_keyword_error();
				
		//------- print ki {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} -------
		} else if (strmatch(buf+6, PSTR("ki "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				port->print_P(PSTR("I gain for "));
				port->print(buf+9);
				port->print_P(PSTR(" = "));
				port->println(pid_index[i]->kI(),DEC);
			} else
				print_gain_keyword_error();
				
		//------- print kd {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} -------
		} else if (strmatch(buf+6, PSTR("kd "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				port->print_P(PSTR("D gain for "));
				port->print(buf+9);
				port->print_P(PSTR(" = "));
				port->println(pid_index[i]->kD(),DEC);
			} else
				print_gain_keyword_error();
				
		//------- print kff {pitchcomp|ruddermix|pitchtothrottle} -------
		} else if (strmatch(buf+6, PSTR("kff "))) {
			if (strmatch(buf+10, PSTR("pitchcomp"))) {
				port->print_P(PSTR("FF gain for pitchcomp = "));
				port->println(get(PARAM_KFF_PTCHCOMP),DEC);
			} else if (strmatch(buf+10, PSTR("ruddermix"))) {
				port->print_P(PSTR("FF gain for ruddermix = "));
				port->println(get(PARAM_KFF_RDDRMIX),DEC);
			} else if (strmatch(buf+10, PSTR("pitchtothrottle"))) {
				port->print_P(PSTR("FF gain for pitchtothrottle = "));
				port->println(get(PARAM_KFF_PTCH2THR),DEC);
			/*} else if (strmatch(buf+10, PSTR("throttletopitch"))) {
				port->print_P(PSTR("FF gain for throttletopitch = "));
				port->println(kff[CASE_T_TO_P],DEC);*/
			} else
				print_gain_keyword_error();

		//------- print location -------
		} else if (strmatch(buf+6, PSTR("location"))) {
			print_position();

		//------- print navrolllimit -------
		} else if (strmatch(buf+6, PSTR("navrolllimit"))) {
			port->print_P(PSTR("Nav roll limit = "));  port->println((float)get(PARAM_LIM_ROLL)/100,2);

		//------- print navsettings -------
		} else if (strmatch(buf+6, PSTR("navsettings"))) {
			port->printf_P(PSTR("Navigation settings:\n"
#if AIRSPEED_SENSOR == ENABLED
								"  Cruise airspeed: %.2f\n"
#else
								"  Cruise throttle: %d\n"
#endif
								"  Hold altitude above home: %ld\n"
								"  Loiter radius: %d\n"
								"  Waypoint radius: %d\n"),
#if AIRSPEED_SENSOR == ENABLED
						   (float)get(PARAM_TRIM_AIRSPEED) / 100,
#else
						   get(PARAM_TRIM_THROTTLE),
#endif
						   get(PARAM_ALT_HOLD_HOME),
						   get(PARAM_LOITER_RADIUS),
						   get(PARAM_WP_RADIUS));

		//------- print pitchmax -------
		} else if (strmatch(buf+6, PSTR("pitchmax"))) {
			port->print_P(PSTR("Maximum pitch = "));  port->println((float)get(PARAM_LIM_PITCH_MAX)/100,2);

		//------- print pitchmin -------
		} else if (strmatch(buf+6, PSTR("pitchmin"))) {
			port->print_P(PSTR("Minimum pitch = "));  port->println((float)get(PARAM_LIM_PITCH_MIN)/100,2);

		//------- print pitchtarget -------
		} else if (strmatch(buf+6, PSTR("pitchtarget"))) {
			port->print_P(PSTR("Target pitch = "));  port->println((float)get(PARAM_TRIM_PITCH)/100,2);

#if HIL_MODE != HIL_MODE_ATTITUDE
		//------- print pressure -------
		} else if (strmatch(buf+6, PSTR("pressure"))) {
			port->println_P(PSTR("BMP085 pressure sensor:"));
			port->print_P(PSTR("  Temperature: "));  port->println(pitot.Temp,DEC);
			port->print_P(PSTR("  Pressure: "));  port->println(pitot.Press,DEC);
#endif

		//------- print rlocation home -------
		} else if (strmatch(buf+6, PSTR("rlocation home"))) {
			print_rlocation(&home);

		//------- print rlocation -------
		//(implication is "relative to next waypoint")
		} else if (strmatch(buf+6, PSTR("rlocation"))) {
			print_rlocation(&next_WP);

		//------- print speed -------
		} else if (strmatch(buf+6, PSTR("speed"))) {
			port->println_P(PSTR("Speed:"));
			port->print_P(PSTR("  Ground: "));  port->println((float)gps.ground_speed/100.0,2);
#if AIRSPEED_SENSOR == ENABLED
				port->print_P(PSTR("  Air: "));  port->println((float)airspeed/100.0,2);
				port->print_P(PSTR("  Cruise: "));  port->println((float)get(PARAM_TRIM_AIRSPEED)/100.0,2);
#endif

		//------- print throttlecruise -------
		} else if (strmatch(buf+6, PSTR("throttlecruise"))) {
			port->print_P(PSTR("Throttle cruise = "));  port->print(get(PARAM_TRIM_THROTTLE),DEC);  port->println_P(PSTR("%"));

		//------- print throttlemax -------
		} else if (strmatch(buf+6, PSTR("throttlemax"))) {
			port->print_P(PSTR("Throttle max = "));  port->print(get(PARAM_THR_MAX),DEC);  port->println_P(PSTR("%"));

		//------- print throttlemin -------
		} else if (strmatch(buf+6, PSTR("throttlemin"))) {
			port->print_P(PSTR("Throttle min = "));  port->print(get(PARAM_THR_MIN),DEC);  port->println_P(PSTR("%"));

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
			port->println_P(PSTR("Commands reloaded."));
		} else
			print_error(ERR("USAGE: reset commands"));

	//*********** Process 'rtl' command ***********
	} else if (strmatch(buf, PSTR("rtl"))) {
		return_to_launch();
		port->println_P(PSTR("Returning to launch..."));

	//*********** Process 'set' commands ***********
	} else if (strmatch(buf, PSTR("set "))) {
		//------- set cmd -------
		if (strmatch(buf+4, PSTR("cmd "))) {
			process_set_cmd(buf+8, bufferidx-8);

		//------- set cmdcount -------
		} else if (strmatch(buf+4, PSTR("cmdcount "))) {
			set(PARAM_WP_TOTAL, atoi(buf+13));
			port->print_P(PSTR("PARAM_WP_TOTAL = "));  port->println(get(PARAM_WP_TOTAL),DEC);

		//------- set cmdindex -------
		} else if (strmatch(buf+4, PSTR("cmdindex "))) {
			set(PARAM_WP_INDEX, atoi(buf+13));
			decrement_WP_index();
			next_command = get_wp_with_index(get(PARAM_WP_INDEX)+1);
			port->print_P(PSTR("Command set to index "));  port->print(get(PARAM_WP_INDEX),DEC);
			if (next_command.id >= 0x10 && next_command.id <= 0x1F) { //TODO: create a function the defines what type of command each command ID is
				command_must_index = 0;
				port->println_P(PSTR(" (must)"));
			} else if (next_command.id >= 0x20 && next_command.id <= 0x2F) {
				command_may_index = 0;
				port->println_P(PSTR(" (may)"));
			} else
				port->println_P(PSTR(" (now)"));

			next_command.id = CMD_BLANK;
			if (get(PARAM_WP_INDEX) > get(PARAM_WP_TOTAL)) {
				set(PARAM_WP_TOTAL, get(PARAM_WP_INDEX));
				port->print_P(PSTR("  The total number of commands is now "));
				port->println(get(PARAM_WP_TOTAL),DEC);
			}
			next_WP = current_loc;
			update_commands();

		//------- set cruise -------
		} else if (strmatch(buf+4, PSTR("cruise "))) {
			unsigned char j = 4+7;
#if AIRSPEED_SENSOR == 1
			port->print_P(PSTR("airspeed_cruise changed from "));
			port->print((float)get(PARAM_TRIM_AIRSPEED)/100,2);
			port->print_P(PSTR(" to "));
			set(PARAM_TRIM_AIRSPEED, (int)(readfloat(buf, &j)/100000));
			set(PARAM_TRIM_AIRSPEED, constrain(get(PARAM_TRIM_AIRSPEED),0,9000)); //TODO: constrain minimum as stall speed, maximum as Vne
			port->println(((float)get(PARAM_TRIM_AIRSPEED))/100,2);
#else
			port->print_P(PSTR("throttle_cruise changed from "));
			port->print(get(PARAM_TRIM_THROTTLE),DEC);
			port->print_P(PSTR(" to "));
			set(PARAM_TRIM_THROTTLE, constrain((int)(readfloat(buf, &j)/10000000),0,200));
			port->println(get(PARAM_TRIM_THROTTLE),DEC);
#endif
			//save_user_configs();

		//------- set holdalt -------
		} else if (strmatch(buf+4, PSTR("holdalt "))) {
			int tempalt = atoi(buf+12)*100;
			set(PARAM_ALT_HOLD_HOME, (float)tempalt); //save_alt_to_hold((int32_t)tempalt);
			port->println_P(PSTR("Hold altitude above home set."));
		
		//------- set imax {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} <X> -------
		} else if (strmatch(buf+4, PSTR("imax "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+9, &i, &j)) {
				buf[9+j] = 0;
				port->print_P(PSTR("Integrator maximum for "));
				port->print(buf+9);
				port->print_P(PSTR(" changed from "));
				port->print(pid_index[i]->imax(),DEC);
				port->print_P(PSTR(" to "));
				pid_index[i]->imax((float)atoi(buf+j));
				pid_index[i]->save_gains();
				port->println(pid_index[i]->imax(),DEC);
			} else
				print_error(ERR("ERROR: Did not recognize keyword; type set k -? for more information"));

		//------- set kp {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} <X> -------
		} else if (strmatch(buf+4, PSTR("kp "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+7, &i, &j)) {
				buf[7+j] = 0;
				port->print_P(PSTR("P gain for "));
				port->print(buf+7);
				port->print_P(PSTR(" changed from "));
				port->print(pid_index[i]->kP(),DEC);
				port->print_P(PSTR(" to "));
				j += 7+1;
				pid_index[i]->kP(((float)readfloat(buf, &j))/10000000);
				pid_index[i]->save_gains();
				port->println(pid_index[i]->kP(),DEC);
			} else
				print_gain_keyword_error();
				
		//------- set ki {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} <X> -------
		} else if (strmatch(buf+4, PSTR("ki "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+7, &i, &j)) {
				buf[7+j] = 0;
				port->print_P(PSTR("I gain for "));
				port->print(buf+7);
				port->print_P(PSTR(" changed from "));
				port->print(pid_index[i]->kI(),DEC);
				port->print_P(PSTR(" to "));
				j += 7+1;
				pid_index[i]->kI(((float)readfloat(buf, &j))/10000000);
				pid_index[i]->save_gains();
				port->println(pid_index[i]->kI(),DEC);
			} else
				print_gain_keyword_error();
				
		//------- set kd {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} <X> -------
		} else if (strmatch(buf+4, PSTR("kd "))) {
			int i;
			unsigned char j = 0;
			if (get_pid_offset_name(buf+7, &i, &j)) {
				buf[7+j] = 0;
				port->print_P(PSTR("D gain for "));
				port->print(buf+7);
				port->print_P(PSTR(" changed from "));
				port->print(pid_index[i]->kD(),DEC);
				port->print_P(PSTR(" to "));
				j += 7+1;
				pid_index[i]->kD(((float)readfloat(buf, &j))/10000000);
				pid_index[i]->save_gains();
				port->println(pid_index[i]->kD(),DEC);
			} else
				print_gain_keyword_error();

		//------- set kff {pitchcomp|ruddermix|pitchtothrottle} <X> -------
		} else if (strmatch(buf+4, PSTR("kff "))) {
			unsigned char j = 0;
			if (strmatch(buf+8, PSTR("pitchcomp"))) {
				buf[8+9] = 0;
				port->print_P(PSTR("FF gain for "));
				port->print(buf+8);
				port->print_P(PSTR(" changed from "));
				port->print(get(PARAM_KFF_PTCHCOMP),DEC);
				port->print_P(PSTR(" to "));
				j = 8+9+1;
				set(PARAM_KFF_PTCHCOMP, ((float)readfloat(buf, &j))/10000000);
				port->println(get(PARAM_KFF_PTCHCOMP),DEC);
			} else if (strmatch(buf+8, PSTR("ruddermix"))) {
				buf[8+9] = 0;
				port->print_P(PSTR("FF gain for "));
				port->print(buf+8);
				port->print_P(PSTR(" changed from "));
				port->print(get(PARAM_KFF_RDDRMIX),DEC);
				port->print_P(PSTR(" to "));
				j = 8+9+1;
				set(PARAM_KFF_RDDRMIX, ((float)readfloat(buf, &j))/10000000);
				port->println(get(PARAM_KFF_RDDRMIX),DEC);
			} else if (strmatch(buf+8, PSTR("pitchtothrottle"))) {
				buf[8+15] = 0;
				port->print_P(PSTR("FF gain for "));
				port->print(buf+8);
				port->print_P(PSTR(" changed from "));
				port->print(get(PARAM_KFF_PTCH2THR),DEC);
				port->print_P(PSTR(" to "));
				j = 8+15+1;
				set(PARAM_KFF_PTCH2THR, ((float)readfloat(buf, &j))/10000000);
				port->println(get(PARAM_KFF_PTCH2THR),DEC);
			/*} else if (strmatch(buf+8, PSTR("throttletopitch"))) {
				buf[8+15] = 0;
				port->print_P(PSTR("FF gain for "));
				port->print(buf+8);
				port->print_P(PSTR(" changed from "));
				port->print(kff[CASE_T_TO_P],DEC);
				port->print_P(PSTR(" to "));
				j = 8+15+1;
				kff[CASE_T_TO_P] = ((float)readfloat(buf, &j))/10000000;
				port->println(kff[CASE_T_TO_P],DEC);*/
			} else
				print_gain_keyword_error();

		//------- set loiterradius -------
		} else if (strmatch(buf+4, PSTR("loiterradius "))) {
			set(PARAM_LOITER_RADIUS, atoi(buf+17));
			port->print_P(PSTR("Set loiter radius to "));  port->print(get(PARAM_LOITER_RADIUS),DEC);  port->println_P(PSTR(" meters"));

		//------- set navrolllimit <X> -------
		} else if (strmatch(buf+4, PSTR("navrolllimit "))) {
			unsigned char j = 17;
			port->print_P(PSTR("Nav roll limit changed from "));  port->print((float)get(PARAM_LIM_ROLL)/100,2);
			port->print_P(PSTR(" to "));
			set(PARAM_LIM_ROLL, readfloat(buf, &j)/100000);
			port->println((float)get(PARAM_LIM_ROLL)/100,2);

		//------- set pitchmax <X> -------
		} else if (strmatch(buf+4, PSTR("pitchmax "))) {
			unsigned char j = 13;
			port->print_P(PSTR("Maximum pitch changed from "));  port->print((float)get(PARAM_LIM_PITCH_MAX)/100,2);
			port->print_P(PSTR(" to "));
			set(PARAM_LIM_PITCH_MAX, readfloat(buf, &j)/100000);
			port->println((float)get(PARAM_LIM_PITCH_MAX)/100,2);

		//------- set pitchmin <X> -------
		} else if (strmatch(buf+4, PSTR("pitchmin "))) {
			unsigned char j = 13;
			port->print_P(PSTR("Minimum pitch changed from "));  port->print((float)get(PARAM_LIM_PITCH_MIN)/100,2);
			port->print_P(PSTR(" to "));
			set(PARAM_LIM_PITCH_MIN, readfloat(buf, &j)/100000);
			port->println((float)get(PARAM_LIM_PITCH_MIN)/100,2);

		//------- set pitchtarget <X> -------
		} else if (strmatch(buf+4, PSTR("pitchtarget "))) {
			unsigned char j = 16;
			port->print_P(PSTR("Pitch target changed from "));  port->print((float)get(PARAM_TRIM_PITCH)/100,2);
			port->print_P(PSTR(" to "));
			set(PARAM_TRIM_PITCH, readfloat(buf, &j)/100000);
			port->println((float)get(PARAM_TRIM_PITCH)/100,2);

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

		//------- set relay -------
		} else if (strmatch(buf+4, PSTR("relay "))) {
			unsigned char newvalue = atoi(buf+10);
			if (newvalue == 0) {
				relay_off();
				port->println_P(PSTR("Relay turned off"));
			} else if (newvalue == 1) {
				relay_on();
				port->println_P(PSTR("Relay turned on"));
			} else {
				relay_toggle();
				port->println_P(PSTR("Relay toggled"));
			}

		//------- set throttlecruise <X> -------
		} else if (strmatch(buf+4, PSTR("throttlecruise "))) {
			port->print_P(PSTR("Throttle cruise changed from "));  port->print(get(PARAM_TRIM_THROTTLE),DEC);  port->print_P(PSTR("% to "));
			set(PARAM_TRIM_THROTTLE,atoi(buf+19));
			port->print(get(PARAM_TRIM_THROTTLE),DEC);  port->println_P(PSTR("%"));

		//------- set throttlefailsafe <N> -------
		} else if (strmatch(buf+4, PSTR("throttlefailsafe "))) {
			set(PARAM_THR_FAILSAFE, atoi(buf+21));
			/*if (get(PARAM_THROTTLE_FS) == 0)
				throttle_FS_enabled = 0;
			else
				throttle_FS_enabled = 1;*/
			//save_user_configs();
			port->println_P(PSTR("Throttle failsafe adjusted."));

		//------- set throttlefailsafeaction <N> -------
		} else if (strmatch(buf+4, PSTR("throttlefailsafeaction "))) {
			set(PARAM_THR_FS_ACTION, atoi(buf+27));
			//save_user_configs();

		//------- set throttlemax <X> -------
		} else if (strmatch(buf+4, PSTR("throttlemax "))) {
			port->print_P(PSTR("Throttle max changed from "));  port->print(get(PARAM_THR_MAX),DEC);  port->print_P(PSTR("% to "));
			set(PARAM_THR_MAX, atoi(buf+16));
			port->print(get(PARAM_THR_MAX),DEC);  port->println_P(PSTR("%"));
			//save_user_configs();

		//------- set throttlemin <X> -------
		} else if (strmatch(buf+4, PSTR("throttlemin "))) {
			port->print_P(PSTR("Throttle min changed from "));  port->print(get(PARAM_THR_MIN),DEC);  port->print_P(PSTR("% to "));
			set(PARAM_THR_MIN, atoi(buf+16));
			port->print(get(PARAM_THR_MIN),DEC);  port->println_P(PSTR("%"));
			//save_user_configs();

		//------- set wpradius -------
		} else if (strmatch(buf+4, PSTR("wpradius "))) {
			set(PARAM_WP_RADIUS, atoi(buf+13));
			port->print_P(PSTR("Set waypoint radius to "));  port->print(get(PARAM_WP_RADIUS),DEC);  port->println_P(PSTR(" meters"));

		//------- set xtrackentryangle -------
		} else if (strmatch(buf+4, PSTR("xtrackentryangle "))) {
			unsigned char j = 21;
			set(PARAM_XTRACK_ANGLE, readfloat(buf, &j)/100000);
			port->print_P(PSTR("Crosstrack entry angle set to "));  port->println((float)get(PARAM_XTRACK_ANGLE)/100,2);

		//------- set xtrackgain -------
		} else if (strmatch(buf+4, PSTR("xtrackgain "))) {
			unsigned char j = 15;
			set(PARAM_XTRACK_GAIN, ((float)readfloat(buf, &j))/10000000);
			port->print_P(PSTR("Crosstrack gain set to "));  port->println(get(PARAM_XTRACK_GAIN),2);
			
		} else
			print_error(ERR("USAGE: set {cmd|cmdcount|cmdindex|cruise|holdalt|kp|ki|kd|kff|loiterradius|rcin|rcout|wpradius}"));

	//*********** Process 'status' commands ***********
	} else if (strmatch(buf, PSTR("status "))) {
		//------- status altitude -------
		if (strmatch(buf+7, PSTR("altitude"))) {
			port->printf_P(PSTR("Altitude:\n"
								"  altitude_error = %.2fm\n"
								"  target_altitude = %.2fm\n"
								"  next_WP.alt = %.2fm\n"
								"  wp_distance = %ldm\n"
								"  wp_totalDistance = %ldm\n"
								"  offset_altitude = %.2fm\n"),
								(float)altitude_error/100,
								(float)target_altitude/100,
								(float)next_WP.alt/100,
								wp_distance,
								wp_totalDistance,
								(float)offset_altitude/100);

		//------- status climb -------
		} else if (strmatch(buf+7, PSTR("climb"))) {
			print_climb_debug_info();

		//------- status control -------
		} else if (strmatch(buf+7, PSTR("control"))) {
			port->printf_P(PSTR("Control status:\n"
								"  Roll: nav= %.2f, servo_out= %.2f\n"	// XXX float?
								"  Pitch: nav= %.2f, servo_out= %.2f\n"	// XXX float?
								"  Throttle: nav= %d, servo_out= %d\n"),
						   (float)nav_roll  / 100, (float)servo_out[CH_ROLL]  / 100,
						   (float)nav_pitch / 100, (float)servo_out[CH_PITCH] / 100,
						   get(PARAM_TRIM_THROTTLE), servo_out[CH_THROTTLE]);								

		//------- status gps -------
		} else if (strmatch(buf+7, PSTR("gps"))) {
			port->printf_P(PSTR("GPS status:\n"
								"  Fix: %S (%d)\n"
								"  Satellites: %d\n"
								"  Fix count: %d\n"),
						   (gps.fix ? PSTR("Valid") : PSTR("INVALID")),
						   (int)gps.fix,
						   (int)gps.num_sats,
						   gps_fix_count);

		//------- status landing -------
		} else if (strmatch(buf+7, PSTR("landing"))) {
			port->printf_P(PSTR("Landing status:"
								"  land_complete = %d\n"
								"  landing_pitch = %d\n"
								"  nav_pitch = %ld\n"
								"  airspeed_cruise = %d\n"
								"  throttle_cruise = %d\n"
								"  hold_course = %ld\n"
								"  nav_bearing = %ld\n"
								"  bearing_error = %ld\n"),
						   (int)land_complete,
						   landing_pitch,
						   nav_pitch,
						   get(PARAM_TRIM_AIRSPEED),
						   get(PARAM_TRIM_THROTTLE),
						   hold_course,
						   nav_bearing,
						   bearing_error);
			
		//------- status loiter -------
		} else if (strmatch(buf+7, PSTR("loiter"))) {
			port->printf_P(PSTR("Loiter status:"
								"  Loiter radius: %d\n"
								"  Progress: %d/%d\n"	// XXX original had non-ASCII units char
								"  Time: %ldms/%dms\n"),
						   get(PARAM_LOITER_RADIUS),
						   loiter_sum, loiter_total,
						   loiter_time, loiter_time_max);

		//------- status navigation -------
		} else if (strmatch(buf+7, PSTR("navigation"))) {
			port->printf_P(PSTR("Navigation status:\n"
								"  From <%.6f, %.6f, %.1f>: "),
								(float)prev_WP.lat/10000000.0,
								(float)prev_WP.lng/10000000.0,
								(float)prev_WP.alt/100.0);
			print_rlocation(&prev_WP);
			port->printf_P(PSTR("  At <%.6f, %.6f, %.1f>\n"),
								(float)current_loc.lat/10000000.0,
								(float)current_loc.lng/10000000.0,
								(float)current_loc.alt/100.0);
			port->printf_P(PSTR("  To <%.6f, %.6f, %.1f>: "),
								(float)next_WP.lat/10000000.0,
								(float)next_WP.lng/10000000.0,
								(float)next_WP.alt/100.0);
			print_rlocation(&next_WP);
			port->printf_P(PSTR("  Distance: %.1f%% %ldm / %ldm; %.1f vertically\n"),
								100.0*(float)(wp_totalDistance-wp_distance)/(float)wp_totalDistance,
								wp_totalDistance-wp_distance,
								wp_totalDistance,
								(float)altitude_error/100.0);
			port->printf_P(PSTR("  Nav bearing: %.2f; error = %.2f\n"),
								(float)nav_bearing/100.0,
								(float)bearing_error/100.0);
			port->printf_P(PSTR("  Ground course: %.1f (current), %.1f (target)\n"),
								(float)gps.ground_course/100.0,
								(float)target_bearing/100.0);
			if (hold_course >= 0) {
				port->print_P(PSTR("  HOLD COURSE: "));  port->println(hold_course/100.0,2);
			}

		//------- status navpitch -------
		} else if (strmatch(buf+7, PSTR("navpitch"))) {
#if AIRSPEED_SENSOR == ENABLED
			port->printf_P(PSTR(">>> nav_pitch = PID[airspeed_error (%.2f) = airspeed_cruise (%.2f) - airspeed (%.2f)]\n"),
						   (float)airspeed_error / 100,
						   (float)get(PARAM_TRIM_AIRSPEED) / 100,
						   (float)airspeed / 100);
#else
			port->printf_P(PSTR(">>> nav_pitch = PID[altitude_error (%ld) = target_altitude (%ld) - current_loc.alt (%ld)]\n"),
						   altitude_error,
						   target_altitude,
						   current_loc.alt);
#endif
			port->printf_P(PSTR("nav_pitch (%.2f) - pitch_sensor (%.2f) + pitch_comp (%.2f) = %.2f\n"),
						   (float)nav_pitch / 100,
						   (float)dcm.pitch_sensor / 100,
						   fabs(dcm.roll_sensor * get(PARAM_KFF_PTCHCOMP)) / 100,
						   (float)(nav_pitch-dcm.pitch_sensor + fabs(dcm.roll_sensor * get(PARAM_KFF_PTCHCOMP))) / 100);
			port->printf_P(PSTR("servo_out[CH_PITCH] (%.2f) = PID[nav_pitch + pitch_comp - pitch_sensor]"),
						   (float)servo_out[CH_PITCH] / 100);

		//------- status navroll -------
		} else if (strmatch(buf+7, PSTR("navroll"))) {
			print_rlocation(&next_WP);
			port->printf_P(PSTR("bearing_error (%ld) = nav_bearing (%ld) - yaw_sensor (%ld)\n"
								"nav_roll (%ld) - roll_sensor (%ld) = %ld\n"
								"servo_out[CH_ROLL] = %d\n"),
						   bearing_error, nav_bearing, dcm.yaw_sensor,
						   nav_roll, dcm.roll_sensor, nav_roll - dcm.roll_sensor,
						   servo_out[CH_ROLL]);

		//------- status pid {servoroll|servopitch|servorudder|navroll|navpitchasp|navpitchalt|throttlete|throttlealt} -------
		} else if (strmatch(buf+7, PSTR("pid "))) {
			int i;
			unsigned char j;
			if (get_pid_offset_name(buf+11, &i, &j)) {
				port->print(buf+11);
				port->print_P(PSTR(": "));
				display_PID = i; //The next time a PID calculation is performed on this channel, the print_PID routine in this GCS will be called
			} else
				print_error(ERR("ERROR: Did not recognize keyword"));

		//------- status rcinputch -------
		} else if (strmatch(buf+7, PSTR("rcinputch"))) {
			unsigned char i;
			port->println_P(PSTR("RC hardware input:"));
			for (i=0; i<8; i++) {
				port->print_P(PSTR("  Ch"));
				port->print(i+1,DEC);
				port->print_P(PSTR(": "));
				port->println(APM_RC.InputCh(i));
			}

		//------- status rcin -------
		} else if (strmatch(buf+7, PSTR("rcin"))) {
			unsigned char i;
			port->println_P(PSTR("RC software input:"));
			for (i=0; i<8; i++) {
				port->print_P(PSTR("  Ch"));
				port->print(i+1,DEC);
				port->print_P(PSTR(": "));
				port->println(radio_in[i]);
			}

		//------- status rcout -------
		} else if (strmatch(buf+7, PSTR("rcout"))) {
			unsigned char i;
			port->println_P(PSTR("RC software output:"));
			for (i=0; i<8; i++) {
				port->print_P(PSTR("  Ch"));
				port->print(i+1,DEC);
				port->print_P(PSTR(": "));
				port->println(radio_out[i]);
			}

		//------- status rcpwm -------
		} else if (strmatch(buf+7, PSTR("rcpwm"))) {
			unsigned char i;
			port->println_P(PSTR("RC hardware output:"));
			for (i=0; i<8; i++) {
				port->print_P(PSTR("  Ch"));
				port->print(i+1,DEC);
				port->print_P(PSTR(": "));
				port->println(readOutputCh(i));
			}

		//------- status rctrim -------
		} else if (strmatch(buf+7, PSTR("rctrim"))) {
			unsigned char i;
			port->println_P(PSTR("RC trim:"));
			for (i=0; i<8; i++) {
				port->print_P(PSTR("  Ch"));
				port->print(i+1,DEC);
				port->print_P(PSTR(": "));
				port->println(radio_trim(i));
			}
			port->print_P(PSTR("  elevon1_trim = "));  port->println(elevon1_trim,DEC);
			port->print_P(PSTR("  elevon2_trim = "));  port->println(elevon2_trim,DEC);

		//------- status rc -------
		} else if (strmatch(buf+7, PSTR("rc"))) {
			unsigned char i;
			port->println_P(PSTR("RC:\tCh\tHWin\tSWtrim\tSWin\tServo\tSWout\tHWout\t"));
			for (i=0; i<8; i++) {
				// XXX might benefit from field widths, since some terminals tab badly
				port->printf_P(PSTR("\t%u\t%u\t%u\t%u\t%.2f\t%d\t%d\n"),
							   (unsigned int)(i + 1),
							   APM_RC.InputCh(i),
							   radio_trim(i),
							   radio_in[i],
							   (float)servo_out[i] / 100,
							   radio_out[i],
							   readOutputCh(i));
			}

		//------- status system -------
		} else if (strmatch(buf+7, PSTR("system"))) {
			port->printf_P(PSTR("System status:"
								"  Ground start: %S (%d)\n"
								"  Home: %SSET (%d)\n"),
						   (ground_start ? PSTR("YES") : PSTR("NO")), (int)ground_start,
						   (home_is_set  ? PSTR("") : PSTR("NOT ")), (int)home_is_set);

		//------- status takeoff -------
		/*} else if (strmatch(buf+7, PSTR("takeoff"))) {
			port->println_P(PSTR("Takeoff status:"));
			port->print_P(PSTR("  takeoff in progress: "));
			if (takeoff_in_progress)
				port->println_P(PSTR("YES"));
			else {
				port->print_P(PSTR("NO; trigger = "));
				port->print(toss_trigger,DEC);
				port->print_P(PSTR(", current = "));
				Vector3f temp = ardupilotDCM.get_accels();
				port->println(abs(temp.y),1);
			}
			port->printf_P(PSTR("  takeoff_pitch = %.2f\n"
								"  scaler = %.3f\n"
								"  nav_pitch = %.2f\n"
								"  throttle = %d\n"
								"  hold_course = %.2f\n"
								"  nav_bearing = %ld\n"
								"  bearing_error = %ld\n"
								"  current_loc.alt = %ld\n"
								"  home.alt + takeoff_altitude = %ld"),
						   (float)takeoff_pitch / 100,
						   (float)airspeed / (float)airspeed_cruise,
						   (float)nav_pitch / 100,
						   servo_out[CH_THROTTLE],
						   (float)hold_course / 100,
						   nav_bearing,
						   bearing_error,
						   current_loc.alt,
						   home.alt + takeoff_altitude);*/
			
		//------- status telemetry -------
		} else if (strmatch(buf+7, PSTR("telemetry"))) {
			port->printf_P(PSTR("Telemetry status:\n"
								"  %S heartbeat\n"
								"  %S location\n"
								"  %S attitude\n"
								"  %S command\n"
								"  Severity report level %d\n"),
						   (report_heartbeat ? PSTR("Show") : PSTR("Hide")),
						   (report_location  ? PSTR("Show") : PSTR("Hide")),
						   (report_attitude  ? PSTR("Show") : PSTR("Hide")),
						   (report_command   ? PSTR("Show") : PSTR("Hide")),
						   (int)report_severity);

		//------- status throttle -------
		} else if (strmatch(buf+7, PSTR("throttle"))) {
#if AIRSPEED_SENSOR == ENABLED
			port->printf_P(PSTR(">>> airspeed_energy_error (%ld) = 0.5 * (airspeed_cruise (%.2f)^2 - airspeed (%.2f)^2)\n"
								"energy_error (%ld) = airspeed_energy_error (%ld) + altitude_error*0.098 (%ld)\n"
								"servo_out[CH_THROTTLE] (%d) = PID[energy_error]\n"),
						   airspeed_energy_error, (float)get(PARAM_TRIM_AIRSPEED) / 100, (float)airspeed / 100,
						   energy_error, airspeed_energy_error, (long)((float)altitude_error * 0.098),
						   servo_out[CH_THROTTLE]);
#else
			port->printf_P(PSTR("altitude_error (%ld) = target_altitude (%ld) - current_loc.alt (%ld)\n"
								"servo_out[CH_THROTTLE] (%d) = PID[altitude_error]\n"),
						   altitude_error, target_altitude, current_loc.alt,
						   servo_out[CH_THROTTLE]);
#endif

		//------- status waypoints -------
		} else if (strmatch(buf+7, PSTR("waypoints"))) {
			port->printf_P(PSTR("Waypoints status:\n"
								"  Distance: %ld/%ld\n"
								"  Index: %d/%d\n"
								"  Next: %d\n"),
						   wp_distance, wp_totalDistance,
						   (int)get(PARAM_WP_INDEX), (int)get(PARAM_WP_TOTAL),
						   (int)next_wp_index);
			
		} else if (strmatch(buf+7, PSTR("james"))) {
			port->println_P(PSTR("James is a monkey"));
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
int
GCS_DEBUGTERMINAL::strmatch(char* s1, const char* s2)
{
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
   numbers are multiplied by 10,000,000 and decimals beyond 7 places are discarded
   parsing is terminated with either a space or a null, other characters are ignored */
long
GCS_DEBUGTERMINAL::readfloat(char* s, unsigned char* i) 
{
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

void 
GCS_DEBUGTERMINAL::process_set_cmd(char *buffer, int bufferlen) 
{
	BetterStream *port = _port;
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
			else if (strmatch(buffer+i, PSTR("relay")))
				cmdid = CMD_RELAY;
			else if (strmatch(buffer+i, PSTR("servo")))
				cmdid = CMD_SERVO;
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
				if (setparam == SETPARAM_ALT || setparam == SETPARAM_LAT || setparam == SETPARAM_LNG) {
					lat = readfloat(buffer, &i);
				} else {
					unsigned char k;
					for (k=i; i<sizeof(gcsinputbuffer); k++)
						if (buffer[k] != '-' && !(buffer[k] >= '0' && buffer[k] <= '9'))
							break;
					buffer[k] = 0;
					lat = atol(buffer+i);
					i = k;
				}

				temp = get_wp_with_index(cmdindex);
				if (setparam == SETPARAM_ID)
					temp.id = lat;
				else if (setparam == SETPARAM_P1)
					temp.p1 = lat;
				else if (setparam == SETPARAM_ALT)
					temp.alt = lat/100000;
				else if (setparam == SETPARAM_LAT)
					temp.lat = lat;
				else if (setparam == SETPARAM_LNG)
					temp.lng = lat;
				else if (setparam == SETPARAM_P2)
					temp.alt = lat;
				else if (setparam == SETPARAM_P3)
					temp.lat = lat;
				else if (setparam == SETPARAM_P4)
					temp.lng = lat;
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
						lat = gps.latitude;
						lng = gps.longitude;
						alt = get_altitude_above_home(); //GPS.altitude;
						err = 0;
					} else {
						alt = readfloat(buffer, &i)/100000;

						if (buffer[i] == ' ') {
							//Process latitude
							i++;
							lat = readfloat(buffer, &i);
							if (strmatch(buffer+i, PSTR("here"))) {
								lat = gps.latitude;
								lng = gps.longitude;
								err = 0;
							} else {
								if (buffer[i] == ' ') {
									//Process longitude
									i++;
									lng = readfloat(buffer, &i);

									//TODO: add other command special cases here
									if (cmdid == CMD_LAND_OPTIONS) {
										temp.p1 = p1;
										temp.alt = alt;
										temp.lat = lat / 10000000;
										temp.lng = lng / 10000000;
									}

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
		temp.p1 = p1;
		temp.lat = lat;
		temp.lng = lng;
		temp.alt = alt;

		if (cmdindex >= get(PARAM_WP_TOTAL)) {
			set(PARAM_WP_TOTAL, cmdindex+1);
		}
		set_wp_with_index(temp, cmdindex);

		port->printf_P(PSTR("Set command %d to %d with p1=%d, lat=%ld, lng=%ld, alt=%ld\n"),
					   (int)cmdindex, (int)temp.id, (int)temp.p1, temp.lat, temp.lng, temp.alt);
	}
}

/* get_pid_offset_name matches a string expressed in *buffer with a pid keyword and returns the k-array
   gain offset in *offset, and the length of that string expression in *length.  If a good keyword
   match is found, 1 is returned; 0 otherwise
*/
int
GCS_DEBUGTERMINAL::get_pid_offset_name(char *buffer, int *offset, unsigned char *length) 
{
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
void
GCS_DEBUGTERMINAL::print_rlocation(Location *wp) 
{
	//float x, y;
	//y = (float)(wp->lat - current_loc.lat) * 0.0111194927;
	//x = (float)(wp->lng - current_loc.lng) * cos(radians(current_loc.lat)) * 0.0111194927;
	BetterStream *port = _port;
	int x, y;
	y = (wp->lat - current_loc.lat) * 0.0111194927;
	x = (wp->lng - current_loc.lng) * cos(radians(current_loc.lat)) * 0.0111194927;
	port->printf_P(PSTR("dP = <%d%c, %d%c, %.1f> meters\n"),
				   abs(y), (y >= 0 ? 'N' : 'S'),
				   abs(x), (x >= 0 ? 'E' : 'W'),
				   (float)(wp->alt - current_loc.alt) / 100);
}

/* print_error prints an error message if the user sends an invalid command
*/
void
GCS_DEBUGTERMINAL::print_error(const char *msg) 
{
	BetterStream *port = _port;

	if (msg == 0)
		port->println_P(PSTR("ERROR: Invalid command"));
	else
		port->println_P(msg);
}


void
GCS_DEBUGTERMINAL::send_text(uint8_t severity, const char *str)
{
	BetterStream *port = _port;

	if(severity >= DEBUG_LEVEL){
		port->print_P(PSTR("MSG: "));
		port->println(str);
	}
}

void
GCS_DEBUGTERMINAL::send_message(uint8_t id, uint32_t param)
{
	switch(id) {
		case MSG_MODE_CHANGE:
			print_control_mode();
			break;

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
				send_text(0,"First location received");
				first_location = 1;
			}
			break;
		
		case MSG_CPU_LOAD:
			if (report_cpu_load) {
				_port->printf_P(PSTR("MSG: load %ld%%\n"), param);
			}
			break;

		case MSG_COMMAND_LIST:
			struct Location cmd = get_wp_with_index(param);
			print_command(&cmd, param);
			break;
	}
}
			
void
GCS_DEBUGTERMINAL::print_current_waypoints()
{
	_port->printf_P(PSTR("Current waypoints:"
						 "  Prev:\t%ld,\t%ld\t%ld\n"
						 "  Cur: \t%ld,\t%ld\t%ld\n"
						 "  Next:%d\t%ld,\t%ld\t%ld\n"),
					prev_WP.lat, prev_WP.lng, prev_WP.alt,	
					current_loc.lat, current_loc.lng, current_loc.alt,
					(int)get(PARAM_WP_INDEX), next_WP.lat, next_WP.lng, next_WP.alt);
}

void
GCS_DEBUGTERMINAL::print_position(void)
{
	recalc_climb_rate();
	_port->printf_P(PSTR("Pos: %ld, %ld, %ldcm, %ldcm/s GS, %d cm/s AS, %d cm above home, %d? climb, %ldm from wp, %d%% throttle, %ld alt err\n"),
					current_loc.lat, current_loc.lng, current_loc.alt,
					gps.ground_speed,
					airspeed,
					get_altitude_above_home(),
					climb_rate,
					wp_distance,
					get(PARAM_TRIM_THROTTLE),
					altitude_error);
}

void
GCS_DEBUGTERMINAL::print_attitude(void)
{
	_port->printf_P(PSTR("Att: %u roll_in, %u pitch in, %u throttle_in, "
						 "%ld roll_sensor, %ld pitch_sensor, "
						 "%ld ground_course, %ld target_bearing, "
						 "%ld nav_roll, %d loiter_sum, "
						 "%d roll servo_out, %d pitch_servo_out\n"),
					radio_in[CH_ROLL], radio_in[CH_PITCH], radio_in[CH_THROTTLE],
					dcm.roll_sensor, dcm.pitch_sensor,
					gps.ground_course, target_bearing,
					nav_roll, loiter_sum,
					servo_out[CH_ROLL], servo_out[CH_PITCH]);
}

// required by Groundstation to plot lateral tracking course 
void
GCS_DEBUGTERMINAL::print_new_wp_info()
{
	_port->printf_P(PSTR("New waypt (%d): %ld, %ld, %ldm -> "
						 "%ld, %ld, %ldm; "
						 "%ldm dist, %u roll trim, %u pitch trim\n"),
					(int)get(PARAM_WP_INDEX),
					prev_WP.lat, prev_WP.lng, prev_WP.alt,
					next_WP.lat, next_WP.lng, next_WP.alt,
					wp_totalDistance,
					radio_trim(CH_ROLL), radio_trim(CH_PITCH));
}

void
GCS_DEBUGTERMINAL::print_control_mode(void)
{
	BetterStream *port = _port;

	switch (control_mode){
		case MANUAL:
			port->println_P(PSTR("##MANUAL"));
			break;
		case STABILIZE:
			port->println_P(PSTR("##STABILIZE"));
			break;
		case FLY_BY_WIRE_A:
			port->println_P(PSTR("##FBW A"));
			break;
		case FLY_BY_WIRE_B:
			port->println_P(PSTR("##FBW B"));
			break;
		case AUTO:
			port->println_P(PSTR("##AUTO"));
			break;
		case RTL:
			port->println_P(PSTR("##RTL"));
			break;
		case LOITER:
			port->println_P(PSTR("##LOITER"));
			break;
		case 98:
			port->println_P(PSTR("##Air Start Complete"));
			break;
		case 99:
			port->println_P(PSTR("##Ground Start Complete"));
			break;
	}
}

void
GCS_DEBUGTERMINAL::print_tuning(void) 
{
	_port->printf_P(PSTR("TUN:%d,    %ld,     %ld,    %d,    %ld,    %ld\n"),
					servo_out[CH_ROLL],  nav_roll  / 100, dcm.roll_sensor  / 100,
					servo_out[CH_PITCH], nav_pitch / 100, dcm.pitch_sensor / 100);					
}

void
GCS_DEBUGTERMINAL::print_command_id(byte id)
{
	BetterStream *port = _port;

	switch (id) {
		// Command IDs - Must
		case CMD_BLANK: port->print_P(PSTR("CMD_BLANK"));  break;
		case CMD_WAYPOINT: port->print_P(PSTR("waypoint"));  break;
		case CMD_LOITER: port->print_P(PSTR("loiter"));  break;
		case CMD_LOITER_N_TURNS: port->print_P(PSTR("loiternturns"));  break;
		case CMD_LOITER_TIME: port->print_P(PSTR("loitertime"));  break;
		case CMD_RTL: port->print_P(PSTR("rtl"));  break;
		case CMD_LAND: port->print_P(PSTR("land"));  break;
		case CMD_TAKEOFF: port->print_P(PSTR("takeoff"));  break;

		// Command IDs - May
		case CMD_DELAY: port->print_P(PSTR("delay"));  break;
		case CMD_CLIMB: port->print_P(PSTR("climb"));  break;
		case CMD_LAND_OPTIONS: port->print_P(PSTR("landoptions"));  break;

		// Command IDs - Now
		case CMD_RESET_INDEX: port->print_P(PSTR("resetindex"));  break;
		case CMD_GOTO_INDEX: port->print_P(PSTR("CMD_GOTO_INDEX"));  break;
		case CMD_GETVAR_INDEX: port->print_P(PSTR("CMD_GETVAR_INDEX"));  break;
		case CMD_SENDVAR_INDEX: port->print_P(PSTR("CMD_SENDVAR_INDEX"));  break;
		case CMD_TELEMETRY: port->print_P(PSTR("CMD_TELEMETRY"));  break;

		case CMD_THROTTLE_CRUISE: port->print_P(PSTR("throttlecruise"));  break;
		case CMD_AIRSPEED_CRUISE: port->print_P(PSTR("CMD_AIRSPEED_CRUISE"));  break;
		case CMD_RESET_HOME: port->print_P(PSTR("resethome"));  break;

		case CMD_KP_GAIN: port->print_P(PSTR("CMD_KP_GAIN"));  break;
		case CMD_KI_GAIN: port->print_P(PSTR("CMD_KI_GAIN"));  break;
		case CMD_KD_GAIN: port->print_P(PSTR("CMD_KD_GAIN"));  break;
		case CMD_KI_MAX: port->print_P(PSTR("CMD_KI_MAX"));  break;
		case CMD_KFF_GAIN: port->print_P(PSTR("CMD_KFF_GAIN"));  break;

		case CMD_RADIO_TRIM: port->print_P(PSTR("CMD_RADIO_TRIM"));  break;
		case CMD_RADIO_MAX: port->print_P(PSTR("CMD_RADIO_MAX"));  break;
		case CMD_RADIO_MIN: port->print_P(PSTR("CMD_RADIO_MIN"));  break;
		case CMD_ELEVON_TRIM: port->print_P(PSTR("CMD_ELEVON_TRIM"));  break;

		case CMD_INDEX: port->print_P(PSTR("index"));  break;
		case CMD_REPEAT: port->print_P(PSTR("CMD_REPEAT"));  break;
		case CMD_RELAY: port->print_P(PSTR("relay"));  break;
		case CMD_SERVO: port->print_P(PSTR("servo"));  break;

		default: port->print(id,DEC);
	}
}

void
GCS_DEBUGTERMINAL::print_command(struct Location *cmd,byte index)
{
	BetterStream *port = _port;

	port->print_P(PSTR("    command #: "));
	port->print(index, DEC);
	port->print_P(PSTR(" id: "));
	print_command_id(cmd->id);

	port->print_P(PSTR(" p1: "));
	port->print(cmd->p1,DEC);
	port->print_P(PSTR(" p2/alt: "));
	port->print(cmd->alt,DEC);
	port->print_P(PSTR(" p3/lat: "));
	port->print(cmd->lat,DEC);
	port->print_P(PSTR(" p4/lng: "));
	port->println(cmd->lng,DEC);
}

void
GCS_DEBUGTERMINAL::print_commands()
{
	print_commands(1, get(PARAM_WP_TOTAL));
}

void
GCS_DEBUGTERMINAL::print_commands(unsigned char i1, unsigned char i2)
{
	BetterStream *port = _port;

	port->println_P(PSTR("Commands in memory:"));
	port->print_P(PSTR("  "));
	port->print(get(PARAM_WP_TOTAL), DEC);
	port->println_P(PSTR(" commands total"));
	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	port->println_P(PSTR("  Home: "));
	struct Location cmd = get_wp_with_index(0);
	print_command(&cmd, 0);
	port->println_P(PSTR("  Commands: "));
	
	for (int i=i1; i<=i2; i++){
		cmd = get_wp_with_index(i);
		print_command(&cmd, i);
		delay(10);
	}
}

void
GCS_DEBUGTERMINAL::print_gains()
{
	BetterStream *port = _port;
	unsigned char i;

	port->println_P(PSTR("PID gains   \tP    \tI    \tD    \tIMax)"));
	port->print_P(PSTR("Servo roll  \t"));  print_gain(CASE_SERVO_ROLL);
	port->print_P(PSTR("Servo pitch \t"));  print_gain(CASE_SERVO_PITCH);
	port->print_P(PSTR("Servo yaw   \t"));  print_gain(CASE_SERVO_RUDDER);
	port->print_P(PSTR("Nav roll    \t"));  print_gain(CASE_NAV_ROLL);

	port->print_P(PSTR("Nav pitch   \t"));
	if (AIRSPEED_SENSOR)
		print_gain(CASE_NAV_PITCH_ASP);
	else
		print_gain(CASE_NAV_PITCH_ALT);

	port->print_P(PSTR("Nav throttle\t"));
	if (AIRSPEED_SENSOR)
		print_gain(CASE_TE_THROTTLE);
	else
		print_gain(CASE_ALT_THROTTLE);

	port->println_P(PSTR("Feed-forward gains"));
	port->print_P(PSTR("Pitch compensation\t"));  port->println(get(PARAM_KFF_PTCHCOMP),3);
	port->print_P(PSTR("Rudder mix        \t"));  port->println(get(PARAM_KFF_RDDRMIX),3);
	port->print_P(PSTR("Pitch to throttle \t"));  port->println(get(PARAM_KFF_PTCH2THR),3);
	//port->print_P(PSTR("Throttle to pitch \t"));  port->println(kff[CASE_T_TO_P],3);
}

void
GCS_DEBUGTERMINAL::print_gain(unsigned char g)
{
	BetterStream *port = _port;

	port->print(pid_index[g]->kP(),3);
	port->print_P(PSTR("\t"));
	port->print(pid_index[g]->kI(),3);
	port->print_P(PSTR("\t"));
	port->print(pid_index[g]->kD(),3);
	port->print_P(PSTR("\t"));
	port->println(pid_index[g]->imax(),DEC);
}

void
GCS_DEBUGTERMINAL::print_gain_keyword_error()
{
	print_error(ERR("ERROR: Did not recognize keyword; type k -? for more information"));
}

void
GCS_DEBUGTERMINAL::print_PID(long PID_error, long dt, float scaler, float derivative, float integrator, float last_error)
{
	BetterStream *port = _port;

	port->print_P(PSTR("P = "));			port->print(pid_index[display_PID]->kP() * scaler * (float)PID_error,2);
	port->print_P(PSTR(",\tI = "));		port->print(integrator,2);
	port->print_P(PSTR(",\tD = "));		port->print(pid_index[display_PID]->kD() * scaler * derivative,2);
	port->print_P(PSTR("\terrors = {"));	port->print(PID_error,DEC);
	port->print_P(PSTR(", "));				port->print(last_error,DEC);
	port->print_P(PSTR("} with dt = "));	port->println(dt,DEC);

	display_PID = -1;
}

#endif // GCS_PROTOCOL_DEBUGTERMINAL
