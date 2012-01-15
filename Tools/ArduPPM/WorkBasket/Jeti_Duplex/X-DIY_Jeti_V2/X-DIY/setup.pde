// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Functions called from the setup menu
static int8_t	setup_radio(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu 
const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"reset", 			setup_factory},
	{"radio",			setup_radio},
	{"modes",			setup_flightmodes},
	{"show",			setup_show},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
	// Give the user some guidance
	Serial.printf_P(PSTR("Setup Mode\n"
						 "\n"
						 "IMPORTANT: if you have not previously set this system up, use the\n"
						 "'reset' command to initialize the EEPROM to sensible default values\n"
						 "and then the 'radio' command to configure for your radio.\n"
						 "\n"));

	// Run the setup menu.  When the menu exits, we will return to the main menu.
	setup_menu.run();
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
	uint8_t		i;

	Serial.printf_P(PSTR("\nRadio:\n"));
	read_EEPROM_radio_minmax();			
	for(i = 0; i < 8; i++)
		Serial.printf_P(PSTR("CH%d: %d | %d\n"), i + 1, radio_min[i], radio_max[i]);

	Serial.printf_P(PSTR("\nGains:\n"));
	read_EEPROM_gains();

	for(i = 0; i < 8; i++){
		Serial.printf_P(PSTR("P,I,D,iMax:"));
		Serial.print(kp[i],3);
		Serial.print(",");
		Serial.print(ki[i],3);
		Serial.print(",");
		Serial.print(kd[i],3);
		Serial.print(",");
		Serial.println(integrator_max[i]/100,DEC);
	}
	
	Serial.printf_P(PSTR("kff:"));
	Serial.print(kff[0],3);
	Serial.print(",");
	Serial.print(kff[1],3);
	Serial.print(",");
	Serial.println(kff[2],3);
	
	Serial.printf_P(PSTR("XTRACK_GAIN:"));
	Serial.println(x_track_gain,DEC);

	Serial.printf_P(PSTR("XTRACK_ENTRY_ANGLE: %d\n"), x_track_angle);
	Serial.printf_P(PSTR("HEAD_MAX: %d\n"), head_max);
	Serial.printf_P(PSTR("PITCH_MAX: %d\n"), pitch_max);
	Serial.printf_P(PSTR("PITCH_MIN: %d\n"), pitch_min);

	read_user_configs();
	Serial.printf_P(PSTR("\nUser config:\n"));
	Serial.printf_P(PSTR("airspeed_cruise: %d\n"), airspeed_cruise);
	Serial.printf_P(PSTR("airspeed_fbw_min: %d\n"), airspeed_fbw_min);
	Serial.printf_P(PSTR("airspeed_fbw_max: %d\n"), airspeed_fbw_max);
	Serial.printf_P(PSTR("airspeed_ratio: "));
	Serial.println(airspeed_ratio, 4);
	
	Serial.printf_P(PSTR("throttle_min: %d\n"), throttle_min);
	Serial.printf_P(PSTR("throttle_max: %d\n"), throttle_max);
	Serial.printf_P(PSTR("throttle_cruise: %d\n"), throttle_cruise);
	Serial.printf_P(PSTR("throttle_failsafe_enabled: %d\n"), throttle_failsafe_enabled);
	Serial.printf_P(PSTR("throttle_failsafe_value: %d\n"), throttle_failsafe_value);
	Serial.printf_P(PSTR("flight_mode_channel: %d\n"), flight_mode_channel+1); //Add 1 to flight_mode_channel to change 0-based channels to 1-based channels
	Serial.printf_P(PSTR("auto_trim: %d\n"), auto_trim);
	Serial.printf_P(PSTR("log_bitmask: %d\n\n"), log_bitmask);
	//Serial.printf_P(PSTR("Switch settings:\n"));
	//for(i = 0; i < 6; i++){
	//	print_switch(i+1, flight_modes[i]);
	//}

	return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
	uint8_t		i;
	int			c;

	Serial.printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort: "));

	do {
		c = Serial.read();
	} while (-1 == c);
	
	if (('y' != c) && ('Y' != c))
		return(-1);

	Serial.printf_P(PSTR("\nFACTORY RESET\n\n"));

	wp_radius = WP_RADIUS_DEFAULT;
	loiter_radius = LOITER_RADIUS_DEFAULT;
	
	save_EEPROM_waypoint_info();
	
	radio_min[CH_1] = 0;
	radio_min[CH_2] = 0;
	radio_min[CH_3] = 0;
	radio_min[CH_4] = 0;
	radio_min[CH_5] = CH5_MIN;
	radio_min[CH_6] = CH6_MIN;
	radio_min[CH_7] = CH7_MIN;
	radio_min[CH_8] = CH8_MIN;

	radio_max[CH_1] = 0;
	radio_max[CH_2] = 0;
	radio_max[CH_3] = 0;
	radio_max[CH_4] = 0;
	radio_max[CH_5] = CH5_MAX;
	radio_max[CH_6] = CH6_MAX;
	radio_max[CH_7] = CH7_MAX;
	radio_max[CH_8] = CH8_MAX;

	save_EEPROM_radio_minmax();

	kp[0] = SERVO_ROLL_P;
	kp[1] = SERVO_PITCH_P;
	kp[2] = SERVO_YAW_P;
	kp[3] = NAV_ROLL_P;
	kp[4] = NAV_PITCH_ASP_P;
	kp[5] = NAV_PITCH_ALT_P;
	kp[6] = THROTTLE_TE_P;
	kp[7] = THROTTLE_ALT_P;

	ki[0] = SERVO_ROLL_I;
	ki[1] = SERVO_PITCH_I;
	ki[2] = SERVO_YAW_I;
	ki[3] = NAV_ROLL_I;
	ki[4] = NAV_PITCH_ASP_I;
	ki[5] = NAV_PITCH_ALT_I;
	ki[6] = THROTTLE_TE_I;
	ki[7] = THROTTLE_ALT_I;
	
	kd[0] = SERVO_ROLL_D;
	kd[1] = SERVO_PITCH_D;
	kd[2] = SERVO_YAW_D;
	kd[3] = NAV_ROLL_D;
	kd[4] = NAV_PITCH_ASP_D;
	kd[5] = NAV_PITCH_ALT_D;
	kd[6] = THROTTLE_TE_D;
	kd[7] = THROTTLE_ALT_D;
	
	integrator_max[0] = SERVO_ROLL_INT_MAX;
	integrator_max[1] = SERVO_PITCH_INT_MAX;
	integrator_max[2] = SERVO_YAW_INT_MAX;
	integrator_max[3] = NAV_ROLL_INT_MAX;
	integrator_max[4] = NAV_PITCH_ASP_INT_MAX;
	integrator_max[5] = NAV_PITCH_ALT_INT_MAX;
	integrator_max[6] = THROTTLE_TE_INT_MAX;
	integrator_max[7] = THROTTLE_ALT_INT_MAX;
	
	kff[0] = PITCH_COMP;
	kff[1] = RUDDER_MIX;
	kff[2] = P_TO_T;
	
	for(i = 0; i < 8; i++){
		// scale input to deg * 100;
		integrator_max[i] *= 100;
	}
	x_track_gain 	= XTRACK_GAIN * 100;
	x_track_angle 	= XTRACK_ENTRY_ANGLE * 100;
	altitude_mix 	= ALTITUDE_MIX;
	
	head_max 	= HEAD_MAX * 100;
	pitch_max 	= PITCH_MAX * 100;
	pitch_min	= PITCH_MIN * 100;	
		
	save_EEPROM_gains();

	airspeed_cruise				= AIRSPEED_CRUISE*100;
	airspeed_fbw_min			= AIRSPEED_FBW_MIN;
	airspeed_fbw_max 			= AIRSPEED_FBW_MAX;
	airspeed_ratio 				= AIRSPEED_RATIO;
	throttle_min				= THROTTLE_MIN;
	throttle_max				= THROTTLE_MAX;
	throttle_cruise				= THROTTLE_CRUISE;
	throttle_failsafe_enabled	= THROTTLE_FAILSAFE;
	throttle_failsafe_action	= THROTTLE_FAILSAFE_ACTION;
	throttle_failsafe_value		= THROTTLE_FS_VALUE;
	//flight_mode_channel 		= FLIGHT_MODE_CHANNEL - 1;
	auto_trim					= AUTO_TRIM;

	flight_modes[0] 			= FLIGHT_MODE_1;
	flight_modes[1] 			= FLIGHT_MODE_2;
	flight_modes[2] 			= FLIGHT_MODE_3;
	flight_modes[3] 			= FLIGHT_MODE_4;
	flight_modes[4] 			= FLIGHT_MODE_5;
	flight_modes[5] 			= FLIGHT_MODE_6;
	save_EEPROM_flight_modes();

	// convenience macro for testing LOG_* and setting LOGBIT_*
#define LOGBIT(_s)	(LOG_ ## _s ? LOGBIT_ ## _s : 0)
	log_bitmask = 
		LOGBIT(ATTITUDE_FAST)	|
		LOGBIT(ATTITUDE_MED)	|
		LOGBIT(GPS)				|
		LOGBIT(PM)				|
		LOGBIT(CTUN)			|
		LOGBIT(NTUN)			|
		LOGBIT(MODE)			|
		LOGBIT(RAW)				|
		LOGBIT(CMD);
#undef LOGBIT

	save_user_configs();

	return(0);
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
	Serial.println("\n\nRadio Setup:");
	uint8_t i;
	
	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}
	
	if(radio_in[CH_ROLL] < 500){
		while(1){
			Serial.print("Radio error");
			delay(1000);
			// stop here
		}
	}

	for(i = 0; i < 4; i++){
		radio_min[i] = radio_in[i];
		radio_max[i] = radio_in[i];
	}
		
	Serial.printf_P(PSTR("\nMove both sticks to each corner. Hit Enter to save: "));
	while(1){
		
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		for(i = 0; i < 4; i++){
			radio_min[i] = min(radio_min[i], radio_in[i]);
			radio_max[i] = max(radio_max[i], radio_in[i]);
		}
				
		if(Serial.available() > 0){
			Serial.flush();
			Serial.println("Saving:");
			
			save_EEPROM_radio_minmax();
			delay(100);
			read_EEPROM_radio_minmax();
			
			for(i = 0; i < 8; i++)
				Serial.printf_P(PSTR("CH%d: %d | %d\n"), i + 1, radio_min[i], radio_max[i]);
			break;
		}
	}
	
	return(0);
}


static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
	byte switchPosition, oldSwitchPosition, mode;
	
	Serial.printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
	print_hit_enter();
	trim_radio();
	
	while(1){
		delay(20);
		read_radio();		
		switchPosition = readSwitch();

		
		// look for control switch change
		if (oldSwitchPosition != switchPosition){
		
			// Override position 5
			if(switchPosition > 4){
				mode = flight_modes[switchPosition] = MANUAL;
			}else{
				// update our current mode
				mode = flight_modes[switchPosition];
			}

			// update the user
			print_switch(switchPosition, mode);

			// Remember switch position
			oldSwitchPosition = switchPosition;			
		}
		
		// look for stick input
		if (radio_input_switch() == true){
			switch(mode){
				case MANUAL:
					mode = STABILIZE;
					break;
		
				case STABILIZE:
					mode = FLY_BY_WIRE_A;
					break;
				
				case FLY_BY_WIRE_A:
					mode = FLY_BY_WIRE_B;
					break;
				
				case FLY_BY_WIRE_B:
					mode = AUTO;
					break;
		
				case AUTO:
					mode = RTL;
					break;
		
				case RTL:
					mode = LOITER;
					break;
				
				case LOITER:
					mode = MANUAL;
					break;

				default:
					mode = MANUAL;
					break;
			}
			
			// Override position 5
			if(switchPosition > 4)
				mode = MANUAL;
				
			// save new mode
			flight_modes[switchPosition] = mode;

			// print new mode
			print_switch(switchPosition, mode);
		}

		// escape hatch
		if(Serial.available() > 0){
			save_EEPROM_flight_modes();
			return (0);
		}
	}
}

void
print_switch(byte p, byte m)
{
	Serial.printf_P(PSTR("Pos %d: "),p);
	Serial.println(flight_mode_strings[m]);
}

boolean
radio_input_switch(void)
{
	static byte bouncer;
	
	if (abs(radio_in[0] - radio_trim[0]) > 200)
		bouncer = 10;

	if (bouncer > 0)
		bouncer--;

	if (bouncer == 1){
		return true;
	}else{
		return false;
	}
}

