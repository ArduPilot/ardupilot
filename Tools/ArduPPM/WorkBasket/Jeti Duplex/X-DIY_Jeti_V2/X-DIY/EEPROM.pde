// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//	************************************************************************************
//	This function gets critical data from EEPROM to get us underway if restarting in air
//	************************************************************************************
void read_EEPROM_startup(void)
{
	read_EEPROM_gains();
	read_EEPROM_radio_minmax();		// read Radio limits
	read_EEPROM_trims();			// read Radio trims
	read_user_configs();
	read_EEPROM_waypoint_info();
}

void read_EEPROM_airstart_critical(void)
{
	int16_t temp = 0;
	read_EEPROM_IMU_offsets();
	
	// For debugging only
	/*	
	Serial.print ("Offsets   ");		Serial.print (AN_OFFSET[0]);
	Serial.print ("   ");				Serial.print (AN_OFFSET[1]);
	Serial.print ("   ");				Serial.print (AN_OFFSET[2]);
	Serial.print ("   ");				Serial.print (AN_OFFSET[3]);
	Serial.print ("   ");				Serial.print (AN_OFFSET[4]);
	Serial.print ("   ");				Serial.println (AN_OFFSET[5]);
	*/	

	// Read the home location
	//-----------------------	
	home = get_wp_with_index(0);
		
	// Read pressure sensor values
	//----------------------------
	read_pressure_data();
}

void save_EEPROM_groundstart(void)
{
	save_EEPROM_trims();
	save_EEPROM_IMU_offsets();
	// pressure sensor data saved by init_home
}


/********************************************************************************/
long read_alt_to_hold()
{
	byte options = eeprom_read_byte((byte *) EE_CONFIG);
	
	// save the alitude above home option
	if(options & HOLD_ALT_ABOVE_HOME){
		int32_t temp = eeprom_read_dword((const uint32_t *) EE_ALT_HOLD_HOME);
		return temp + home.alt;
	}else{
		return current_loc.alt;
	}
}

long save_alt_to_hold(int32_t alt_to_hold)
{
	byte options = eeprom_read_byte((byte *) EE_CONFIG);
	
	// save the alitude above home option
	if(options & HOLD_ALT_ABOVE_HOME)
		eeprom_write_block((const void *)&alt_to_hold, (void *)EE_ALT_HOLD_HOME, sizeof (alt_to_hold));
}


/********************************************************************************/
void read_EEPROM_waypoint_info(void)
{
	wp_total 		= eeprom_read_byte((uint8_t *) EE_WP_TOTAL);				
	wp_radius 		= eeprom_read_byte((uint8_t *) EE_WP_RADIUS);
	loiter_radius 	= eeprom_read_byte((uint8_t *) EE_LOITER_RADIUS);
}

void save_EEPROM_waypoint_info(void)
{
	eeprom_write_byte((uint8_t *)	EE_WP_TOTAL, 		wp_total);
	eeprom_write_byte((uint8_t *)	EE_WP_RADIUS, 		wp_radius);
	eeprom_write_byte((uint8_t *)	EE_LOITER_RADIUS, 	loiter_radius);
}

/********************************************************************************/
void read_EEPROM_gains(void)
{	
	eeprom_read_block((void*)&kp, (const void*)EE_KP, sizeof(kp));
	eeprom_read_block((void*)&ki, (const void*)EE_KI, sizeof(ki));
	eeprom_read_block((void*)&kd, (const void*)EE_KD, sizeof(kd));
	eeprom_read_block((void*)&integrator_max, (const void*)EE_IMAX, sizeof(integrator_max));
	eeprom_read_block((void*)&kff, (const void*)EE_KFF, sizeof(kff));

	// stored as degree * 100
	x_track_gain 	= eeprom_read_word((uint16_t *)	EE_XTRACK_GAIN);

	// stored as degrees
	x_track_angle 	= eeprom_read_word((uint16_t *)	EE_XTRACK_ANGLE) * 100;

	// stored as degrees
	head_max 	= eeprom_read_byte((uint8_t *) EE_HEAD_MAX) * 100;	// scale to degress * 100
	pitch_max 	= eeprom_read_byte((uint8_t *) EE_PITCH_MAX) * 100;	// scale to degress * 100
	pitch_min 	= -eeprom_read_byte((uint8_t *) EE_PITCH_MIN) * 100; // scale to degress * 100
	
	// stored as a float
	eeprom_read_block((void*)&altitude_mix, (const void*)EE_ALT_MIX, sizeof(altitude_mix));
}

void save_EEPROM_gains(void)
{
 	eeprom_write_block((const void *)&kp, (void *)EE_KP, sizeof (kp));
 	eeprom_write_block((const void *)&ki, (void *)EE_KI, sizeof (ki));
 	eeprom_write_block((const void *)&kd, (void *)EE_KD, sizeof (kd));
 	eeprom_write_block((const void *)&integrator_max, (void *)EE_IMAX, sizeof (integrator_max));
 	eeprom_write_block((const void *)&kff, (void *)EE_KFF, sizeof (kff));

	// stored as degree * 100
 	eeprom_write_word((uint16_t *)	EE_XTRACK_GAIN, x_track_gain);

	// stored as degrees
 	eeprom_write_word((uint16_t *)	EE_XTRACK_ANGLE, x_track_angle / 100);

	// stored as degrees
	eeprom_write_byte((uint8_t *) EE_HEAD_MAX, head_max / 100);
	eeprom_write_byte((uint8_t *) EE_PITCH_MAX, pitch_max / 100);
	eeprom_write_byte((uint8_t *) EE_PITCH_MIN, -pitch_min / 100);
	
	// stored as a float
	eeprom_write_block((const void*)&altitude_mix, (void*)EE_ALT_MIX, sizeof(altitude_mix));
}


/********************************************************************************/

void read_EEPROM_trims(void)
{	
	// Read Radio trim settings
	eeprom_read_block((void*)&radio_trim, (const void*)EE_TRIM, sizeof(radio_trim));
	elevon1_trim 	= eeprom_read_word((uint16_t *)	EE_ELEVON1_TRIM); 
	elevon2_trim 	= eeprom_read_word((uint16_t *)	EE_ELEVON2_TRIM); 	
}

void save_EEPROM_trims(void)
{
	// Save Radio trim settings
 	eeprom_write_block((const void *)&radio_trim, (void *)EE_TRIM, sizeof(radio_trim));
	eeprom_write_word((uint16_t *)	EE_ELEVON1_TRIM, elevon1_trim);
	eeprom_write_word((uint16_t *)	EE_ELEVON2_TRIM, elevon2_trim);
}

/********************************************************************************/

void save_EEPROM_IMU_offsets(void)
{
 	eeprom_write_block((const void *)&AN_OFFSET, (void *)EE_AN_OFFSET, sizeof (AN_OFFSET));
}

void read_EEPROM_IMU_offsets(void)
{
	eeprom_read_block((void*)&AN_OFFSET, (const void*)EE_AN_OFFSET, sizeof(AN_OFFSET));
}

/********************************************************************************/

void save_command_index(void)
{
	eeprom_write_byte((uint8_t *) EE_WP_INDEX, command_must_index);
}

void read_command_index(void)
{
	wp_index = command_must_index 	= eeprom_read_byte((uint8_t *) EE_WP_INDEX);
}

/********************************************************************************/

void save_pressure_data(void)
{
	eeprom_write_dword((uint32_t *)EE_ABS_PRESS_GND, abs_press_gnd);
	eeprom_write_word((uint16_t *)EE_GND_TEMP, 	ground_temperature);
	eeprom_write_word((uint16_t *)EE_GND_ALT, 	(ground_alt / 100));

	#if AIRSPEED_SENSOR == 1
		eeprom_write_word((uint16_t *)EE_AP_OFFSET, airpressure_offset);
	#endif
}

void read_pressure_data(void)
{
	abs_press_gnd 		= eeprom_read_dword((uint32_t *) EE_ABS_PRESS_GND);
	abs_press_filt 		= abs_press_gnd;		// Better than zero for an air start value
	ground_temperature 	= eeprom_read_word((uint16_t *) EE_GND_TEMP);
	ground_alt 			= eeprom_read_word((uint16_t *) EE_GND_ALT) * 100;

	#if AIRSPEED_SENSOR == 1
		airpressure_offset = eeprom_read_word((uint16_t *) EE_AP_OFFSET);	
	#endif
}



/********************************************************************************/

void read_EEPROM_radio_minmax(void)
{
	// Read Radio min/max settings
	eeprom_read_block((void*)&radio_max, (const void*)EE_MAX, sizeof(radio_max));
	eeprom_read_block((void*)&radio_min, (const void*)EE_MIN, sizeof(radio_min));
}


void save_EEPROM_radio_minmax(void)
{
	// Save Radio min/max settings
 	eeprom_write_block((const void *)&radio_max, (void *)EE_MAX, sizeof(radio_max));
 	eeprom_write_block((const void *)&radio_min, (void *)EE_MIN, sizeof(radio_min));
}



/********************************************************************************/

void read_user_configs(void)
{
	// Read Radio min/max settings
	airspeed_cruise = eeprom_read_byte((byte *) EE_AIRSPEED_CRUISE)*100;
	eeprom_read_block((void*)&airspeed_ratio, (const void*)EE_AIRSPEED_RATIO, sizeof(airspeed_ratio));

	throttle_min 				= eeprom_read_byte((byte *) 	EE_THROTTLE_MIN);
	throttle_max 				= eeprom_read_byte((byte *) 	EE_THROTTLE_MAX);
	throttle_cruise 			= eeprom_read_byte((byte *) 	EE_THROTTLE_CRUISE);

	airspeed_fbw_min 			= eeprom_read_byte((byte *) 	EE_AIRSPEED_FBW_MIN);
	airspeed_fbw_max 			= eeprom_read_byte((byte *) 	EE_AIRSPEED_FBW_MAX);

	throttle_failsafe_enabled 	= eeprom_read_byte((byte *) 	EE_THROTTLE_FAILSAFE);
	throttle_failsafe_action 	= eeprom_read_byte((byte *) 	EE_THROTTLE_FAILSAFE_ACTION);
	throttle_failsafe_value 	= eeprom_read_word((uint16_t *) EE_THROTTLE_FS_VALUE);
	
	//flight_mode_channel 		= eeprom_read_byte((byte *) 	EE_FLIGHT_MODE_CHANNEL);
	auto_trim 					= eeprom_read_byte((byte *) 	EE_AUTO_TRIM);
	log_bitmask 				= eeprom_read_word((uint16_t *) EE_LOG_BITMASK);
	
	read_EEPROM_flight_modes();
	reverse_switch = eeprom_read_byte((byte *) 	EE_REVERSE_SWITCH);
}


void save_user_configs(void)
{
	eeprom_write_byte((byte *) 		EE_AIRSPEED_CRUISE, 		airspeed_cruise/100);
 	eeprom_write_block((const void *)&airspeed_ratio, (void *)EE_AIRSPEED_RATIO, sizeof(airspeed_ratio));

	eeprom_write_byte((byte *) 		EE_THROTTLE_MIN, 			throttle_min);
	eeprom_write_byte((byte *) 		EE_THROTTLE_MAX, 			throttle_max);
	eeprom_write_byte((byte *) 		EE_THROTTLE_CRUISE,			throttle_cruise);

	eeprom_write_byte((byte *) 		EE_AIRSPEED_FBW_MIN,		airspeed_fbw_min);	
	eeprom_write_byte((byte *) 		EE_AIRSPEED_FBW_MAX,		airspeed_fbw_max);

	eeprom_write_byte((byte *) 		EE_THROTTLE_FAILSAFE,		throttle_failsafe_enabled);
	eeprom_write_byte((byte *) 		EE_THROTTLE_FAILSAFE_ACTION,throttle_failsafe_action);
	eeprom_write_word((uint16_t *) 	EE_THROTTLE_FS_VALUE,		throttle_failsafe_value);
	
	//eeprom_write_byte((byte *) 		EE_FLIGHT_MODE_CHANNEL,		flight_mode_channel);
	eeprom_write_byte((byte *) 		EE_AUTO_TRIM,				auto_trim);
	eeprom_write_word((uint16_t *) 	EE_LOG_BITMASK,				log_bitmask);
	
 	//save_EEPROM_flight_modes();
	eeprom_write_byte((byte *) 		EE_REVERSE_SWITCH, 			reverse_switch);
}

/********************************************************************************/
void read_EEPROM_flight_modes(void)
{
	// Read Radio min/max settings
	eeprom_read_block((void*)&flight_modes, (const void*)EE_FLIGHT_MODES, sizeof(flight_modes));
}


void save_EEPROM_flight_modes(void)
{
	// Save Radio min/max settings
 	eeprom_write_block((const void *)&flight_modes, (void *)EE_FLIGHT_MODES, sizeof(flight_modes));
}

