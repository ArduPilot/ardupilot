// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//	************************************************************************************
//	This function gets critical data from EEPROM to get us underway if restarting in air
//	************************************************************************************

// Macro functions
// ---------------
void read_EEPROM_startup(void)
{
	read_EEPROM_PID();
	read_EEPROM_frame();
	read_EEPROM_configs();
	read_EEPROM_flight_modes();
	read_EEPROM_waypoint_info();
	imu.load_gyro_eeprom();
	imu.load_accel_eeprom();
	read_EEPROM_alt_RTL();

	// magnatometer
	read_EEPROM_mag();
	read_EEPROM_mag_declination();
	read_EEPROM_mag_offset();
}

void read_EEPROM_airstart_critical(void)
{
	int16_t temp = 0;
	imu.load_gyro_eeprom();
	imu.load_accel_eeprom();
	
	// Read the home location
	//-----------------------	
	home = get_wp_with_index(0);
		
	// Read pressure sensor values
	//----------------------------
	read_EEPROM_pressure();
}

void save_EEPROM_groundstart(void)
{
	rc_1.save_trim();
	rc_2.save_trim();
	rc_3.save_trim();
	rc_4.save_trim();
	rc_5.save_trim();
	rc_6.save_trim();
	rc_7.save_trim();
	rc_8.save_trim();
	
	// pressure sensor data saved by init_home
}

/********************************************************************************/

long read_alt_to_hold()
{	
	read_EEPROM_alt_RTL();
	if(alt_to_hold == -1)
		return current_loc.alt;
	else
		return alt_to_hold + home.alt;
}

/********************************************************************************/

void save_EEPROM_alt_RTL(void)
{
	eeprom_write_dword((uint32_t *)EE_ALT_HOLD_HOME, alt_to_hold);
}

void read_EEPROM_alt_RTL(void)
{
	alt_to_hold = eeprom_read_dword((const uint32_t *)	EE_ALT_HOLD_HOME);
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

void read_EEPROM_nav(void)
{
	// for nav estimation
	distance_gain 		= read_EE_compressed_float(EE_DISTANCE_GAIN, 4);
	altitude_gain 		= read_EE_compressed_float(EE_ALTITUDE_GAIN, 4);

	// stored as degree * 100
	x_track_gain 		= read_EE_compressed_float(EE_XTRACK_GAIN, 4);
	x_track_angle 		= eeprom_read_word((uint16_t *)	EE_XTRACK_ANGLE) * 100;
	pitch_max 			= eeprom_read_word((uint16_t *) EE_PITCH_MAX);	// scale to degress * 100
}

void save_EEPROM_nav(void)
{
	// for nav estimation
	write_EE_compressed_float(altitude_gain, 	EE_ALTITUDE_GAIN, 	4);
	write_EE_compressed_float(distance_gain, 	EE_DISTANCE_GAIN, 	4);
	write_EE_compressed_float(x_track_gain, 	EE_XTRACK_GAIN, 	4);
	
	// stored as degrees
 	eeprom_write_word((uint16_t *)	EE_XTRACK_ANGLE, x_track_angle / 100);

	// stored as degrees
	eeprom_write_word((uint16_t *)	EE_PITCH_MAX, pitch_max);
}

/********************************************************************************/

void read_EEPROM_PID(void)
{
	pid_acro_rate_roll.load_gains();
	pid_acro_rate_pitch.load_gains();
	pid_acro_rate_yaw.load_gains();
	
	pid_stabilize_roll.load_gains();
	pid_stabilize_pitch.load_gains();
	pid_yaw.load_gains();
	
	pid_nav.load_gains();
	pid_throttle.load_gains();
	
	stabilize_rate_roll_pitch 	= read_EE_compressed_float(EE_STAB_RATE_RP, 4);
	stabilize_rate_yaw			= read_EE_compressed_float(EE_STAB_RATE_YAW, 4);
	acro_rate_roll_pitch 		= read_EE_compressed_float(EE_ACRO_RATE_RP, 4);
	acro_rate_yaw 				= read_EE_compressed_float(EE_ACRO_RATE_YAW, 4);
}

void save_EEPROM_PID(void)
{
	pid_acro_rate_roll.save_gains(); 
	pid_acro_rate_pitch.save_gains();
	pid_acro_rate_yaw.save_gains();
	
	pid_stabilize_roll.save_gains();
	pid_stabilize_pitch.save_gains();
	pid_yaw.save_gains();
	
	pid_nav.save_gains();
	pid_throttle.save_gains();

	write_EE_compressed_float(stabilize_rate_roll_pitch, 	EE_STAB_RATE_RP, 4);
	write_EE_compressed_float(stabilize_rate_yaw, 			EE_STAB_RATE_YAW, 4);
	write_EE_compressed_float(acro_rate_roll_pitch, 		EE_ACRO_RATE_RP, 4);
	write_EE_compressed_float(acro_rate_yaw, 				EE_ACRO_RATE_YAW, 4);
}

/********************************************************************************/

void save_EEPROM_frame(void)
{
	eeprom_write_byte((uint8_t *)EE_FRAME, frame_type);
}

void read_EEPROM_frame(void)
{
	frame_type 	= eeprom_read_byte((uint8_t *)	EE_FRAME);
}

/********************************************************************************/

void save_EEPROM_throttle_cruise(void)
{
	eeprom_write_word((uint16_t *)EE_THROTTLE_CRUISE, throttle_cruise);
}

void read_EEPROM_throttle_cruise(void)
{
	throttle_cruise 	= eeprom_read_word((uint16_t *)	EE_THROTTLE_CRUISE);
}

/********************************************************************************/

void save_EEPROM_mag_declination(void)
{
	write_EE_compressed_float(mag_declination, 	EE_MAG_DECLINATION, 1);
}

void read_EEPROM_mag_declination(void)
{
	mag_declination 	= read_EE_compressed_float(EE_MAG_DECLINATION, 1);
}

/********************************************************************************/

void save_EEPROM_mag_offset(void)
{
	write_EE_compressed_float(mag_offset_x, 	EE_MAG_X, 2);
	write_EE_compressed_float(mag_offset_y, 	EE_MAG_Y, 2);
	write_EE_compressed_float(mag_offset_z, 	EE_MAG_Z, 2);
}

void read_EEPROM_mag_offset(void)
{
	mag_offset_x 	= read_EE_compressed_float(EE_MAG_X, 2);
	mag_offset_y 	= read_EE_compressed_float(EE_MAG_Y, 2);
	mag_offset_z 	= read_EE_compressed_float(EE_MAG_Z, 2);
}

/********************************************************************************/

void read_EEPROM_mag(void)
{
	compass_enabled	= eeprom_read_byte((uint8_t *) EE_COMPASS);
}

void save_EEPROM_mag(void)
{
	eeprom_write_byte((uint8_t *)	EE_COMPASS, 	compass_enabled);
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

void save_EEPROM_pressure(void)
{
	eeprom_write_dword((uint32_t *)EE_ABS_PRESS_GND, 	abs_pressure_ground);
	eeprom_write_word((uint16_t *)EE_GND_TEMP, 			ground_temperature);
}

void read_EEPROM_pressure(void)
{
	abs_pressure_ground = eeprom_read_dword((uint32_t *) 	EE_ABS_PRESS_GND);
	// Better than zero for an air start value
	abs_pressure	 	= abs_pressure_ground;
	ground_temperature 	= eeprom_read_word((uint16_t *) 	EE_GND_TEMP);
}

/********************************************************************************/

void read_EEPROM_radio(void)
{
	rc_1.load_eeprom();
	rc_2.load_eeprom();
	rc_3.load_eeprom();
	rc_4.load_eeprom();
	rc_5.load_eeprom();
	rc_6.load_eeprom();
	rc_7.load_eeprom();
	rc_8.load_eeprom();
}

void save_EEPROM_radio(void)
{	
	rc_1.save_eeprom();
	rc_2.save_eeprom();
	rc_3.save_eeprom();
	rc_4.save_eeprom();
	rc_5.save_eeprom();
	rc_6.save_eeprom();
	rc_7.save_eeprom();
	rc_8.save_eeprom();
}

/********************************************************************************/
// configs are the basics
void read_EEPROM_configs(void)
{
	throttle_min 				= eeprom_read_word((uint16_t *) 	EE_THROTTLE_MIN);
	throttle_max 				= eeprom_read_word((uint16_t *) 	EE_THROTTLE_MAX);
	read_EEPROM_throttle_cruise();
	throttle_failsafe_enabled 	= eeprom_read_byte((byte *) 	EE_THROTTLE_FAILSAFE);
	throttle_failsafe_action 	= eeprom_read_byte((byte *) 	EE_THROTTLE_FAILSAFE_ACTION);
	throttle_failsafe_value 	= eeprom_read_word((uint16_t *) EE_THROTTLE_FS_VALUE);
	log_bitmask 				= eeprom_read_word((uint16_t *) EE_LOG_BITMASK);
}

void save_EEPROM_configs(void)
{
	eeprom_write_word((uint16_t *) 	EE_THROTTLE_MIN, 			throttle_min);
	eeprom_write_word((uint16_t *) 	EE_THROTTLE_MAX, 			throttle_max);
	save_EEPROM_throttle_cruise();
	eeprom_write_byte((byte *) 		EE_THROTTLE_FAILSAFE,		throttle_failsafe_enabled);
	eeprom_write_byte((byte *) 		EE_THROTTLE_FAILSAFE_ACTION,throttle_failsafe_action);
	eeprom_write_word((uint16_t *) 	EE_THROTTLE_FS_VALUE,		throttle_failsafe_value);
	eeprom_write_word((uint16_t *) 	EE_LOG_BITMASK,				log_bitmask);
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

/********************************************************************************/

float
read_EE_float(int address)
{
	union {
		byte bytes[4];
		float value;
	} _floatOut;
	
	for (int i = 0; i < 4; i++) 
		_floatOut.bytes[i] = eeprom_read_byte((uint8_t *) (address + i));
	return _floatOut.value;
}

void write_EE_float(float value, int address)
{
	union {
		byte bytes[4];
		float value;
	} _floatIn;
	
	_floatIn.value = value;
	for (int i = 0; i < 4; i++) 
		eeprom_write_byte((uint8_t *) (address + i), _floatIn.bytes[i]);
}

/********************************************************************************/

float
read_EE_compressed_float(int address, byte places)
{
	float scale = pow(10, places);
	
	int temp 	= eeprom_read_word((uint16_t *) address);
	return ((float)temp / scale);
}

void write_EE_compressed_float(float value, int address, byte places)
{
	float scale = pow(10, places);
	int temp 	= value * scale;
	eeprom_write_word((uint16_t *) 	address, temp);
}
