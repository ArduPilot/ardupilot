// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//	************************************************************************************
//	This function gets critical data from EEPROM to get us underway if restarting in air
//	************************************************************************************

// Macro functions
// ---------------
/*void read_EEPROM_startup(void)
{
	read_EEPROM_PID();
	read_EEPROM_frame();
	read_EEPROM_throttle();
	read_EEPROM_logs();
	read_EEPROM_flight_modes();
	read_EEPROM_waypoint_info();
	//imu.load_gyro_eeprom();
	//imu.load_accel_eeprom();
	read_EEPROM_alt_RTL();
	read_EEPROM_current();
	read_EEPROM_nav();
	// magnatometer
	read_EEPROM_compass();
	//read_EEPROM_compass_declination();
	read_EEPROM_compass_offset();
}
*/

void save_EEPROM_groundstart(void)
{
	g.rc_1.save_trim();
	g.rc_2.save_trim();
	g.rc_3.save_trim();
	g.rc_4.save_trim();
	g.rc_5.save_trim();
	g.rc_6.save_trim();
	g.rc_7.save_trim();
	g.rc_8.save_trim();
	
	// pressure sensor data saved by init_home
}

/********************************************************************************/


/********************************************************************************/

void save_EEPROM_alt_RTL(void)
{
	g.RTL_altitude.save();
	
}

void read_EEPROM_alt_RTL(void)
{
	g.RTL_altitude.load();
}

/********************************************************************************/

void read_EEPROM_waypoint_info(void)
{
	g.waypoint_total.load();
	g.waypoint_radius.load();
	g.loiter_radius.load();
}

void save_EEPROM_waypoint_info(void)
{
	g.waypoint_total.save();
	g.waypoint_radius.save();
	g.loiter_radius.save();
}

/********************************************************************************/

void read_EEPROM_nav(void)
{
	g.crosstrack_gain.load();
	g.crosstrack_entry_angle.load();
	g.pitch_max.load();
}

void save_EEPROM_nav(void)
{
	g.crosstrack_gain.save();
 	g.crosstrack_entry_angle.save();
	g.pitch_max.save();
}

/********************************************************************************/

void read_EEPROM_PID(void)
{
	g.pid_acro_rate_roll.load_gains();
	g.pid_acro_rate_pitch.load_gains();
	g.pid_acro_rate_yaw.load_gains();
	
	g.pid_stabilize_roll.load_gains();
	g.pid_stabilize_pitch.load_gains();
	g.pid_yaw.load_gains();
	
	g.pid_nav_lon.load_gains();
	g.pid_nav_lat.load_gains();
	g.pid_baro_throttle.load_gains();
	g.pid_sonar_throttle.load_gains();
	
	// roll pitch
	g.stabilize_dampener.load();

	// yaw
	g.hold_yaw_dampener.load();
 	init_pids();
}

void save_EEPROM_PID(void)
{
	g.pid_acro_rate_roll.save_gains(); 
	g.pid_acro_rate_pitch.save_gains();
	g.pid_acro_rate_yaw.save_gains();
	
	g.pid_stabilize_roll.save_gains();
	g.pid_stabilize_pitch.save_gains();
	g.pid_yaw.save_gains();
	
	g.pid_nav_lon.save_gains();
	g.pid_nav_lat.save_gains();
	g.pid_baro_throttle.save_gains();
	g.pid_sonar_throttle.save_gains();

	// roll pitch
	g.stabilize_dampener.save();
	// yaw
	g.hold_yaw_dampener.save();
}

/********************************************************************************/

void save_EEPROM_frame(void)
{
	g.frame_type.save();
}

void read_EEPROM_frame(void)
{
	g.frame_type.load();
}

/********************************************************************************/

void save_EEPROM_throttle_cruise (void)
{
	g.throttle_cruise.save();
}

void read_EEPROM_throttle_cruise(void)
{
	g.throttle_cruise.load();
}

/********************************************************************************/

void save_EEPROM_current(void)
{
	g.current_enabled.save();
	g.milliamp_hours.save();
}

void read_EEPROM_current(void)
{
	g.current_enabled.load();
	g.milliamp_hours.load();
}

/********************************************************************************/
/*
void save_EEPROM_mag_offset(void)
{
	write_EE_compressed_float(mag_offset_x, 	EE_MAG_X, 2);
	write_EE_compressed_float(mag_offset_y, 	EE_MAG_Y, 2);
	write_EE_compressed_float(mag_offset_z, 	EE_MAG_Z, 2);
}

void read_EEPROM_compass_offset(void)
{
	mag_offset_x 	= read_EE_compressed_float(EE_MAG_X, 2);
	mag_offset_y 	= read_EE_compressed_float(EE_MAG_Y, 2);
	mag_offset_z 	= read_EE_compressed_float(EE_MAG_Z, 2);
}
*/
/********************************************************************************/

void read_EEPROM_compass(void)
{
	g.compass_enabled.load();
}

void save_EEPROM_mag(void)
{
	g.compass_enabled.save();
}

/********************************************************************************/

void save_command_index(void)
{
	g.command_must_index.save();
}

void read_command_index(void)
{
	g.command_must_index.load();
}

/********************************************************************************/

void save_EEPROM_pressure(void)
{
	g.ground_pressure.save();
	g.ground_temperature.save();

}

void read_EEPROM_pressure(void)
{
	g.ground_pressure.load();
	g.ground_temperature.load();	

	// to prime the filter
	abs_pressure	 	= g.ground_pressure;
}

/********************************************************************************/

void read_EEPROM_radio(void)
{
	g.rc_1.load_eeprom();
	g.rc_2.load_eeprom();
	g.rc_3.load_eeprom();
	g.rc_4.load_eeprom();
	g.rc_5.load_eeprom();
	g.rc_6.load_eeprom();
	g.rc_7.load_eeprom();
	g.rc_8.load_eeprom();
}

void save_EEPROM_radio(void)
{	
	g.rc_1.save_eeprom();
	g.rc_2.save_eeprom();
	g.rc_3.save_eeprom();
	g.rc_4.save_eeprom();
	g.rc_5.save_eeprom();
	g.rc_6.save_eeprom();
	g.rc_7.save_eeprom();
	g.rc_8.save_eeprom();
}

/********************************************************************************/
// configs are the basics
void read_EEPROM_throttle(void)
{
	g.throttle_min.load();
	g.throttle_max.load();
	g.throttle_cruise.load();
	g.throttle_failsafe_enabled.load();
	g.throttle_failsafe_action.load();
	g.throttle_failsafe_value.load();
}

void save_EEPROM_throttle(void)
{
	g.throttle_min.load();
	g.throttle_max.load();
	g.throttle_cruise.save();
	g.throttle_failsafe_enabled.load();
	g.throttle_failsafe_action.load();
	g.throttle_failsafe_value.load();
}

/********************************************************************************/

void read_EEPROM_logs(void)
{
	g.log_bitmask.load();
}

void save_EEPROM_logs(void)
{
	g.log_bitmask.save();
}

/********************************************************************************/

void read_EEPROM_flight_modes(void)
{
	g.flight_modes.load();
}

void save_EEPROM_flight_modes(void)
{
	g.flight_modes.save();
	
}

/********************************************************************************/
/*
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
*/
/********************************************************************************/
/*
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
*/