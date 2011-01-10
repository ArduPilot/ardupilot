// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Functions called from the setup menu
static int8_t	setup_radio				(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_motors			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_accel				(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_factory			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_erase				(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_flightmodes		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_pid				(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_frame				(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_compass			(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_mag_offset		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_show				(uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu 
const struct Menu::command setup_menu_commands[] PROGMEM = {
	// command			function called
	// =======        	===============
	{"erase", 			setup_erase},
	{"reset", 			setup_factory},
	{"pid",				setup_pid},
	{"radio",			setup_radio},
	{"motors",			setup_motors},
	{"level",			setup_accel},
	{"modes",			setup_flightmodes},
	{"frame",			setup_frame},
	{"compass",			setup_compass},
	{"mag_offset",		setup_mag_offset},
	{"declination",		setup_declination},
	{"show",			setup_show}
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
	print_blanks(10);
	Serial.printf_P(PSTR("Radio\n"));
	print_divider();
	// radio
	read_EEPROM_radio();	
	print_radio_values();
	
	
	// frame
	print_blanks(2);
	Serial.printf_P(PSTR("Frame\n"));
	print_divider();

	read_EEPROM_frame();
	
	if(frame_type == X_FRAME)
		Serial.printf_P(PSTR("X "));
	else if(frame_type == PLUS_FRAME)
		Serial.printf_P(PSTR("Plus "));
	Serial.printf_P(PSTR("(%d)\n"), (int)frame_type);
		
	
	print_blanks(2);
	Serial.printf_P(PSTR("Gains\n"));
	print_divider();

	read_EEPROM_PID();
	// Acro
	Serial.printf_P(PSTR("Acro:\nroll:\n"));
	print_PID(&pid_acro_rate_roll);
	Serial.printf_P(PSTR("pitch:\n"));
	print_PID(&pid_acro_rate_pitch);
	Serial.printf_P(PSTR("yaw:\n"));
	print_PID(&pid_acro_rate_yaw);

	// Stabilize
	Serial.printf_P(PSTR("\nStabilize:\nroll:\n"));
	print_PID(&pid_stabilize_roll);
	Serial.printf_P(PSTR("pitch:\n"));
	print_PID(&pid_stabilize_pitch);
	Serial.printf_P(PSTR("yaw:\n"));
	print_PID(&pid_yaw);
	
	Serial.printf_P(PSTR("Stabilize dampener: %4.3f\n"), stabilize_dampener);	
	Serial.printf_P(PSTR("Yaw Dampener: %4.3f\n\n"), hold_yaw_dampener);
	
	// Nav
	Serial.printf_P(PSTR("Nav:\npitch:\n"));
	print_PID(&pid_nav);
	Serial.printf_P(PSTR("baro throttle:\n"));
	print_PID(&pid_baro_throttle);
	Serial.printf_P(PSTR("sonar throttle:\n"));
	print_PID(&pid_sonar_throttle);
	Serial.println(" ");
	
	print_blanks(2);
	Serial.printf_P(PSTR("User Configs\n"));
	print_divider();
	// Crosstrack
	read_EEPROM_nav();
	Serial.printf_P(PSTR("XTRACK: %4.2f\n"), x_track_gain);
	Serial.printf_P(PSTR("XTRACK angle: %d\n"), x_track_angle);
	Serial.printf_P(PSTR("PITCH_MAX: %d\n"), pitch_max);
	// User Configs
	read_EEPROM_configs();
	Serial.printf_P(PSTR("throttle_min: %d\n"), 	throttle_min);
	Serial.printf_P(PSTR("throttle_max: %d\n"), 	throttle_max);
	Serial.printf_P(PSTR("throttle_cruise: %d\n"), 	throttle_cruise);
	Serial.printf_P(PSTR("throttle_failsafe_enabled: %d\n"), throttle_failsafe_enabled);
	Serial.printf_P(PSTR("throttle_failsafe_value: %d\n"), throttle_failsafe_value);
	Serial.printf_P(PSTR("log_bitmask: %d\n"), log_bitmask);

	print_blanks(2);
	Serial.printf_P(PSTR("IMU\n"));
	print_divider();
	imu.print_gyro_offsets();
	imu.print_accel_offsets();
	
	print_blanks(2);
	Serial.printf_P(PSTR("Compass\n"));
	print_divider();

	if(compass_enabled)
		Serial.printf_P(PSTR("en"));
	else
		Serial.printf_P(PSTR("dis"));
	Serial.printf_P(PSTR("abled\n\n"));


	// mag declination
	read_EEPROM_mag_declination();
	Serial.printf_P(PSTR("Mag Delination: "));
	Serial.println(mag_declination,2);
	
	// mag offsets
	Serial.printf_P(PSTR("Mag offsets: "));
	Serial.print(mag_offset_x, 2);
	Serial.printf_P(PSTR(", "));
	Serial.print(mag_offset_y, 2);
	Serial.printf_P(PSTR(", "));
	Serial.println(mag_offset_z, 2);
	
	return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
	/*
	saves:
		save_EEPROM_waypoint_info();
		save_EEPROM_nav();
		save_EEPROM_flight_modes();
		save_EEPROM_configs();
	*/
	
	uint8_t		i;
	int			c;

	Serial.printf_P(PSTR("\nType 'Y' and hit Enter to perform factory reset, any other key to abort:\n"));

	do {
		c = Serial.read();
	} while (-1 == c);
	
	if (('y' != c) && ('Y' != c))
		return(-1);

	//Serial.printf_P(PSTR("\nFACTORY RESET\n\n"));
	
	//zero_eeprom();
	setup_pid(0 ,NULL);
	
	wp_radius 					= 4; //TODO: Replace this quick fix with a real way to define wp_radius
	loiter_radius 				= 30; //TODO: Replace this quick fix with a real way to define loiter_radius
	save_EEPROM_waypoint_info();
	
	// nav control
	x_track_gain 				= XTRACK_GAIN * 100;
	x_track_angle 				= XTRACK_ENTRY_ANGLE * 100;
	pitch_max 					= PITCH_MAX * 100;
	save_EEPROM_nav();
	
	// alt hold
	alt_to_hold = -1;
	save_EEPROM_alt_RTL();


	// default to a + configuration	
	frame_type = PLUS_FRAME;
	save_EEPROM_frame();

	flight_modes[0] 			= FLIGHT_MODE_1;
	flight_modes[1] 			= FLIGHT_MODE_2;
	flight_modes[2] 			= FLIGHT_MODE_3;
	flight_modes[3] 			= FLIGHT_MODE_4;
	flight_modes[4] 			= FLIGHT_MODE_5;
	flight_modes[5] 			= FLIGHT_MODE_6;
	save_EEPROM_flight_modes();

	// user configs
	throttle_min				= THROTTLE_MIN;
	throttle_max				= THROTTLE_MAX;
	throttle_cruise				= THROTTLE_CRUISE;
	throttle_failsafe_enabled	= THROTTLE_FAILSAFE;
	throttle_failsafe_action	= THROTTLE_FAILSAFE_ACTION;
	throttle_failsafe_value		= THROTTLE_FS_VALUE;
	
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
	save_EEPROM_configs();
	print_done();
	
	// finish
	// ------
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
	
	if(rc_1.radio_in < 500){
		while(1){
			Serial.printf_P(PSTR("\nNo radio; Check connectors."));
			delay(1000);
			// stop here
		}
	}
	
	rc_1.radio_min = rc_1.radio_in;
	rc_2.radio_min = rc_2.radio_in;
	rc_3.radio_min = rc_3.radio_in;
	rc_4.radio_min = rc_4.radio_in;
	rc_5.radio_min = rc_5.radio_in;
	rc_6.radio_min = rc_6.radio_in;
	rc_7.radio_min = rc_7.radio_in;
	rc_8.radio_min = rc_8.radio_in;

	rc_1.radio_max = rc_1.radio_in;
	rc_2.radio_max = rc_2.radio_in;
	rc_3.radio_max = rc_3.radio_in;
	rc_4.radio_max = rc_4.radio_in;
	rc_5.radio_max = rc_5.radio_in;
	rc_6.radio_max = rc_6.radio_in;
	rc_7.radio_max = rc_7.radio_in;
	rc_8.radio_max = rc_8.radio_in;

	rc_1.radio_trim = rc_1.radio_in;
	rc_2.radio_trim = rc_2.radio_in;
	rc_4.radio_trim = rc_4.radio_in;
	// 3 is not trimed
	rc_5.radio_trim = 1500;
	rc_6.radio_trim = 1500;
	rc_7.radio_trim = 1500;
	rc_8.radio_trim = 1500;
			
	Serial.printf_P(PSTR("\nMove all controls to each extreme. Hit Enter to save: "));
	while(1){
		
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		rc_1.update_min_max();
		rc_2.update_min_max();
		rc_3.update_min_max();
		rc_4.update_min_max();
		rc_5.update_min_max();
		rc_6.update_min_max();
		rc_7.update_min_max();
		rc_8.update_min_max();
		
		if(Serial.available() > 0){
			//rc_3.radio_max += 250;
			Serial.flush();
			
			save_EEPROM_radio();
			//delay(100);
			// double checking
			//read_EEPROM_radio();
			//print_radio_values();
			print_done();
			break;
		}
	}
	return(0);
}

static int8_t
setup_motors(uint8_t argc, const Menu::arg *argv)
{
	init_rc_in();
	// read the radio to set trims
	// ---------------------------
	trim_radio();

	print_hit_enter();
	delay(1000);

	
	int out_min = rc_3.radio_min + 70;
	
	if(frame_type == PLUS_FRAME){
		Serial.printf_P(PSTR("PLUS"));

	}else if(frame_type == X_FRAME){
		Serial.printf_P(PSTR("X"));
	
	}else if(frame_type == TRI_FRAME){
		Serial.printf_P(PSTR("TRI"));
	}
	
	Serial.printf_P(PSTR(" Frame\n"));
	
	while(1){
		delay(20);
		read_radio();
		motor_out[LEFT]		= rc_3.radio_min;
		motor_out[BACK] 	= rc_3.radio_min;
		motor_out[FRONT]	= rc_3.radio_min;
		motor_out[RIGHT]	= rc_3.radio_min;

	
		
		if(frame_type == PLUS_FRAME){
			if(rc_1.control_in > 0){
				motor_out[RIGHT] 	= out_min;
				Serial.println("0");
				
			}else if(rc_1.control_in < 0){
				motor_out[LEFT]		= out_min;
				Serial.println("1");
			}
			
			if(rc_2.control_in > 0){
				motor_out[BACK] 	= out_min;
				Serial.println("3");
				
			}else if(rc_2.control_in < 0){
				motor_out[FRONT]	= out_min;
				Serial.println("2");
			}

		}else if(frame_type == PLUS_FRAME){

			// lower right
			if((rc_1.control_in > 0) 		&& (rc_2.control_in > 0)){
				motor_out[BACK] 	= out_min;
				Serial.println("3");
			// lower left
			}else if((rc_1.control_in < 0) 	&& (rc_2.control_in > 0)){
				motor_out[LEFT]		= out_min;
				Serial.println("1");

			// upper left
			}else if((rc_1.control_in < 0) 	&& (rc_2.control_in < 0)){
				motor_out[FRONT]	= out_min;
				Serial.println("2");

			// upper right
			}else if((rc_1.control_in > 0) 	&& (rc_2.control_in < 0)){
				motor_out[RIGHT]	= out_min;
				Serial.println("0");
			}
			
		}else if(frame_type == TRI_FRAME){

			if(rc_1.control_in > 0){
				motor_out[RIGHT] 	= out_min;
				
			}else if(rc_1.control_in < 0){
				motor_out[LEFT]		= out_min;
			}
			
			if(rc_2.control_in > 0){
				motor_out[BACK] 	= out_min;	
			}

			if(rc_4.control_in > 0){
				rc_4.servo_out	= 2000;
				
			}else if(rc_4.control_in < 0){
				rc_4.servo_out	= -2000;
			}
			
			rc_4.calc_pwm();
			motor_out[FRONT] 	= rc_4.radio_out;
		}
		
		if(rc_3.control_in > 0){
			APM_RC.OutputCh(CH_1, rc_3.radio_in);
			APM_RC.OutputCh(CH_2, rc_3.radio_in);
			APM_RC.OutputCh(CH_3, rc_3.radio_in);
			APM_RC.OutputCh(CH_4, rc_3.radio_in);
		}else{
			APM_RC.OutputCh(CH_1, motor_out[RIGHT]);
			APM_RC.OutputCh(CH_2, motor_out[LEFT]);
			APM_RC.OutputCh(CH_3, motor_out[FRONT]);
			APM_RC.OutputCh(CH_4, motor_out[BACK]);
		}
		
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
setup_accel(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("\nHold ArduCopter completely still and level.\n"));

	/*
	imu.init_gyro();
	print_gyro();
	imu.load_gyro_eeprom();
	print_gyro();
	*/
	
	imu.init_accel();
	imu.print_accel_offsets();
	//imu.load_accel_eeprom();
	//print_accel();

	return(0);
}

static int8_t
setup_accel_flat(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("\nClear Accel offsets.\n"));
	imu.zero_accel();
	imu.print_accel_offsets();
	return(0);
}

static int8_t
setup_pid(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("\nSetting default PID gains\n"));
	// acro, angular rate
	pid_acro_rate_roll.kP(ACRO_RATE_ROLL_P);
	pid_acro_rate_roll.kI(ACRO_RATE_ROLL_I);
	pid_acro_rate_roll.kD(ACRO_RATE_ROLL_D);
	pid_acro_rate_roll.imax(ACRO_RATE_ROLL_IMAX * 100);
	
	pid_acro_rate_pitch.kP(ACRO_RATE_PITCH_P);
	pid_acro_rate_pitch.kI(ACRO_RATE_PITCH_I);
	pid_acro_rate_pitch.kD(ACRO_RATE_PITCH_D);
	pid_acro_rate_pitch.imax(ACRO_RATE_PITCH_IMAX * 100);

	pid_acro_rate_yaw.kP(ACRO_RATE_YAW_P);
	pid_acro_rate_yaw.kI(ACRO_RATE_YAW_I);
	pid_acro_rate_yaw.kD(ACRO_RATE_YAW_D);
	pid_acro_rate_yaw.imax(ACRO_RATE_YAW_IMAX * 100);


	// stabilize, angle error
	pid_stabilize_roll.kP(STABILIZE_ROLL_P);
	pid_stabilize_roll.kI(STABILIZE_ROLL_I);
	pid_stabilize_roll.kD(STABILIZE_ROLL_D);
	pid_stabilize_roll.imax(STABILIZE_ROLL_IMAX * 100);
		
	pid_stabilize_pitch.kP(STABILIZE_PITCH_P);
	pid_stabilize_pitch.kI(STABILIZE_PITCH_I);
	pid_stabilize_pitch.kD(STABILIZE_PITCH_D);
	pid_stabilize_pitch.imax(STABILIZE_PITCH_IMAX * 100);

	// YAW hold
	pid_yaw.kP(YAW_P);
	pid_yaw.kI(YAW_I);
	pid_yaw.kD(YAW_D);
	pid_yaw.imax(YAW_IMAX * 100);


	// custom dampeners
	// roll pitch
	stabilize_dampener 	= STABILIZE_DAMPENER;
	
	//yaw
	hold_yaw_dampener	= HOLD_YAW_DAMPENER;


	// navigation
	pid_nav.kP(NAV_P);
	pid_nav.kI(NAV_I);
	pid_nav.kD(NAV_D);
	pid_nav.imax(NAV_IMAX * 100);

	pid_baro_throttle.kP(THROTTLE_BARO_P);
	pid_baro_throttle.kI(THROTTLE_BARO_I);
	pid_baro_throttle.kD(THROTTLE_BARO_D);
	pid_baro_throttle.imax(THROTTLE_BARO_IMAX * 100);

	pid_sonar_throttle.kP(THROTTLE_SONAR_P);
	pid_sonar_throttle.kI(THROTTLE_SONAR_I);
	pid_sonar_throttle.kD(THROTTLE_SONAR_D);
	pid_sonar_throttle.imax(THROTTLE_SONAR_IMAX * 100);
	
	save_EEPROM_PID();
	print_done();
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
		
			mode = flight_modes[switchPosition];
			mode = constrain(mode, 0, NUM_MODES-1);

			// update the user
			print_switch(switchPosition, mode);

			// Remember switch position
			oldSwitchPosition = switchPosition;			
		}

		// look for stick input
		if (radio_input_switch() == true){
			mode++;	
			if(mode >= NUM_MODES)
				mode = 0;

			// save new mode
			flight_modes[switchPosition] = mode;

			// print new mode
			print_switch(switchPosition, mode);
		}
		
		// escape hatch
		if(Serial.available() > 0){
			save_EEPROM_flight_modes();
			print_done();
			return (0);
		}
	}
}

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
	mag_declination = argv[1].f;
	save_EEPROM_mag_declination();
	read_EEPROM_mag_declination();

	Serial.printf_P(PSTR("\nsaved: "));
	Serial.println(argv[1].f, 2);
}

static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
	zero_eeprom();
	return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Compass "));
	if (!strcmp_P(argv[1].str, PSTR("on"))) {
		Serial.printf_P(PSTR("Enabled\n"));
		compass_enabled = true;
		init_compass();

		save_EEPROM_mag();
		print_done();
	
	} else if (!strcmp_P(argv[1].str, PSTR("off"))) {
		Serial.printf_P(PSTR("Disabled\n"));
		compass_enabled = false;

		save_EEPROM_mag();
		print_done();
	
	} else {
		if (compass_enabled) {
			Serial.printf_P(PSTR("en"));
		} else {
			Serial.printf_P(PSTR("dis"));
		}
		Serial.printf_P(PSTR("abled\n\n"));
		
		Serial.printf_P(PSTR("\nUsage:\nEnabled =>compass 1\nDisabled =>compass 0\n\n"));
	}
	return 0;
}




static int8_t
setup_frame(uint8_t argc, const Menu::arg *argv)
{
	if(argv[1].i < 1){
		Serial.printf_P(PSTR("\nUsage:\nPlus frame =>frame 1\nX frame =>frame 2\nTRI frame =>frame 3\n\n"));
		return 0;
	}
	print_done();

	if(argv[1].i == 1){
		Serial.printf_P(PSTR("Plus "));
		frame_type = PLUS_FRAME;

	}else if(argv[1].i == 2){
		Serial.printf_P(PSTR("X "));
		frame_type = X_FRAME;
	
	}else if(argv[1].i == 3){
		Serial.printf_P(PSTR("Tri "));
		frame_type = TRI_FRAME;
	}
	
	Serial.printf_P(PSTR("frame\n\n"));
	save_EEPROM_frame();	
	return 0;
}


static int8_t
setup_mag_offset(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("\nRotate/Pitch/Roll your ArduCopter until the offset variables stop changing.\n"));
	print_hit_enter();
	Serial.printf_P(PSTR("Starting in 3 secs.\n"));
	delay(3000);


	compass.init();	 // Initialization
	compass.set_orientation(MAGORIENTATION);		// set compass's orientation on aircraft
	compass.set_offsets(0, 0, 0);					// set offsets to account for surrounding interference
	compass.set_declination(ToRad(DECLINATION));	// set local difference between magnetic north and true north
	//int counter = 0; 
	float _min[3], _max[3], _offset[3];

	while(1){
		static float min[3], _max[3], offset[3];
		if (millis() - fast_loopTimer > 100) {
			deltaMiliSeconds 	= millis() - fast_loopTimer;
			fast_loopTimer		= millis();
			G_Dt 				= (float)deltaMiliSeconds / 1000.f;


			compass.read();
			compass.calculate(0, 0);	// roll = 0, pitch = 0 for this example

			// capture min
			if(compass.mag_x < _min[0]) _min[0] = compass.mag_x;
			if(compass.mag_y < _min[1]) _min[1] = compass.mag_y;
			if(compass.mag_z < _min[2]) _min[2] = compass.mag_z;

			// capture max
			if(compass.mag_x > _max[0]) _max[0] = compass.mag_x;
			if(compass.mag_y > _max[1]) _max[1] = compass.mag_y;
			if(compass.mag_z > _max[2]) _max[2] = compass.mag_z;

			// calculate offsets
			offset[0] = -(_max[0] + _min[0]) / 2;
			offset[1] = -(_max[1] + _min[1]) / 2;
			offset[2] = -(_max[2] + _min[2]) / 2;
			
			// display all to user			
			Serial.printf_P(PSTR("Heading: "));
			Serial.print(ToDeg(compass.heading));
			Serial.print("  \t(");
			Serial.print(compass.mag_x);
			Serial.print(",");
			Serial.print(compass.mag_y);
			Serial.print(",");    
			Serial.print(compass.mag_z);
			Serial.print(")\t offsets(");
			Serial.print(offset[0]);
			Serial.print(",");
			Serial.print(offset[1]);
			Serial.print(",");
			Serial.print(offset[2]);
			Serial.println(")");
						
			if(Serial.available() > 0){
				
				mag_offset_x = offset[0];
				mag_offset_y = offset[1];
				mag_offset_z = offset[2];
			
				save_EEPROM_mag_offset();
				
				// set offsets to account for surrounding interference
				compass.set_offsets(mag_offset_x, mag_offset_y, mag_offset_z);
				
				print_done();
				break;
			}
		}
	}
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

void print_PID(PID * pid)
{
	Serial.printf_P(PSTR("P: %4.3f, I:%4.3f, D:%4.3f, IMAX:%d\n"), pid->kP(), pid->kI(), pid->kD(), (int)(pid->imax()/100));
}

void 
print_radio_values()
{
	Serial.printf_P(PSTR("CH1: %d | %d\n"), rc_1.radio_min, rc_1.radio_max);
	Serial.printf_P(PSTR("CH2: %d | %d\n"), rc_2.radio_min, rc_2.radio_max);
	Serial.printf_P(PSTR("CH3: %d | %d\n"), rc_3.radio_min, rc_3.radio_max);
	Serial.printf_P(PSTR("CH4: %d | %d\n"), rc_4.radio_min, rc_4.radio_max);
	Serial.printf_P(PSTR("CH5: %d | %d\n"), rc_5.radio_min, rc_5.radio_max);
	Serial.printf_P(PSTR("CH6: %d | %d\n"), rc_6.radio_min, rc_6.radio_max);
	Serial.printf_P(PSTR("CH7: %d | %d\n"), rc_7.radio_min, rc_7.radio_max);
	Serial.printf_P(PSTR("CH8: %d | %d\n"), rc_8.radio_min, rc_8.radio_max);
}

void
print_switch(byte p, byte m)
{
	Serial.printf_P(PSTR("Pos %d: "),p);
	Serial.println(flight_mode_strings[m]);
}

void
print_done()
{
	Serial.printf_P(PSTR("\nSaved Settings\n\n"));
}

void
print_blanks(int num)
{
	while(num > 0){
		num--;
		Serial.println("");
	}
}

void
print_divider(void)
{
	for (int i = 0; i < 40; i++) {
		Serial.print("-");
	}
	Serial.println("");
}


// for reading in vales for mode switch
boolean
radio_input_switch(void)
{
	static byte bouncer;
	
	if (abs(rc_1.radio_in - rc_1.radio_trim) > 200)
		bouncer = 10;

	if (bouncer > 0)
		bouncer--;

	if (bouncer == 1){
		return true;
	}else{
		return false;
	}
}


void zero_eeprom(void)
{
	byte b;
	Serial.printf_P(PSTR("\nErasing EEPROM\n"));
	for (int i = 0; i < EEPROM_MAX_ADDR; i++) {
		eeprom_write_byte((uint8_t *) i, b);
	}
	Serial.printf_P(PSTR("done\n"));
}
