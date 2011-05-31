// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

// Functions called from the top-level menu
extern int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
extern int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
extern int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	main_menu_help(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Commands:\n"
						 "  logs\n"
						 "  setup\n"
						 "  test\n"
						 "\n"
						 "Move the slide switch and reset to FLY.\n"
						 "\n"));
	return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
	{"logs",		process_logs},
	{"setup",		setup_mode},
	{"test",		test_mode},
	{"help",		main_menu_help}
};

// Create the top-level menu object.
MENU(main_menu, "AC 2.0.20 Beta", main_menu_commands);

void init_ardupilot()
{
	// Console serial port
	//
	// The console port buffers are defined to be sufficiently large to support
	// the console's use as a logging device, optionally as the GPS port when
	// GPS_PROTOCOL_IMU is selected, and as the telemetry port.
	//
	// XXX This could be optimised to reduce the buffer sizes in the cases
	// where they are not otherwise required.
	//
	Serial.begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	// Not used if the IMU/X-Plane GPS is in use.
	//
	// XXX currently the EM406 (SiRF receiver) is nominally configured
	// at 57600, however it's not been supported to date.  We should
	// probably standardise on 38400.
	//
	// XXX the 128 byte receive buffer may be too small for NMEA, depending
	// on the message set configured.
	//
	#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
	Serial1.begin(38400, 128, 16);
	#endif

	// Telemetry port.
	//
	// Not used if telemetry is going to the console.
	//
	// XXX for unidirectional protocols, we could (should) minimize
	// the receive buffer, and the transmit buffer could also be
	// shrunk for protocols that don't send large messages.
	//
	Serial3.begin(SERIAL3_BAUD, 128, 128);

	Serial.printf_P(PSTR("\n\nInit ACM"
						 "\n\nRAM: %lu\n"),
						 freeRAM());


	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	report_version();

	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {
		//Serial.printf_P(PSTR("\n\nForcing complete parameter reset..."));

		/*Serial.printf_P(PSTR("\n\nEEPROM format version  %d not compatible with this firmware (requires %d)"
		                     "\n\nForcing complete parameter reset..."),
		                     g.format_version.get(),
		                     Parameters::k_format_version);
		*/

		// erase all parameters
		AP_Var::erase_all();

		// save the new format version
		g.format_version.set_and_save(Parameters::k_format_version);

		Serial.printf_P(PSTR("Please Run Setup...\n"));
		while (true) {
			delay(1000);
			if(motor_light){
				digitalWrite(A_LED_PIN, HIGH);
				digitalWrite(B_LED_PIN, HIGH);
				digitalWrite(C_LED_PIN, HIGH);
			}else{
				digitalWrite(A_LED_PIN, LOW);
				digitalWrite(B_LED_PIN, LOW);
				digitalWrite(C_LED_PIN, LOW);
			}
			motor_light = !motor_light;
		}

	}else{
		// unsigned long before = micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Var::load_all();

	  //  Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
	  //  Serial.printf_P(PSTR("using %u bytes of memory\n"), AP_Var::get_memory_use());
	}


	#ifdef RADIO_OVERRIDE_DEFAULTS
	{
		int16_t rc_override[8] = RADIO_OVERRIDE_DEFAULTS;
		APM_RC.setHIL(rc_override);
	}
	#endif

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	#if CAMERA_STABILIZER == ENABLED
		init_camera();
	#endif

	#if HIL_MODE != HIL_MODE_ATTITUDE
	adc.Init();	 		// APM ADC library initialization
	barometer.Init();	// APM Abs Pressure sensor initialization
	#endif

	// Do GPS init
	//g_gps = &GPS;
	g_gps = &g_gps_driver;
	g_gps->init();			// GPS Initialization

	// init the GCS
	#if GCS_PORT == 3
		gcs.init(&Serial3);
	#else
		gcs.init(&Serial);
	#endif

	// init the HIL
#if HIL_MODE != HIL_MODE_DISABLED

  #if HIL_PORT == 3
	hil.init(&Serial3);
  #elif HIL_PORT == 1
	hil.init(&Serial1);
  #else
	hil.init(&Serial);
  #endif
#endif

//  We may have a hil object instantiated just for mission planning
#if HIL_MODE == HIL_MODE_DISABLED && HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && HIL_PORT == 0
	hil.init(&Serial);
#endif

	if(g.compass_enabled)
		init_compass();

	#if HIL_MODE != HIL_MODE_ATTITUDE
	if(g.sonar_enabled){
		sonar.init(SONAR_PIN, &adc);
	}
	#endif

	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused
	DDRL |= B00000100;					// Set Port L, pin 2 to output for the relay

	#if MOTOR_LEDS == 1
		pinMode(FR_LED, OUTPUT);			// GPS status LED
		pinMode(RE_LED, OUTPUT);			// GPS status LED
		pinMode(RI_LED, OUTPUT);			// GPS status LED
		pinMode(LE_LED, OUTPUT);			// GPS status LED
	#endif

	// Logging:
	// --------
	DataFlash.Init(); 	// DataFlash log initialization
	// setup the log bitmask
	if (g.log_bitmask & MASK_LOG_SET_DEFAULTS)
		default_log_bitmask();

	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
	if (digitalRead(SLIDE_SWITCH_PIN) == 0) {
		digitalWrite(A_LED_PIN,HIGH);		// turn on setup-mode LED
		Serial.printf_P(PSTR("\n"
							 "Entering interactive setup mode...\n"
							 "\n"
							 "Type 'help' to list commands, 'exit' to leave a submenu.\n"
							 "Visit the 'setup' menu for first-time configuration.\n\n"));
		for (;;) {
			//Serial.println_P(PSTR("\nMove the slide switch and reset to FLY.\n"));
			main_menu.run();
		}
	}else{
		if(g.esc_calibrate == 1){
			init_esc();
		}
	}

	// All of the Gyro calibrations
	// ----------------------------
	startup_ground();

	// set the correct flight mode
	// ---------------------------
	//reset_control_switch();

	// init the Yaw Hold output
	clear_yaw_control();

	// Logging:
	// --------
	if(g.log_bitmask != 0){
		//	TODO - Here we will check  on the length of the last log
		//  We don't want to create a bunch of little logs due to powering on and off
		start_new_log();
	}

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void startup_ground(void)
{
	gcs.send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

	#if(GROUND_START_DELAY > 0)
		//gcs.send_text_P(SEVERITY_LOW, PSTR(" With Delay"));
		delay(GROUND_START_DELAY * 1000);
	#endif

	// Output waypoints for confirmation
	// --------------------------------
	for(int i = 1; i < g.waypoint_total + 1; i++) {
		gcs.send_message(MSG_COMMAND_LIST, i);
	}

	#if HIL_MODE != HIL_MODE_ATTITUDE
		// Warm up and read Gyro offsets
		// -----------------------------
		imu.init_gyro();
		report_imu();
	#endif

	#if HIL_MODE != HIL_MODE_ATTITUDE
		// read Baro pressure at ground
		//-----------------------------
		init_barometer();
	#endif

	// initialize commands
	// -------------------
	init_commands();

    GPS_enabled = false;

    // Read in the GPS
	for (byte counter = 0; ; counter++) {
		g_gps->update();
		if (g_gps->status() != 0){
			GPS_enabled = true;
			break;
		}

		if (counter >= 2) {
			GPS_enabled = false;
			break;
	    }
	}

	report_gps();
	SendDebug("\nReady to FLY ");
	//gcs.send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to FLY."));
}

void set_mode(byte mode)
{
	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}

	control_mode = mode;
	control_mode = constrain(control_mode, 0, NUM_MODES - 1);

	// used to stop fly_aways
	if(g.rc_3.control_in == 0){ // throttle is 0
		// we are on the ground is this is true
		// disarm motors for Auto
		motor_auto_armed = false;
	}

	//send_text_P(SEVERITY_LOW,PSTR("control mode"));
	//Serial.printf("set mode: %d\n",control_mode);
	Serial.println(flight_mode_strings[control_mode]);

	switch(control_mode)
	{
		case ACRO:
			break;

		case SIMPLE:
		case STABILIZE:
			do_loiter_at_location();
			g.pid_baro_throttle.reset_I();
			g.pid_sonar_throttle.reset_I();
			break;

		case ALT_HOLD:
			init_throttle_cruise();
			do_loiter_at_location();
			break;

		case AUTO:
			init_throttle_cruise();
			init_auto();
			break;

		case LOITER:
			init_throttle_cruise();
			do_loiter_at_location();
			break;

		case RTL:
			//init_throttle_cruise();
			do_RTL();
			break;

		default:
			break;
	}

	Log_Write_Mode(control_mode);

	// output control mode to the ground station
	gcs.send_message(MSG_MODE_CHANGE);
}

void set_failsafe(boolean mode)
{
	// only act on changes
	// -------------------
	if(failsafe != mode){

		// store the value so we don't trip the gate twice
		// -----------------------------------------------
		failsafe = mode;

		if (failsafe == false){
			// We've regained radio contact
			// ----------------------------
			failsafe_off_event();

		}else{
			// We've lost radio contact
			// ------------------------
			failsafe_on_event();

		}
	}
}

#if MOTOR_LEDS == 1
void update_motor_leds(void)
{
	// blink rear
	static bool blink;

	if (blink){
		blink = false;
		digitalWrite(RE_LED, LOW);

	}else{
		blink = true;
		digitalWrite(RE_LED, HIGH);
	}
}
#endif

void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	switch (g_gps->status()){

		case(2):
			digitalWrite(C_LED_PIN, HIGH);  //Turn LED C on when gps has valid fix.
			break;

		case(1):
			if (g_gps->valid_read == true){
				GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
				if (GPS_light){
					digitalWrite(C_LED_PIN, LOW);
				}else{
					digitalWrite(C_LED_PIN, HIGH);
				}
				g_gps->valid_read = false;
			}
			break;

		default:
			digitalWrite(C_LED_PIN, LOW);
			break;
	}
}

void update_motor_light(void)
{
	if(motor_armed == false){
		motor_light = !motor_light;

		// blink
		if(motor_light){
			digitalWrite(A_LED_PIN, HIGH);
		}else{
			digitalWrite(A_LED_PIN, LOW);
		}
	}else{
		if(!motor_light){
			motor_light = true;
			digitalWrite(A_LED_PIN, HIGH);
		}
	}
}

void update_esc_light()
{
	static byte step;

	if (step++ == 3)
		step = 0;

	switch(step)
	{
		case 0:
			digitalWrite(C_LED_PIN, LOW);
			digitalWrite(A_LED_PIN, HIGH);
			break;

		case 1:
			digitalWrite(A_LED_PIN, LOW);
			digitalWrite(B_LED_PIN, HIGH);
			break;

		case 2:
			digitalWrite(B_LED_PIN, LOW);
			digitalWrite(C_LED_PIN, HIGH);
			break;
	}
}


void resetPerfData(void) {
	mainLoop_count 		= 0;
	G_Dt_max 			= 0;
	gps_fix_count 		= 0;
	perf_mon_timer 		= millis();
}

void
init_compass()
{
	dcm.set_compass(&compass);
	bool junkbool 		= compass.init();
	compass.set_orientation(MAG_ORIENTATION);						// set compass's orientation on aircraft
	Vector3f junkvector = compass.get_offsets();					// load offsets to account for airframe magnetic interference
}


/* This function gets the current value of the heap and stack pointers.
* The stack pointer starts at the top of RAM and grows downwards. The heap pointer
* starts just above the static variables etc. and grows upwards. SP should always
* be larger than HP or you'll be in big trouble! The smaller the gap, the more
* careful you need to be. Julian Gall 6 - Feb - 2009.
*/
unsigned long freeRAM() {
	uint8_t * heapptr, * stackptr;
	stackptr = (uint8_t *)malloc(4); // use stackptr temporarily
	heapptr = stackptr; // save value of heap pointer
	free(stackptr); // free up the memory again (sets stackptr to 0)
	stackptr = (uint8_t *)(SP); // save value of stack pointer
	return stackptr - heapptr;
}

void
init_simple_bearing()
{
	initial_simple_bearing = dcm.yaw_sensor;
}

void
init_throttle_cruise()
{
	//if(set_throttle_cruise_flag == false){
		if(g.rc_3.control_in > 200){
			g.throttle_cruise.set_and_save(g.rc_3.control_in);
		}
	//}
}

