// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
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
						 "  logs        log readback/setup mode\n"
						 "  setup       setup mode\n"
						 "  test        test mode\n"
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
MENU(main_menu, "ArduPilotMega", main_menu_commands);

void init_ardupilot()
{

	byte last_log_num;
	int last_log_start;
	int last_log_end;

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
#if GCS_PORT == 3
	Serial3.begin(SERIAL3_BAUD, 128, 128);
#endif

	Serial.printf_P(PSTR("\n\n"
						 "Init ArduCopterMega 1.0.3 Public Alpha\n\n"
#if TELEMETRY_PORT == 3
						 "Telemetry is on the xbee port\n"
#endif
						 "freeRAM: %d\n"),freeRAM());

	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {

		Serial.printf_P(PSTR("\n\nEEPROM format version  %d not compatible with this firmware (requires %d)"
		                     "\n\nForcing complete parameter reset..."),
		                     g.format_version.get(), Parameters::k_format_version);

		// erase all parameters
		AP_Var::erase_all();

		// save the new format version
		g.format_version.set_and_save(Parameters::k_format_version);

		Serial.println_P(PSTR("done."));
	} else {

	    // Load all auto-loaded EEPROM variables
	    AP_Var::load_all();
	}

	//read_EEPROM_startup(); // Read critical config information to start

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs
	init_camera();
	adc.Init();	 		// APM ADC library initialization
	APM_BMP085.Init();	// APM Abs Pressure sensor initialization
	DataFlash.Init(); 	// DataFlash log initialization

	g_gps = &GPS;
	g_gps->init();
	
	imu.init(IMU::WARM_START);	// offsets are loaded from EEPROM


	if(g.compass_enabled)
		init_compass();

	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused

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
							 "If using the Arduino Serial Monitor, ensure Line Ending is set to Carriage Return.\n"
							 "Type 'help' to list commands, 'exit' to leave a submenu.\n"
							 "Visit the 'setup' menu for first-time configuration.\n"));
		for (;;) {
			Serial.printf_P(PSTR("\n"
								 "Move the slide switch and reset to FLY.\n"
								 "\n"));
			main_menu.run();
		}
	}


	if(g.log_bitmask > 0){
		//	Here we will check  on the length of the last log
		//  We don't want to create a bunch of little logs due to powering on and off
		last_log_num 	= eeprom_read_byte((uint8_t *) EE_LAST_LOG_NUM);
		last_log_start 	= eeprom_read_word((uint16_t *) (EE_LOG_1_START+(last_log_num - 1) * 0x02));
		last_log_end 	= eeprom_read_word((uint16_t *) EE_LAST_LOG_PAGE);

		if(last_log_num == 0) {
			// The log space is empty.  Start a write session on page 1
			DataFlash.StartWrite(1);
			eeprom_write_byte((uint8_t *)	EE_LAST_LOG_NUM, (1));
			eeprom_write_word((uint16_t *)	EE_LOG_1_START, (1));

		} else if (last_log_end <= last_log_start + 10) {
			// The last log is small.  We consider it junk.  Overwrite it.
			DataFlash.StartWrite(last_log_start);

		} else {
			//  The last log is valid.  Start a new log
			if(last_log_num >= 19) {
				Serial.println("Number of log files exceeds max.  Log 19 will be overwritten.");
				last_log_num --;
			}
			DataFlash.StartWrite(last_log_end + 1);
			eeprom_write_byte((uint8_t *)	EE_LAST_LOG_NUM, (last_log_num + 1));
			eeprom_write_word((uint16_t *)	(EE_LOG_1_START+(last_log_num)*0x02), (last_log_end + 1));
		}
	}

	// read in the flight switches
	//update_servo_switches();

	//Serial.print("GROUND START");
	send_message(SEVERITY_LOW,"GROUND START");
	startup_ground();

	if (g.log_bitmask & MASK_LOG_CMD)
		Log_Write_Startup(TYPE_GROUNDSTART_MSG);

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void startup_ground(void)
{
	/*
	read_radio();
	while (g.rc_3.control_in > 0){
		delay(20);
		read_radio();
		APM_RC.OutputCh(CH_1, g.rc_3.radio_in);
		APM_RC.OutputCh(CH_2, g.rc_3.radio_in);
		APM_RC.OutputCh(CH_3, g.rc_3.radio_in);
		APM_RC.OutputCh(CH_4, g.rc_3.radio_in);
		Serial.println("*")
	}
	*/
	// read the radio to set trims
	// ---------------------------
	trim_radio();

	if (g.log_bitmask & MASK_LOG_CMD)
		Log_Write_Startup(TYPE_GROUNDSTART_MSG);

	#if(GROUND_START_DELAY > 0)
		send_message(SEVERITY_LOW,"With Delay");
		delay(GROUND_START_DELAY * 1000);
	#endif

	// Output waypoints for confirmation
	// --------------------------------
	for(int i = 1; i < g.waypoint_total + 1; i++) {
		gcs.send_message(MSG_COMMAND_LIST, i);
	}

	//IMU ground start
	//------------------------
	init_pressure_ground();

	// Warm up and read Gyro offsets
	// -----------------------------
	imu.init(IMU::COLD_START);

	// Save the settings for in-air restart
	// ------------------------------------
	save_EEPROM_groundstart();

	// initialize commands
	// -------------------
	init_commands();

	send_message(SEVERITY_LOW,"\n\n Ready to FLY.");
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
	if(g.rc_1.control_in == 0){
		// we are on the ground is this is true
		// disarm motors temp
		motor_auto_safe = false;
	}
	//send_message(SEVERITY_LOW,"control mode");
	//Serial.printf("set mode: %d old: %d\n", (int)mode, (int)control_mode);
	switch(control_mode)
	{
		case ACRO:
			break;

		case STABILIZE:
			set_current_loc_here();
			break;

		case ALT_HOLD:
			set_current_loc_here();
			break;

		case AUTO:
			update_auto();
			break;

		case POSITION_HOLD:
			set_current_loc_here();
			break;

		case RTL:
			return_to_launch();
			break;

		case TAKEOFF:
			break;

		case LAND:
			break;

		default:
			break;
	}

	// output control mode to the ground station
	send_message(MSG_HEARTBEAT);

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
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
			// We're back in radio contact
			// ---------------------------

			// re-read the switch so we can return to our preferred mode
			reset_control_switch();

			// Reset control integrators
			// ---------------------
			reset_I();

		}else{
			// We've lost radio contact
			// ------------------------
			// nothing to do right now
		}

		// Let the user know what's up so they can override the behavior
		// -------------------------------------------------------------
		failsafe_event();
	}
}

void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	if(g_gps->fix == 0){
		GPS_light = !GPS_light;
		if(GPS_light){
			digitalWrite(C_LED_PIN, HIGH);
		}else{
			digitalWrite(C_LED_PIN, LOW);
		}
	}else{
		if(!GPS_light){
			GPS_light = true;
			digitalWrite(C_LED_PIN, HIGH);
		}
	}

	if(motor_armed == true){
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


void resetPerfData(void) {
	/*
	mainLoop_count 		= 0;
	G_Dt_max 			= 0;
	gyro_sat_count 		= 0;
	adc_constraints 	= 0;
	renorm_sqrt_count 	= 0;
	renorm_blowup_count = 0;
	gps_fix_count 		= 0;
	perf_mon_timer 		= millis();
	*/
}

void
init_compass()
{
	dcm.set_compass(&compass);
	bool junkbool = compass.init();
	compass.set_orientation(MAGORIENTATION);						// set compass's orientation on aircraft
	Vector3f junkvector = compass.get_offsets();						// load offsets to account for airframe magnetic interference	
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

