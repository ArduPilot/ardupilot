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
	#if HIL_MODE != HIL_MODE_DISABLED && HIL_PORT == 1 // TODO: figure out a better way to do this
		// Steal gps port for hardware in the loop
		Serial1.begin(115200, 128, 128);
	#else
		// standard gps running
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

	Serial.printf_P(PSTR("\n\n"
						 "Init ArduPilotMega (unstable development version)\n\n"
						 "Firmware Version: %d freeRAM: %lu\n"),
					FIRMWARE_VERSION, freeRAM());

	// !!! Check firmware version before loading any 
	// data from EEPROM !!!
	if (FIRMWARE_VERSION != get(PARAM_FIRMWARE_VER))
	{
		Serial.printf_P(PSTR("\n\n"
			"Firmware Mismatch! ROM Firmware Version: %d\n"
			"\nA factory reset is being performed."),get(PARAM_FIRMWARE_VER));

		// If firmware mismatches a factory reset is forced
		param_reset_defaults();
	}


	APM_RC.Init();		// APM Radio initialization
	//read_EEPROM_startup(); // Read critical config information to start

#if HIL_MODE != HIL_MODE_ATTITUDE
	adc.Init();	 		// APM ADC library initialization
	pitot.Init();	// APM Abs Pressure sensor initialization
	compass.init();	// I2C initialization
#endif
	DataFlash.Init(); 	// DataFlash log initialization
	gps.init();			// GPS Initialization

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

	APM_RC.OutputCh(CH_ROLL, 		radio_trim(CH_ROLL));					// Initialization of servo outputs
	APM_RC.OutputCh(CH_PITCH, 		radio_trim(CH_PITCH));
	APM_RC.OutputCh(CH_THROTTLE, 	radio_trim(CH_THROTTLE));
	APM_RC.OutputCh(CH_RUDDER, 		radio_trim(CH_RUDDER));
 
	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused
	DDRL |= B00000100;					// Set Port L, pin 2 to output for the relay

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
	

	if(get(PARAM_LOG_BITMASK) > 0){
		//	Here we will check  on the length of the last log
		//  We don't want to create a bunch of little logs due to powering on and off
		//  XXX: TODO implement for new struct
		//last_log_num 	= eeprom_read_byte((uint8_t *) EE_LAST_LOG_NUM);
		//last_log_start 	= eeprom_read_word((uint16_t *) (EE_LOG_1_START+(last_log_num - 1) * 0x02));
		//last_log_end 	= eeprom_read_word((uint16_t *) EE_LAST_LOG_PAGE);

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
	update_servo_switches();

	if(DEBUG_SUBSYSTEM > 0){
		debug_subsystem();

	} else if (ENABLE_AIR_START == 1) {
		// Perform an air start and get back to flying
		gcs.send_text(SEVERITY_LOW,"<init_ardupilot> AIR START");	
		
		// Get necessary data from EEPROM
		//----------------
		//read_EEPROM_airstart_critical();
#if HIL_MODE != HIL_MODE_ATTITUDE
		imu.load_gyro_eeprom();
        imu.load_accel_eeprom();
#endif

		// This delay is important for the APM_RC library to work.  
		// We need some time for the comm between the 328 and 1280 to be established.
		int old_pulse = 0;
		while (millis()<=1000 && (abs(old_pulse - APM_RC.InputCh(get(PARAM_FLIGHT_MODE_CH))) > 5 || 
					APM_RC.InputCh(get(PARAM_FLIGHT_MODE_CH)) == 1000 || 
					APM_RC.InputCh(get(PARAM_FLIGHT_MODE_CH)) == 1200)) {
			old_pulse = APM_RC.InputCh(get(PARAM_FLIGHT_MODE_CH));
			delay(25);
		}
		if (get(PARAM_LOG_BITMASK) & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_AIRSTART_MSG);
		reload_commands();		// Get set to resume AUTO from where we left off

	}else {
		startup_ground();
		if (get(PARAM_LOG_BITMASK) & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_GROUNDSTART_MSG);
	}

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void startup_ground(void)
{	
	gcs.send_text(SEVERITY_LOW,"<startup_ground> GROUND START");

	#if(GROUND_START_DELAY > 0)	
		gcs.send_text(SEVERITY_LOW,"<startup_ground> With Delay");
		delay(GROUND_START_DELAY * 1000);
	#endif
	
	// Output waypoints for confirmation
	// --------------------------------
	for(int i = 1; i < get(PARAM_WP_TOTAL) + 1; i++) {
		gcs.send_message(MSG_COMMAND_LIST, i);
	}

	// Makes the servos wiggle
	// step 1 = 1 wiggle
	// -----------------------
	demo_servos(1);

	//IMU ground start
	//------------------------
    //
	startup_IMU_ground();
	
	// read the radio to set trims
	// ---------------------------
	trim_radio();
	
#if HIL_MODE != HIL_MODE_ATTITUDE
# if AIRSPEED_SENSOR == ENABLED
	// initialize airspeed sensor
	// --------------------------
	zero_airspeed();
	gcs.send_text(SEVERITY_LOW,"<startup_ground> zero airspeed calibrated");
# else
	gcs.send_text(SEVERITY_LOW,"<startup_ground> NO airspeed");
# endif
#endif

	// Save the settings for in-air restart
	// ------------------------------------
	//save_EEPROM_groundstart();
		
	// initialize commands
	// -------------------
	init_commands();
	
	// Makes the servos wiggle - 3 times signals ready to fly
	// -----------------------
	demo_servos(3);

	gcs.send_text(SEVERITY_LOW,"\n\n Ready to FLY.");
}

void set_mode(byte mode)
{
	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	if(get(PARAM_TRIM_AUTO) > 0 || control_mode == MANUAL) 
		trim_control_surfaces();
	
	control_mode = mode;
	crash_timer = 0;
	
	switch(control_mode)
	{
		case MANUAL:
			break;

		case STABILIZE:
			break;
		
		case FLY_BY_WIRE_A:
			break;
		
		case FLY_BY_WIRE_B:
			break;

		case AUTO:
			update_auto();
			break;

		case RTL:
			return_to_launch();
			break;
		
		case LOITER:
			loiter_at_location();
			break;

		case TAKEOFF:
			break;

		case LAND:
			break;

		default:
			return_to_launch();
			break;
	}
	
	// output control mode to the ground station
	gcs.send_message(MSG_MODE_CHANGE);
	
	if (get(PARAM_LOG_BITMASK) & MASK_LOG_MODE)	
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


void startup_IMU_ground(void)
{
#if HIL_MODE != HIL_MODE_ATTITUDE
	uint16_t store = 0;
	int flashcount = 0;
	SendDebugln("<startup_IMU_ground> Warming up ADC...");
 	delay(500);
 	
	// Makes the servos wiggle twice - about to begin IMU calibration - HOLD LEVEL AND STILL!!
	// -----------------------
	demo_servos(2);
	SendDebugln("<startup_IMU_ground> Beginning IMU calibration; do not move plane");
	delay(1000);

	imu.init_accel();
	imu.init_gyro();

# if HIL_MODE == HIL_MODE_SENSORS
	hil.update();		// look for inbound hil packets for initialization
# endif

	pitot.Read(); 	// Get initial data from absolute pressure sensor
	abs_press_gnd = pitot.Press;
	ground_temperature = pitot.Temp;
	delay(20);
	// ***********

	for(int i = 0; i < 200; i++){		// We take some readings...

# if HIL_MODE == HIL_MODE_SENSORS
		hil.update(); // look for inbound hil packets
# endif

		pitot.Read(); 	// Get initial data from absolute pressure sensor
		abs_press_gnd = (abs_press_gnd * 9l + pitot.Press) / 10l;
		ground_temperature = (ground_temperature * 9 + pitot.Temp) / 10;
		
		delay(20);
		if(flashcount == 5) {
			digitalWrite(C_LED_PIN, LOW);
			digitalWrite(A_LED_PIN, HIGH);
			digitalWrite(B_LED_PIN, LOW);
		}
		if(flashcount >= 10) {
			flashcount = 0;
			digitalWrite(C_LED_PIN, HIGH);
			digitalWrite(A_LED_PIN, LOW);
			digitalWrite(B_LED_PIN, HIGH);
		}
		flashcount++;
		
	}
	SendDebugln("  <startup_IMU_ground> Calibration complete.");
#endif // HIL_MODE_ATTITUDE

	digitalWrite(B_LED_PIN, HIGH);		// Set LED B high to indicate IMU ready
	digitalWrite(A_LED_PIN, LOW);
	digitalWrite(C_LED_PIN, LOW);
}


void update_GPS_light(void)
{				
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	switch (gps.status()) {
		case(2):
			digitalWrite(C_LED_PIN, HIGH);  //Turn LED C on when gps has valid fix. 
			break;
                                              
		case(1):
			if (gps.valid_read == true){
				GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
				if (GPS_light){
					digitalWrite(C_LED_PIN, LOW); 
				} else {
					digitalWrite(C_LED_PIN, HIGH); 
				}
				gps.valid_read = false;
			}
			break;
                                              
		default:
			digitalWrite(C_LED_PIN, LOW);
			break;
	}
}



void resetPerfData(void) {
	mainLoop_count 		= 0;
	G_Dt_max 			= 0;
	gyro_sat_count 		= 0;
	adc_constraints 	= 0;
	renorm_sqrt_count 	= 0;	
	renorm_blowup_count = 0;
	gps_fix_count 		= 0;
	perf_mon_timer 		= millis();
}


/* 
 * This function gets the current value of the heap and stack pointers.
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

