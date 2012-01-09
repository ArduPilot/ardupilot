// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED
// Functions called from the top-level menu
static int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
static int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
static int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp
static int8_t	planner_mode(uint8_t argc, const Menu::arg *argv);	// in planner.pde

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	main_menu_help(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Commands:\n"
						 "  logs\n"
						 "  setup\n"
						 "  test\n"
 						 "  planner\n"
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
	{"help",		main_menu_help},
	{"planner",		planner_mode}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

// the user wants the CLI. It never exits
static void run_cli(void)
{
    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void init_ardupilot()
{
#if USB_MUX_PIN > 0
    // on the APM2 board we have a mux thet switches UART0 between
    // USB and the board header. If the right ArduPPM firmware is
    // installed we can detect if USB is connected using the
    // USB_MUX_PIN
    pinMode(USB_MUX_PIN, INPUT);

    usb_connected = !digitalRead(USB_MUX_PIN);
    if (!usb_connected) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }
#endif

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

	Serial.printf_P(PSTR("\n\nInit " THISFIRMWARE
						 "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());

	//
	// Initialize Wire and SPI libraries
	//
#ifndef DESKTOP_BUILD
    I2c.begin();
    I2c.timeOut(5);
    // initially set a fast I2c speed, and drop it on first failures
    I2c.setSpeed(true);
#endif
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHZ SPI rate
	//
	// Initialize the isr_registry.
	//
    isr_registry.init();

	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	report_version();

	// setup IO pins
	pinMode(A_LED_PIN, OUTPUT);				// GPS status LED
  digitalWrite(A_LED_PIN, LED_OFF);

  pinMode(B_LED_PIN, OUTPUT);				// GPS status LED
  digitalWrite(B_LED_PIN, LED_OFF);

  pinMode(C_LED_PIN, OUTPUT);				// GPS status LED
  digitalWrite(C_LED_PIN, LED_OFF);

#if SLIDE_SWITCH_PIN > 0
  pinMode(SLIDE_SWITCH_PIN, INPUT);		// To enter interactive mode
#endif
#if CONFIG_PUSHBUTTON == ENABLED
	pinMode(PUSHBUTTON_PIN, INPUT);			// unused
#endif
#if CONFIG_RELAY == ENABLED
	DDRL |= B00000100;						// Set Port L, pin 2 to output for the relay
#endif
	// XXX set Analog out 14 to output
	//  	   76543210
	//DDRK |= B01010000;

	#if MOTOR_LEDS == 1
		pinMode(FR_LED, OUTPUT);			// GPS status LED
		pinMode(RE_LED, OUTPUT);			// GPS status LED
		pinMode(RI_LED, OUTPUT);			// GPS status LED
		pinMode(LE_LED, OUTPUT);			// GPS status LED
	#endif

	#if PIEZO == 1
		pinMode(PIEZO_PIN,OUTPUT);
		piezo_beep();
	#endif


	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {
		//Serial.printf_P(PSTR("\n\nForcing complete parameter reset..."));

		/*Serial.printf_P(PSTR("\n\nEEPROM format version  %d not compatible with this firmware (requires %d)"
		                     "\n\nForcing complete parameter reset..."),
		                     g.format_version.get(),
		                     Parameters::k_format_version);
		*/

		// erase all parameters
		Serial.printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
		delay(100); // wait for serial send
		AP_Var::erase_all();

		// save the new format version
		g.format_version.set_and_save(Parameters::k_format_version);

		// save default radio values
		default_dead_zones();
	}else{
		// save default radio values
		//default_dead_zones();

	    // Load all auto-loaded EEPROM variables
	    AP_Var::load_all();
	}

	// init the GCS
    gcs0.init(&Serial);

#if USB_MUX_PIN > 0
    if (!usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        Serial.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
    }
#else
    // we have a 2nd serial port for telemetry
    Serial3.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
	gcs3.init(&Serial3);
#endif

    // identify ourselves correctly with the ground station
	mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0){
		DataFlash.start_new_log();
	}
#endif

	#ifdef RADIO_OVERRIDE_DEFAULTS
	{
		int16_t rc_override[8] = RADIO_OVERRIDE_DEFAULTS;
		APM_RC.setHIL(rc_override);
	}
	#endif

    #if FRAME_CONFIG ==	HELI_FRAME
		g.heli_servo_manual = false;
		heli_init_swash();  // heli initialisation
	#endif

    RC_Channel::set_apm_rc(&APM_RC);
	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	init_camera();

    timer_scheduler.init( &isr_registry );

#if HIL_MODE != HIL_MODE_ATTITUDE
#if CONFIG_ADC == ENABLED
		// begin filtering the ADC Gyros
		adc.filter_result = true;
        adc.Init(&timer_scheduler);       // APM ADC library initialization
#endif // CONFIG_ADC

        barometer.init(&timer_scheduler);

#endif // HIL_MODE

	// Do GPS init
	g_gps = &g_gps_driver;
	g_gps->init();			// GPS Initialization
    g_gps->callback = mavlink_delay;

	if(g.compass_enabled)
		init_compass();

	// init the optical flow sensor
	if(g.optflow_enabled) {
		init_optflow();
	}


// agmatthews USERHOOKS
#ifdef USERHOOK_INIT
   USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED && CLI_SLIDER_ENABLED == ENABLED
	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
	if (check_startup_for_CLI()) {
		digitalWrite(A_LED_PIN, LED_ON);		// turn on setup-mode LED
		Serial.printf_P(PSTR("\nCLI:\n\n"));
        run_cli();
	}
#else
    Serial.printf_P(PSTR("\nPress ENTER 3 times for CLI\n\n"));
#endif // CLI_ENABLED

    GPS_enabled = false;

	#if HIL_MODE == HIL_MODE_DISABLED
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
	#else
		GPS_enabled = true;
	#endif

	// lengthen the idle timeout for gps Auto_detect
	// ---------------------------------------------
	g_gps->idleTimeout = 20000;

	// print the GPS status
	// --------------------
	report_gps();

	#if HIL_MODE != HIL_MODE_ATTITUDE
	// read Baro pressure at ground
	//-----------------------------
	init_barometer();
	#endif

	// initialise sonar
	#if HIL_MODE != HIL_MODE_ATTITUDE && CONFIG_SONAR == ENABLED
	init_sonar();
	#endif

	// initialize commands
	// -------------------
	init_commands();

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();

	#if HIL_MODE != HIL_MODE_ATTITUDE
		dcm.kp_roll_pitch(0.130000);
		dcm.ki_roll_pitch(0.00001278),	// 50 hz I term
		dcm.kp_yaw(0.08);
		dcm.ki_yaw(0.00004);
		dcm._clamp = 5;
	#endif

	// init the Z damopener
	// --------------------
	#if ACCEL_ALT_HOLD != 0
	init_z_damper();
	#endif


	startup_ground();

#if LOGGING_ENABLED == ENABLED
	Log_Write_Startup();
	Log_Write_Data(10, g.pi_stabilize_roll.kP());
	Log_Write_Data(11, g.pi_stabilize_pitch.kP());
	Log_Write_Data(12, g.pi_rate_roll.kP());
	Log_Write_Data(13, g.pi_rate_pitch.kP());
#endif

	SendDebug("\nReady to FLY ");
}

/*
  reset all I integrators
 */
static void reset_I_all(void)
{
	g.pi_rate_roll.reset_I();
	g.pi_rate_pitch.reset_I();
	g.pi_rate_yaw.reset_I();
	g.pi_stabilize_roll.reset_I();
	g.pi_stabilize_pitch.reset_I();
	g.pi_stabilize_yaw.reset_I();
	g.pi_loiter_lat.reset_I();
	g.pi_loiter_lon.reset_I();
	g.pi_nav_lat.reset_I();
	g.pi_nav_lon.reset_I();
	g.pi_alt_hold.reset_I();
	g.pi_throttle.reset_I();
	g.pi_acro_roll.reset_I();
	g.pi_acro_pitch.reset_I();
	g.pi_optflow_roll.reset_I();
	g.pi_optflow_pitch.reset_I();
}


//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
static void startup_ground(void)
{
	gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

	#if HIL_MODE != HIL_MODE_ATTITUDE
		// Warm up and read Gyro offsets
		// -----------------------------
        imu.init(IMU::COLD_START, mavlink_delay, flash_leds, &timer_scheduler);
		#if CLI_ENABLED == ENABLED
			report_imu();
		#endif
	#endif

	// reset the leds
	// ---------------------------
	clear_leds();

    // when we re-calibrate the gyros, all previous I values now
    // make no sense
    reset_I_all();
}

/*
#define YAW_HOLD 			0
#define YAW_ACRO 			1
#define YAW_AUTO 			2
#define YAW_LOOK_AT_HOME 	3

#define ROLL_PITCH_STABLE 	0
#define ROLL_PITCH_ACRO 	1
#define ROLL_PITCH_AUTO		2

#define THROTTLE_MANUAL 	0
#define THROTTLE_HOLD 		1
#define THROTTLE_AUTO		2

*/

static void set_mode(byte mode)
{
	//if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
	//	Serial.print("Double mode\n");
		//return;
	//}

	// if we don't have GPS lock
	if(home_is_set == false){
		// our max mode should be
		if (mode > ALT_HOLD)
			mode = STABILIZE;
	}

	// nothing but Loiter for OptFlow only
	if (g.optflow_enabled && GPS_enabled == false){
		if (mode > ALT_HOLD && mode != LOITER)
			mode = STABILIZE;
	}

	old_control_mode = control_mode;

	control_mode = mode;
	control_mode = constrain(control_mode, 0, NUM_MODES - 1);

	// used to stop fly_aways
	motor_auto_armed = (g.rc_3.control_in > 0);

	// clearing value used in interactive alt hold
	manual_boost = 0;

	// do not auto_land if we are leaving RTL
	auto_land_timer = 0;

	// if we change modes, we must clear landed flag
	land_complete 	= false;

	// debug to Serial terminal
	Serial.println(flight_mode_strings[control_mode]);

	// report the GPS and Motor arming status
	led_mode = NORMAL_LEDS;

	switch(control_mode)
	{
		case ACRO:
			yaw_mode 		= YAW_ACRO;
			roll_pitch_mode = ROLL_PITCH_ACRO;
			throttle_mode 	= THROTTLE_MANUAL;
			reset_rate_I();
			break;

		case STABILIZE:
			yaw_mode 		= YAW_HOLD;
			roll_pitch_mode = ROLL_PITCH_STABLE;
			throttle_mode 	= THROTTLE_MANUAL;
			break;

		case ALT_HOLD:
			yaw_mode 		= ALT_HOLD_YAW;
			roll_pitch_mode = ALT_HOLD_RP;
			throttle_mode 	= ALT_HOLD_THR;

			next_WP = current_loc;
			// 1m is the alt hold limit
			next_WP.alt = max(next_WP.alt, 100);
			break;

		case AUTO:
			yaw_mode 		= AUTO_YAW;
			roll_pitch_mode = AUTO_RP;
			throttle_mode 	= AUTO_THR;

			// loads the commands from where we left off
			init_commands();
			break;

		case CIRCLE:
			yaw_mode 		= CIRCLE_YAW;
			roll_pitch_mode = CIRCLE_RP;
			throttle_mode 	= CIRCLE_THR;
			next_WP 		= current_loc;

			// reset the desired circle angle
			circle_angle 	= 0;
			break;

		case LOITER:
			yaw_mode 		= LOITER_YAW;
			roll_pitch_mode = LOITER_RP;
			throttle_mode 	= LOITER_THR;
			next_WP 		= current_loc;
			break;

		case POSITION:
			yaw_mode 		= YAW_HOLD;
			roll_pitch_mode = ROLL_PITCH_AUTO;
			throttle_mode 	= THROTTLE_MANUAL;
			next_WP 		= current_loc;
			break;

		case GUIDED:
			yaw_mode 		= YAW_AUTO;
			roll_pitch_mode = ROLL_PITCH_AUTO;
			throttle_mode 	= THROTTLE_AUTO;
			next_WP 		= current_loc;
			set_next_WP(&guided_WP);
			break;

		case LAND:
			yaw_mode 		= LOITER_YAW;
			roll_pitch_mode = LOITER_RP;
			throttle_mode 	= THROTTLE_AUTO;
			do_land();
			break;

		case RTL:
			yaw_mode 		= RTL_YAW;
			roll_pitch_mode = RTL_RP;
			throttle_mode 	= RTL_THR;

			do_RTL();
			break;

		default:
			break;
	}

	if(failsafe){
		// this is to allow us to fly home without interactive throttle control
		throttle_mode = THROTTLE_AUTO;
		// does not wait for us to be in high throttle, since the
		// Receiver will be outputting low throttle
		motor_auto_armed = true;
	}

	if(throttle_mode == THROTTLE_MANUAL){
		// reset all of the throttle iterms
		g.pi_alt_hold.reset_I();
		g.pi_throttle.reset_I();

	}else {
		// an automatic throttle
		// todo: replace with a throttle cruise estimator
		init_throttle_cruise();
	}

	if(roll_pitch_mode <= ROLL_PITCH_ACRO){
		// We are under manual attitude control
		// removes the navigation from roll and pitch commands, but leaves the wind compensation
		reset_nav();

		// removes the navigation from roll and pitch commands, but leaves the wind compensation
		if(GPS_enabled)
			wp_control = NO_NAV_MODE;
			update_nav_wp();

	}

	Log_Write_Mode(control_mode);
}

static void set_failsafe(boolean mode)
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

static void
init_simple_bearing()
{
	initial_simple_bearing = dcm.yaw_sensor;
}

static void
init_throttle_cruise()
{
	// are we moving from manual throttle to auto_throttle?
	if((old_control_mode <= STABILIZE) && (g.rc_3.control_in > MINIMUM_THROTTLE)){
		g.pi_throttle.reset_I();
		g.pi_alt_hold.reset_I();
		g.throttle_cruise.set_and_save(g.rc_3.control_in);
	}
}

#if CLI_SLIDER_ENABLED == ENABLED && CLI_ENABLED == ENABLED
static boolean
check_startup_for_CLI()
{
	return (digitalRead(SLIDE_SWITCH_PIN) == 0);
}
#endif // CLI_ENABLED

/*
  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    //Serial.println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}

#if USB_MUX_PIN > 0
static void check_usb_mux(void)
{
    bool usb_check = !digitalRead(USB_MUX_PIN);
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
    if (usb_connected) {
        Serial.begin(SERIAL0_BAUD, 128, 128);
    } else {
        Serial.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
    }
}
#endif

/*
  called by gyro/accel init to flash LEDs so user
  has some mesmerising lights to watch while waiting
 */
void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}
