// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED

// Functions called from the top-level menu
#if LITE == DISABLED
static int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
#endif
static int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
static int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	main_menu_help(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("Commands:\n"
						 "  logs        log readback/setup mode\n"
						 "  setup       setup mode\n"
						 "  test        test mode\n"
						 "\n"
						 "Move the slide switch and reset to FLY.\n"
						 "\n"));
	return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
#if LITE == DISABLED
	{"logs",		process_logs},
#endif
	{"setup",		setup_mode},
	{"test",		test_mode},
	{"help",		main_menu_help}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

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
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	// XXX currently the EM406 (SiRF receiver) is nominally configured
	// at 57600, however it's not been supported to date.  We should
	// probably standardise on 38400.
	//
	// XXX the 128 byte receive buffer may be too small for NMEA, depending
	// on the message set configured.
	//
    // standard gps running
    hal.uartB->begin(115200, 128, 16);

	cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
						 "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());
                    
	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	
    load_parameters();

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

	// init the GCS
	gcs0.init(hal.uartA);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

#if USB_MUX_PIN > 0
    if (!usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#else
    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
	gcs3.init(hal.uartC);
#endif

	mavlink_system.sysid = g.sysid_this_mav;

#if LITE == DISABLED
#if LOGGING_ENABLED == ENABLED
	DataFlash.Init(); 	// DataFlash log initialization
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0) {
		DataFlash.start_new_log();
	}
#endif
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE

#if CONFIG_ADC == ENABLED
    adc.Init();      // APM ADC library initialization
#endif

#if LITE == DISABLED
	if (g.compass_enabled==true) {
        compass.set_orientation(MAG_ORIENTATION);							// set compass's orientation on aircraft
		if (!compass.init()|| !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();						// load offsets to account for airframe magnetic interference
        }
	}
#else
  I2c.begin();
  I2c.timeOut(20);

  // I2c.setSpeed(true);

  if (!compass.init()) {
	  cliSerial->println("compass initialisation failed!");
	  while (1) ;
  }

  compass.set_orientation(MAG_ORIENTATION);  // set compass's orientation on aircraft.
  compass.set_offsets(0,0,0);  // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0));  // set local difference between magnetic north and true north

  cliSerial->print("Compass auto-detected as: ");
  switch( compass.product_id ) {
      case AP_COMPASS_TYPE_HIL:
	      cliSerial->println("HIL");
		  break;
      case AP_COMPASS_TYPE_HMC5843:
	      cliSerial->println("HMC5843");
		  break;
      case AP_COMPASS_TYPE_HMC5883L:
	      cliSerial->println("HMC5883L");
		  break;
      default:
	      cliSerial->println("unknown");
		  break;
  }
  
  delay(3000);

#endif
	// initialise sonar
	#if CONFIG_SONAR == ENABLED
	init_sonar();
	#endif

#endif
	// Do GPS init
	g_gps = &g_gps_driver;
    // GPS initialisation
	g_gps->init(hal.uartB, GPS::GPS_ENGINE_AUTOMOTIVE);

	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_TYPE_GROUND_ROVER;

    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	pinMode(C_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(A_LED_PIN, OUTPUT);			// GPS status LED
	pinMode(B_LED_PIN, OUTPUT);			// GPS status LED
#if SLIDE_SWITCH_PIN > 0
	pinMode(SLIDE_SWITCH_PIN, INPUT);	// To enter interactive mode
#endif
#if CONFIG_PUSHBUTTON == ENABLED
	pinMode(PUSHBUTTON_PIN, INPUT);		// unused
#endif
#if CONFIG_RELAY == ENABLED
    relay.init();
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
#if CLI_ENABLED == ENABLED && CLI_SLIDER_ENABLED == ENABLED
	if (digitalRead(SLIDE_SWITCH_PIN) == 0) {
		digitalWrite(A_LED_PIN,LED_ON);		// turn on setup-mode LED
		cliSerial->printf_P(PSTR("\n"
							 "Entering interactive setup mode...\n"
							 "\n"
							 "If using the Arduino Serial Monitor, ensure Line Ending is set to Carriage Return.\n"
							 "Type 'help' to list commands, 'exit' to leave a submenu.\n"
							 "Visit the 'setup' menu for first-time configuration.\n"));
        cliSerial->println_P(PSTR("\nMove the slide switch and reset to FLY.\n"));
        run_cli(&cliSerial);
	}
#else
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif
#endif // CLI_ENABLED

	startup_ground();

#if LITE == DISABLED
	if (g.log_bitmask & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_GROUNDSTART_MSG);
#endif

    set_mode(MANUAL);

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
static void startup_ground(void)
{
    set_mode(INITIALISING);

	gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> GROUND START"));

	#if(GROUND_START_DELAY > 0)
		gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> With Delay"));
		delay(GROUND_START_DELAY * 1000);
	#endif

	// Makes the servos wiggle
	// step 1 = 1 wiggle
	// -----------------------
	demo_servos(1);

#if LITE == DISABLED
	//IMU ground start
	//------------------------
    //

	startup_INS_ground(false);
#endif

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	// initialize commands
	// -------------------
	init_commands();

	// Makes the servos wiggle - 3 times signals ready to fly
	// -----------------------
	demo_servos(3);

	gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to drive."));
}

static void set_mode(uint8_t mode)
{       

	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	if(g.auto_trim > 0 && control_mode == MANUAL)
		trim_control_surfaces();

	control_mode = mode;
    throttle_last = 0;
    throttle = 500;
        
	switch(control_mode)
	{
		case MANUAL:
		case LEARNING:
		case CIRCLE:
			break;

		case AUTO:
            rtl_complete = false;
            restart_nav();
			break;

		case RTL:
			do_RTL();
			break;

		default:
			do_RTL();
			break;
	}

#if LITE == DISABLED
	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
#endif

}

static void check_long_failsafe()
{
	// only act on changes
	// -------------------
	if(failsafe != FAILSAFE_LONG  && failsafe != FAILSAFE_GCS){
		if(rc_override_active && millis() - rc_override_fs_timer > FAILSAFE_LONG_TIME) {
			failsafe_long_on_event(FAILSAFE_LONG);
		}
		if(! rc_override_active && failsafe == FAILSAFE_SHORT && millis() - ch3_failsafe_timer > FAILSAFE_LONG_TIME) {
			failsafe_long_on_event(FAILSAFE_LONG);
		}
		if(g.gcs_heartbeat_fs_enabled && millis() - rc_override_fs_timer > FAILSAFE_LONG_TIME) {
			failsafe_long_on_event(FAILSAFE_GCS);
		}
	} else {
		// We do not change state but allow for user to change mode
		if(failsafe == FAILSAFE_GCS && millis() - rc_override_fs_timer < FAILSAFE_SHORT_TIME) failsafe = FAILSAFE_NONE;
		if(failsafe == FAILSAFE_LONG && rc_override_active && millis() - rc_override_fs_timer < FAILSAFE_SHORT_TIME) failsafe = FAILSAFE_NONE;
		if(failsafe == FAILSAFE_LONG && !rc_override_active && !ch3_failsafe) failsafe = FAILSAFE_NONE;
	}
}

static void check_short_failsafe()
{
	// only act on changes
	// -------------------
	if(failsafe == FAILSAFE_NONE){
		if(ch3_failsafe) {					// The condition is checked and the flag ch3_failsafe is set in radio.pde
			failsafe_short_on_event(FAILSAFE_SHORT);
		}
	}

	if(failsafe == FAILSAFE_SHORT){
		if(!ch3_failsafe) {
			failsafe_short_off_event();
		}
	}
}

#if LITE == DISABLED
static void startup_INS_ground(bool force_accel_level)
{
#if HIL_MODE != HIL_MODE_ATTITUDE
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Warming up ADC..."));
 	mavlink_delay(500);

	// Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
	// -----------------------
	demo_servos(2);
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move plane"));
	mavlink_delay(1000);

	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate, 
             flash_leds);
    if (force_accel_level || g.manual_level == 0) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station	
        ins.init_accel(flash_leds);
	}
	ahrs.set_fly_forward(true);
    ahrs.reset();

#endif // HIL_MODE_ATTITUDE

	digitalWrite(B_LED_PIN, LED_ON);		// Set LED B high to indicate INS ready
	digitalWrite(A_LED_PIN, LED_OFF);
	digitalWrite(C_LED_PIN, LED_OFF);
}
#endif

static void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	switch (g_gps->status()) {
		case(2):
			digitalWrite(C_LED_PIN, LED_ON);  //Turn LED C on when gps has valid fix.
			break;

		case(1):
			if (g_gps->valid_read == true){
				GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
				if (GPS_light){
					digitalWrite(C_LED_PIN, LED_OFF);
				} else {
					digitalWrite(C_LED_PIN, LED_ON);
				}
				g_gps->valid_read = false;
			}
			break;

		default:
			digitalWrite(C_LED_PIN, LED_OFF);
			break;
	}
}


static void resetPerfData(void) {
	mainLoop_count 			= 0;
	G_Dt_max 				= 0;
	ahrs.renorm_range_count 	= 0;
	ahrs.renorm_blowup_count = 0;
	gps_fix_count 			= 0;
	pmTest1					= 0;
	perf_mon_timer 			= millis();
}


/*
  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
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
        hal.uartA->begin(SERIAL0_BAUD, 128, 128);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
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

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return vcc_pin->read_latest();
}

static void
print_flight_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        cliSerial->println_P(PSTR("Manual"));
        break;
    case LEARNING:
        cliSerial->println_P(PSTR("Learning"));
        break;
    case AUTO:
        cliSerial->println_P(PSTR("AUTO"));
        break;
    case RTL:
        cliSerial->println_P(PSTR("RTL"));
        break;
    default:
        cliSerial->println_P(PSTR("---"));
        break;
    }
}

/*
  force a software reset of the APM
 */
static void reboot_apm(void)
{
    hal.scheduler->reboot();
    while (1);
}
