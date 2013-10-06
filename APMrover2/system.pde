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
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

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
	{"logs",		process_logs},
	{"setup",		setup_mode},
	{"test",		test_mode},
    {"reboot",      reboot_board},
	{"help",		main_menu_help}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

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
    hal.uartB->begin(115200, 256, 16);

	cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
						 "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());
                    
	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	
    load_parameters();

    set_control_channels();

    // after parameter load setup correct baud rate on uartA
    hal.uartA->begin(map_baudrate(g.serial0_baud, SERIAL0_BAUD));

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

	// init the GCS
	gcs0.init(hal.uartA);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
	gcs3.init(hal.uartC);

	mavlink_system.sysid = g.sysid_this_mav;

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
		start_logging();
	}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    adc.Init();      // APM ADC library initialization
#endif

	if (g.compass_enabled==true) {
		if (!compass.init()|| !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();						// load offsets to account for airframe magnetic interference
        }
	}

	// initialise sonar
    init_sonar();

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

    relay.init();

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
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs3.initialised) {
        hal.uartC->println_P(msg);
    }

	startup_ground();

	if (g.log_bitmask & MASK_LOG_CMD)
			Log_Write_Startup(TYPE_GROUNDSTART_MSG);

    set_mode((enum mode)g.initial_mode.get());

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

	//IMU ground start
	//------------------------
    //

	startup_INS_ground(false);

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	// initialize commands
	// -------------------
	init_commands();

	// Makes the servos wiggle - 3 times signals ready to fly
	// -----------------------
	demo_servos(3);

    hal.uartA->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

	gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to drive."));
}

static void set_mode(enum mode mode)
{       

	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	control_mode = mode;
    throttle_last = 0;
    throttle = 500;

    if (control_mode != AUTO) {
        auto_triggered = false;
    }
        
	switch(control_mode)
	{
		case MANUAL:
		case HOLD:
		case LEARNING:
		case STEERING:
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

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode();
}

/*
  called to set/unset a failsafe event. 
 */
static void failsafe_trigger(uint8_t failsafe_type, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        gcs_send_text_fmt(PSTR("Failsafe ended"));
    }

    failsafe.triggered &= failsafe.bits;

    if (failsafe.triggered == 0 && 
        failsafe.bits != 0 && 
        millis() - failsafe.start_time > g.fs_timeout*1000 &&
        control_mode != RTL &&
        control_mode != HOLD) {
        failsafe.triggered = failsafe.bits;
        gcs_send_text_fmt(PSTR("Failsafe trigger 0x%x"), (unsigned)failsafe.triggered);
        switch (g.fs_action) {
        case 0:
            break;
        case 1:
            set_mode(RTL);
            break;
        case 2:
            set_mode(HOLD);
            break;
        }
    }
}

static void startup_INS_ground(bool force_accel_level)
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Warming up ADC..."));
 	mavlink_delay(500);

	// Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
	// -----------------------
	demo_servos(2);
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move vehicle"));
	mavlink_delay(1000);

    ahrs.init();
	ahrs.set_fly_forward(true);
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);

    if (force_accel_level) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station	
        ins.init_accel();
        ahrs.set_trim(Vector3f(0, 0, 0));
	}
    ahrs.reset();
}

// updates the notify state
// should be called at 50hz
static void update_notify()
{
    notify.update();
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


static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL3_BAUD.
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#endif
}


/*
 * Read board voltage in millivolts
 */
uint16_t board_voltage(void)
{
    return vcc_pin->voltage_latest() * 1000;
}

static void
print_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->print_P(PSTR("Manual"));
        break;
    case HOLD:
        port->print_P(PSTR("HOLD"));
        break;
    case LEARNING:
        port->print_P(PSTR("Learning"));
        break;
    case STEERING:
        port->print_P(PSTR("Stearing"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

/*
  check a digitial pin for high,low (1/0)
 */
static uint8_t check_digital_pin(uint8_t pin)
{
    int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
    if (dpin == -1) {
        return 0;
    }
    // ensure we are in input mode
    hal.gpio->pinMode(dpin, GPIO_INPUT);

    // enable pullup
    hal.gpio->write(dpin, 1);

    return hal.gpio->read(dpin);
}

/*
  write to a servo
 */
static void servo_write(uint8_t ch, uint16_t pwm)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (ch < 8) {
        RC_Channel::rc_channel(ch)->radio_out = pwm;
    }
#else
    hal.rcout->enable_ch(ch);
    hal.rcout->write(ch, pwm);
#endif
}
