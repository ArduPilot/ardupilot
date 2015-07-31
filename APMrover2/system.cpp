// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#include "Rover.h"

#if CLI_ENABLED == ENABLED

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
int8_t Rover::main_menu_help(uint8_t argc, const Menu::arg *argv)
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
	{"logs",		MENU_FUNC(process_logs)},
	{"setup",		MENU_FUNC(setup_mode)},
	{"test",		MENU_FUNC(test_mode)},
    {"reboot",      MENU_FUNC(reboot_board)},
	{"help",		MENU_FUNC(main_menu_help)}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Rover::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Rover::run_cli(AP_HAL::UARTDriver *port)
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

static void mavlink_delay_cb_static()
{
    rover.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    rover.failsafe_check();
}

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    AP_ADC_ADS7844 apm1_adc;
#endif

void Rover::init_ardupilot()
{
    // initialise console serial port
    serial_manager.init_console();

	cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
						 "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());
                    
	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
	
    load_parameters();

    BoardConfig.init();

    // initialise serial ports
    serial_manager.init();

    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();

    battery.init();

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

	// init the GCS
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // setup serial port for telem1
    gcs[1].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // setup serial port for telem2
    gcs[2].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 1);
#endif

#if MAVLINK_COMM_NUM_BUFFERS > 3
    // setup serial port for fourth telemetry port (not used by default)
    gcs[3].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 2);
#endif

    // setup frsky telemetry
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.init(serial_manager);
#endif

	mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    apm1_adc.Init();      // APM ADC library initialization
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

    // and baro for EKF
    init_barometer();

	// Do GPS init
    gps.init(&DataFlash, serial_manager);

    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

    relay.init();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(serial_manager);
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);


#if CLI_ENABLED == ENABLED
	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
    if (g.cli_enabled == 1) {
        const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
        cliSerial->println_P(msg);
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println_P(msg);
        }
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println_P(msg);
        }
    }
#endif

	init_capabilities();

	startup_ground();
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);

    set_mode((enum mode)g.initial_mode.get());

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void Rover::startup_ground(void)
{
    set_mode(INITIALISING);

	gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> GROUND START"));

	#if(GROUND_START_DELAY > 0)
		gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> With Delay"));
		delay(GROUND_START_DELAY * 1000);
	#endif

	//IMU ground start
	//------------------------
    //

	startup_INS_ground();

	// read the radio to set trims
	// ---------------------------
	trim_radio();

    // initialise mission library
    mission.init();

    // we don't want writes to the serial port to cause us to pause
    // so set serial ports non-blocking once we are ready to drive
    serial_manager.set_blocking_writes_all(false);

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

	gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to drive."));
}

/*
  set the in_reverse flag
  reset the throttle integrator if this changes in_reverse
 */
void Rover::set_reverse(bool reverse)
{
    if (in_reverse == reverse) {
        return;
    }
    g.pidSpeedThrottle.reset_I();    
    in_reverse = reverse;
}

void Rover::set_mode(enum mode mode)
{       

	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}

    // If we are changing out of AUTO mode reset the loiter timer
    if (control_mode == AUTO)
        loiter_time = 0;

	control_mode = mode;
    throttle_last = 0;
    throttle = 500;
    set_reverse(false);
    g.pidSpeedThrottle.reset_I();

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

        case GUIDED:
            rtl_complete = false;
            /*
              when entering guided mode we set the target as the current
              location. This matches the behaviour of the copter code.
            */
            guided_WP = current_loc;
            set_guided_WP();
            break;

		default:
			do_RTL();
			break;
	}

	if (should_log(MASK_LOG_MODE)) {
        DataFlash.Log_Write_Mode(control_mode);
    }
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool Rover::mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
    case GUIDED:
    case AUTO:
    case RTL:
        set_mode((enum mode)mode);
        return true;
    }
    return false;
}

/*
  called to set/unset a failsafe event. 
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, bool on)
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

void Rover::startup_INS_ground(void)
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Warming up ADC..."));
 	mavlink_delay(500);

	// Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
	// -----------------------
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move vehicle"));
	mavlink_delay(1000);

    ahrs.init();
	ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);

    AP_InertialSensor::Start_style style;
    if (g.skip_gyro_cal) {
        style = AP_InertialSensor::WARM_START;
    } else {
        style = AP_InertialSensor::COLD_START;
    }

	ins.init(style, ins_sample_rate);

    ahrs.reset();
}

// updates the notify state
// should be called at 50hz
void Rover::update_notify()
{
    notify.update();
}

void Rover::resetPerfData(void) {
	mainLoop_count 			= 0;
	G_Dt_max 				= 0;
	perf_mon_timer 			= millis();
}


void Rover::check_usb_mux(void)
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
    // at SERIAL1_BAUD.
    if (usb_connected) {
        serial_manager.set_console_baud(AP_SerialManager::SerialProtocol_Console, 0);
    } else {
        serial_manager.set_console_baud(AP_SerialManager::SerialProtocol_MAVLink, 0);
    }
#endif
}


void Rover::print_mode(AP_HAL::BetterStream *port, uint8_t mode)
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
        port->print_P(PSTR("Steering"));
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
uint8_t Rover::check_digital_pin(uint8_t pin)
{
    int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
    if (dpin == -1) {
        return 0;
    }
    // ensure we are in input mode
    hal.gpio->pinMode(dpin, HAL_GPIO_INPUT);

    // enable pullup
    hal.gpio->write(dpin, 1);

    return hal.gpio->read(dpin);
}

/*
  should we log a message type now?
 */
bool Rover::should_log(uint32_t mask)
{
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = hal.util->get_soft_armed() || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        in_mavlink_delay = true;
        start_logging();
        in_mavlink_delay = false;
    }
    return ret;
}

/*
  send FrSky telemetry. Should be called at 5Hz by scheduler
 */
#if FRSKY_TELEM_ENABLED == ENABLED
void Rover::frsky_telemetry_send(void)
{
    frsky_telemetry.send_frames((uint8_t)control_mode);
}
#endif
