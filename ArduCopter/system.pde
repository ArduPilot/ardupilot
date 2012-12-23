// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED
// Functions called from the top-level menu
static int8_t   process_logs(uint8_t argc, const Menu::arg *argv);      // in Log.pde
static int8_t   setup_mode(uint8_t argc, const Menu::arg *argv);        // in setup.pde
static int8_t   test_mode(uint8_t argc, const Menu::arg *argv);         // in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
    {"logs",                process_logs},
    {"setup",               setup_mode},
    {"test",                test_mode},
    {"reboot",              reboot_board},
    {"help",                main_menu_help},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    reboot_apm();
    return 0;
}

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

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

    ap_system.usb_connected = !digitalReadFast(USB_MUX_PIN);
    if (!ap_system.usb_connected) {
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
    // the MAVLink protocol efficiently
    //
    hal.uartA->begin(SERIAL0_BAUD, 128, 256);

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);
#endif

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // setup IO pins
    pinMode(A_LED_PIN, OUTPUT);                                 // GPS status LED
    digitalWrite(A_LED_PIN, LED_OFF);

    pinMode(B_LED_PIN, OUTPUT);                         // GPS status LED
    digitalWrite(B_LED_PIN, LED_OFF);

    pinMode(C_LED_PIN, OUTPUT);                         // GPS status LED
    digitalWrite(C_LED_PIN, LED_OFF);

#if SLIDE_SWITCH_PIN > 0
    pinMode(SLIDE_SWITCH_PIN, INPUT);           // To enter interactive mode
#endif
#if CONFIG_PUSHBUTTON == ENABLED
    pinMode(PUSHBUTTON_PIN, INPUT);                     // unused
#endif

#if CONFIG_RELAY == ENABLED
    relay.init(); 
#endif

#if COPTER_LEDS == ENABLED
    pinMode(COPTER_LED_1, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_2, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_3, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_4, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_5, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_6, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_7, OUTPUT);              //Motor or GPS LED
    pinMode(COPTER_LED_8, OUTPUT);              //Motor or GPS LED

    if ( !bitRead(g.copter_leds_mode, 3) ) {
        piezo_beep();
    }

#endif


    // load parameters from EEPROM
    load_parameters();

    // init the GCS
    gcs0.init(hal.uartA);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

#if USB_MUX_PIN > 0
    if (!ap_system.usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#else
    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 256);
    gcs3.init(hal.uartC);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    mavlink_system.type = 2; //MAV_QUADROTOR;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
    }
    if (g.log_bitmask != 0) {
        DataFlash.start_new_log();
    }
#endif

#if FRAME_CONFIG == HELI_FRAME
    motors.servo_manual = false;
    motors.init_swash();              // heli initialisation
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs
    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    adc.Init();           // APM ADC library initialization
 #endif // CONFIG_ADC

    barometer.init();

#endif // HIL_MODE

    // Do GPS init
    g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);

    if(g.compass_enabled)
        init_compass();

    // init the optical flow sensor
    if(g.optflow_enabled) {
        init_optflow();
    }

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    // initialise inertial nav
    inertial_nav.init();
#endif

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
        digitalWrite(A_LED_PIN, LED_ON);                        // turn on setup-mode LED
        cliSerial->printf_P(PSTR("\nCLI:\n\n"));
        run_cli(cliSerial);
    }
#else
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer();
#endif

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

#if FRAME_CONIG == HELI_FRAME
// initialise controller filters
init_rate_controllers();
#endif // HELI_FRAME

    // initialize commands
    // -------------------
    init_commands();

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();


    startup_ground();

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

    init_ap_limits();

    cliSerial->print_P(PSTR("\nReady to FLY "));
}



///////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima,
// maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
static void init_ap_limits() {
#ifdef AP_LIMITS
    // AP_Limits modules are stored as a _linked list_. That allows us to
    // define an infinite number of modules and also to allocate no space until
    // we actually need to.

    // The linked list looks (logically) like this [limits module] -> [first
    // limit module] -> [second limit module] -> [third limit module] -> NULL


    // The details of the linked list are handled by the methods
    // modules_first, modules_current, modules_next, modules_last, modules_add
    // in limits

    limits.modules_add(&gpslock_limit);
    limits.modules_add(&geofence_limit);
    limits.modules_add(&altitude_limit);


    if (limits.debug())  {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Modules Loaded"));

        AP_Limit_Module *m = limits.modules_first();
        while (m) {
            gcs_send_text_P(SEVERITY_LOW, get_module_name(m->get_module_id()));
            m = limits.modules_next();
        }
    }
#endif
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground(void)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             flash_leds);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.init(&timer_scheduler);
    ahrs2.set_as_secondary(true);
    ahrs2.set_fast_gains(true);
#endif

    // reset the leds
    // ---------------------------
    clear_leds();

    // when we re-calibrate the gyros,
    // all previous I values are invalid
    reset_I_all();
}

// set_mode - change flight mode and perform any necessary initialisation
static void set_mode(uint8_t mode)
{
    // Switch to stabilize mode if requested mode requires a GPS lock
    if(!ap.home_is_set) {
        if (mode > ALT_HOLD && mode != TOY_A && mode != TOY_M && mode != OF_LOITER && mode != LAND) {
            mode = STABILIZE;
        }
    }

    // Switch to stabilize if OF_LOITER requested but no optical flow sensor
    if (mode == OF_LOITER && !g.optflow_enabled ) {
        mode = STABILIZE;
    }

    control_mode 	= mode;
    control_mode    = constrain(control_mode, 0, NUM_MODES - 1);

    // used to stop fly_aways
    // set to false if we have low throttle
    motors.auto_armed(g.rc_3.control_in > 0);
    set_auto_armed(g.rc_3.control_in > 0);

    // if we change modes, we must clear landed flag
    set_land_complete(false);

    // debug to Serial terminal
    //cliSerial->println(flight_mode_strings[control_mode]);

    ap.loiter_override  = false;

    // report the GPS and Motor arming status
    led_mode = NORMAL_LEDS;

    switch(control_mode)
    {
    case ACRO:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_ACRO);
        set_roll_pitch_mode(ROLL_PITCH_ACRO);
        set_throttle_mode(THROTTLE_MANUAL);
        // reset acro axis targets to current attitude
		if(g.axis_enabled){
            roll_axis 	= ahrs.roll_sensor;
            pitch_axis 	= ahrs.pitch_sensor;
            nav_yaw 	= ahrs.yaw_sensor;
        }
        break;

    case STABILIZE:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_HOLD);
        set_roll_pitch_mode(ROLL_PITCH_STABLE);
        set_throttle_mode(STABILIZE_THROTTLE);
        break;

    case ALT_HOLD:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(ALT_HOLD_YAW);
        set_roll_pitch_mode(ALT_HOLD_RP);
        set_throttle_mode(ALT_HOLD_THR);
        break;

    case AUTO:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(AUTO_YAW);
        set_roll_pitch_mode(AUTO_RP);
        set_throttle_mode(AUTO_THR);

        // loads the commands from where we left off
        init_commands();
        break;

    case CIRCLE:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;

        // start circling around current location
        set_next_WP(&current_loc);
        circle_WP       = next_WP;

        // set yaw to point to center of circle
        yaw_look_at_WP = circle_WP;
        set_yaw_mode(YAW_LOOK_AT_LOCATION);
        set_roll_pitch_mode(CIRCLE_RP);
        set_throttle_mode(CIRCLE_THR);
        circle_angle    = 0;
        break;

    case LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(LOITER_YAW);
        set_roll_pitch_mode(LOITER_RP);
        set_throttle_mode(LOITER_THR);
        set_next_WP(&current_loc);
        break;

    case POSITION:
    	ap.manual_throttle = true;
    	ap.manual_attitude = false;
        set_yaw_mode(YAW_HOLD);
        set_roll_pitch_mode(ROLL_PITCH_AUTO);
        set_throttle_mode(THROTTLE_MANUAL);
        set_next_WP(&current_loc);
        break;

    case GUIDED:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(YAW_LOOK_AT_NEXT_WP);
        set_roll_pitch_mode(ROLL_PITCH_AUTO);
        set_throttle_mode(THROTTLE_AUTO);
        next_WP = current_loc;
        set_next_WP(&guided_WP);
        break;

    case LAND:
        if( ap.home_is_set ) {
            // switch to loiter if we have gps
            ap.manual_attitude = false;
            set_yaw_mode(LOITER_YAW);
            set_roll_pitch_mode(LOITER_RP);
        }else{
            // otherwise remain with stabilize roll and pitch
            ap.manual_attitude = true;
            set_yaw_mode(YAW_HOLD);
            set_roll_pitch_mode(ROLL_PITCH_STABLE);
        }
    	ap.manual_throttle = false;
        do_land();
        break;

    case RTL:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        do_RTL();
        break;

    case OF_LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(OF_LOITER_YAW);
        set_roll_pitch_mode(OF_LOITER_RP);
        set_throttle_mode(OF_LOITER_THR);
        set_next_WP(&current_loc);
        break;

    // THOR
    // These are the flight modes for Toy mode
    // See the defines for the enumerated values
    case TOY_A:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_throttle_mode(THROTTLE_AUTO);
        wp_control      = NO_NAV_MODE;

        // save throttle for fast exit of Alt hold
        saved_toy_throttle = g.rc_3.control_in;

        break;

    case TOY_M:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        wp_control          = NO_NAV_MODE;
        set_throttle_mode(THROTTLE_HOLD);
        break;

    default:
        break;
    }

    if(ap.failsafe) {
        // this is to allow us to fly home without interactive throttle control
        set_throttle_mode(THROTTLE_AUTO);
    	ap.manual_throttle = false;

        // does not wait for us to be in high throttle, since the
        // Receiver will be outputting low throttle
        motors.auto_armed(true);
    	set_auto_armed(true);
    }

    if(ap.manual_attitude) {
        // We are under manual attitude control
        // remove the navigation from roll and pitch command
        reset_nav_params();
        // remove the wind compenstaion
        reset_wind_I();
    }

    Log_Write_Mode(control_mode);
}

static void
init_simple_bearing()
{
    initial_simple_bearing = ahrs.yaw_sensor;
    Log_Write_Data(DATA_INIT_SIMPLE_BEARING, initial_simple_bearing);
}

#if CLI_SLIDER_ENABLED == ENABLED && CLI_ENABLED == ENABLED
static bool check_startup_for_CLI()
{
    return (digitalReadFast(SLIDE_SWITCH_PIN) == 0);
}
#endif // CLI_ENABLED

/*
 *  map from a 8 bit EEPROM baud rate to a real baud rate
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
    //cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}

#if USB_MUX_PIN > 0
static void check_usb_mux(void)
{
    bool usb_check = !digitalReadFast(USB_MUX_PIN);
    if (usb_check == ap_system.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap_system.usb_connected = usb_check;
    if (ap_system.usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
}
#endif

/*
 *  called by gyro/accel init to flash LEDs so user
 *  has some mesmerising lights to watch while waiting
 */
void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on ? LED_OFF : LED_ON);
    digitalWrite(C_LED_PIN, on ? LED_ON : LED_OFF);
}

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return board_vcc_analog_source->read_latest();
}

/*
  force a software reset of the APM
 */
static void reboot_apm(void) {
    hal.scheduler->reboot();
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        cliSerial->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        cliSerial->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        cliSerial->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        cliSerial->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        cliSerial->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        cliSerial->print_P(PSTR("LOITER"));
        break;
    case RTL:
        cliSerial->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        cliSerial->print_P(PSTR("CIRCLE"));
        break;
    case POSITION:
        cliSerial->print_P(PSTR("POSITION"));
        break;
    case LAND:
        cliSerial->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        cliSerial->print_P(PSTR("OF_LOITER"));
        break;
    case TOY_M:
        cliSerial->print_P(PSTR("TOY_M"));
        break;
    case TOY_A:
        cliSerial->print_P(PSTR("TOY_A"));
        break;
    default:
        cliSerial->print_P(PSTR("---"));
        break;
    }
}
