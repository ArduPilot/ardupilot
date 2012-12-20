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
                         "  logs        log readback/setup mode\n"
                         "  setup       setup mode\n"
                         "  test        test mode\n"
                         "  reboot      reboot to flight mode\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] PROGMEM = {
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
    // the MAVLink protocol efficiently
    //
    hal.uartA->begin(SERIAL0_BAUD, 128, SERIAL_BUFSIZE);

    // GPS serial port.
    //
    // standard gps running
    hal.uartB->begin(38400, 256, 16);

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());


    //
    // Check the EEPROM format version before loading any parameters from EEPROM
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
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD),
            128, SERIAL_BUFSIZE);
    gcs3.init(hal.uartC);
#endif

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        gcs0.reset_cli_timeout();
    }
    if (g.log_bitmask != 0) {
        DataFlash.start_new_log();
    }
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE

 #if CONFIG_ADC == ENABLED
    adc.Init();      // APM ADC library initialization
 #endif

    barometer.init();

    if (g.compass_enabled==true) {
        compass.set_orientation(MAG_ORIENTATION);                                                       // set compass's orientation on aircraft
        if (!compass.init() || !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }
#endif

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

#if APM_CONTROL == ENABLED
    // the axis controllers need access to the AHRS system
    g.rollController.set_ahrs(&ahrs);
    g.pitchController.set_ahrs(&ahrs);
    g.yawController.set_ahrs(&ahrs);
#endif

	// Do GPS init
	g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);

    //mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
    mavlink_system.compid = 1;          //MAV_COMP_ID_IMU;   // We do not check for comp id
    mavlink_system.type = MAV_TYPE_FIXED_WING;

    // Set initial values for no override
    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs

    pinMode(C_LED_PIN, OUTPUT);                         // GPS status LED
    pinMode(A_LED_PIN, OUTPUT);                         // GPS status LED
    pinMode(B_LED_PIN, OUTPUT);                         // GPS status LED
#if CONFIG_RELAY == ENABLED
    relay.init();
#endif

#if FENCE_TRIGGERED_PIN > 0
    pinMode(FENCE_TRIGGERED_PIN, OUTPUT);
    digitalWrite(FENCE_TRIGGERED_PIN, LOW);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif

    if (ENABLE_AIR_START == 1) {
        // Perform an air start and get back to flying
        gcs_send_text_P(SEVERITY_LOW,PSTR("<init_ardupilot> AIR START"));

        // Get necessary data from EEPROM
        //----------------
        //read_EEPROM_airstart_critical();
#if HIL_MODE != HIL_MODE_ATTITUDE
        ins.init(AP_InertialSensor::WARM_START, 
                 ins_sample_rate,
                 flash_leds);

        ahrs.init();
        ahrs.set_fly_forward(true);
#endif

        // This delay is important for the APM_RC library to work.
        // We need some time for the comm between the 328 and 1280 to be established.
        int old_pulse = 0;
        while (millis()<=1000 
            && (abs(old_pulse - hal.rcin->read(g.flight_mode_channel)) > 5
               || hal.rcin->read(g.flight_mode_channel) == 1000
               || hal.rcin->read(g.flight_mode_channel) == 1200))
        {
            old_pulse = hal.rcin->read(g.flight_mode_channel);
            delay(25);
        }
        g_gps->update();

        if (g.log_bitmask & MASK_LOG_CMD)
            Log_Write_Startup(TYPE_AIRSTART_MSG);
        reload_commands_airstart();                     // Get set to resume AUTO from where we left off

    }else {
        startup_ground();
        if (g.log_bitmask & MASK_LOG_CMD)
            Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    }

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

#if (GROUND_START_DELAY > 0)
    gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> With Delay"));
    delay(GROUND_START_DELAY * 1000);
#endif

    // Makes the servos wiggle
    // step 1 = 1 wiggle
    // -----------------------
    demo_servos(1);

    //INS ground start
    //------------------------
    //
    startup_INS_ground(false);

    // read the radio to set trims
    // ---------------------------
    trim_radio();               // This was commented out as a HACK.  Why?  I don't find a problem.

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialize commands
    // -------------------
    init_commands();

    // Makes the servos wiggle - 3 times signals ready to fly
    // -----------------------
    demo_servos(3);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    hal.uartC->set_blocking_writes(false);
    if (gcs3.initialised) {
        hal.uartC->set_blocking_writes(false);
    }

    gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to FLY."));
}

static void set_mode(enum FlightMode mode)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    if(g.auto_trim > 0 && control_mode == MANUAL)
        trim_control_surfaces();

    control_mode = mode;
    crash_timer = 0;

    switch(control_mode)
    {
    case INITIALISING:
    case MANUAL:
    case CIRCLE:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
        break;

    case AUTO:
        update_auto();
        break;

    case RTL:
        do_RTL();
        break;

    case LOITER:
        do_loiter_at_location();
        break;

    case GUIDED:
        set_guided_WP();
        break;

    default:
        do_RTL();
        break;
    }

    // if in an auto-throttle mode, start with throttle suppressed for
    // safety. suppress_throttle() will unsupress it when appropriate
    if (control_mode == CIRCLE || control_mode >= FLY_BY_WIRE_B) {
        throttle_suppressed = true;
    }

    if (g.log_bitmask & MASK_LOG_MODE)
        Log_Write_Mode(control_mode);
}

static void check_long_failsafe()
{
    // only act on changes
    // -------------------
    if(failsafe != FAILSAFE_LONG  && failsafe != FAILSAFE_GCS) {
        if(rc_override_active && millis() - rc_override_fs_timer > FAILSAFE_LONG_TIME) {
            failsafe_long_on_event(FAILSAFE_LONG);
        }
        if(!rc_override_active && failsafe == FAILSAFE_SHORT && millis() - ch3_failsafe_timer > FAILSAFE_LONG_TIME) {
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
    if(failsafe == FAILSAFE_NONE) {
        if(ch3_failsafe) {                                              // The condition is checked and the flag ch3_failsafe is set in radio.pde
            failsafe_short_on_event(FAILSAFE_SHORT);
        }
    }

    if(failsafe == FAILSAFE_SHORT) {
        if(!ch3_failsafe) {
            failsafe_short_off_event();
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
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move plane"));
    mavlink_delay(1000);

    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
#if HIL_MODE == HIL_MODE_DISABLED
    if (force_accel_level || g.manual_level == 0) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station
        ins.init_accel(flash_leds);
    }
#endif
    ahrs.set_fly_forward(true);
    ahrs.reset();

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed();
    } else {
        gcs_send_text_P(SEVERITY_LOW,PSTR("NO airspeed"));
    }

#endif
    digitalWrite(B_LED_PIN, LED_ON);                    // Set LED B high to indicate INS ready
    digitalWrite(A_LED_PIN, LED_OFF);
    digitalWrite(C_LED_PIN, LED_OFF);
}


static void update_GPS_light(void)
{
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    switch (g_gps->status()) {
    case (2):
        digitalWrite(C_LED_PIN, LED_ON);                  //Turn LED C on when gps has valid fix.
        break;

    case (1):
        if (g_gps->valid_read == true) {
            GPS_light = !GPS_light;                     // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
            if (GPS_light) {
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
    mainLoop_count                  = 0;
    G_Dt_max                                = 0;
    ahrs.renorm_range_count         = 0;
    ahrs.renorm_blowup_count = 0;
    gps_fix_count                   = 0;
    pmTest1                                 = 0;
    perf_mon_timer                  = millis();
}


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
    cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}


static void check_usb_mux(void)
{
#if USB_MUX_PIN > 0
    bool usb_check = !digitalRead(USB_MUX_PIN);
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#endif
}


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
    return vcc_pin->read_latest();
}


/*
  force a software reset of the APM
 */
static void reboot_apm(void)
{
    hal.scheduler->reboot();
    while (1);
}


static void
print_flight_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        cliSerial->println_P(PSTR("Manual"));
        break;
    case CIRCLE:
        cliSerial->println_P(PSTR("Circle"));
        break;
    case STABILIZE:
        cliSerial->println_P(PSTR("Stabilize"));
        break;
    case TRAINING:
        cliSerial->println_P(PSTR("Training"));
        break;
    case FLY_BY_WIRE_A:
        cliSerial->println_P(PSTR("FBW_A"));
        break;
    case FLY_BY_WIRE_B:
        cliSerial->println_P(PSTR("FBW_B"));
        break;
    case AUTO:
        cliSerial->println_P(PSTR("AUTO"));
        break;
    case RTL:
        cliSerial->println_P(PSTR("RTL"));
        break;
    case LOITER:
        cliSerial->println_P(PSTR("Loiter"));
        break;
    default:
        cliSerial->println_P(PSTR("---"));
        break;
    }
}

static void print_comma(void)
{
    cliSerial->print_P(PSTR(","));
}


