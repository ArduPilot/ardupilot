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
    // the MAVLink protocol efficiently
    //
    hal.uartA->begin(SERIAL0_BAUD, 128, SERIAL_BUFSIZE);

    cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());


    //
    // Check the EEPROM format version before loading any parameters from EEPROM
    //
    load_parameters();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // this must be before BoardConfig.init() so if
    // BRD_SAFETYENABLE==0 then we don't have safety off yet
    setup_failsafe_mixing();
#endif

    BoardConfig.init();

    // allow servo set on all channels except first 4
    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();

    // reset the uartA baud rate after parameter load
    hal.uartA->begin(map_baudrate(g.serial0_baud));

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // initialise rangefinder
    init_rangefinder();

    // initialise battery monitoring
    battery.init();

    // init the GCS
    gcs[0].init(hal.uartA);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
    gcs[1].setup_uart(hal.uartC, map_baudrate(g.serial1_baud), 128, SERIAL1_BUFSIZE);

#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (g.serial2_protocol == SERIAL2_FRSKY_DPORT || 
        g.serial2_protocol == SERIAL2_FRSKY_SPORT) {
        frsky_telemetry.init(hal.uartD, g.serial2_protocol);
    } else {
        gcs[2].setup_uart(hal.uartD, map_baudrate(g.serial2_baud), 128, SERIAL2_BUFSIZE);
    }
#endif

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        for (uint8_t i=0; i<num_gcs; i++) {
            gcs[i].reset_cli_timeout();
        }
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    apm1_adc.Init();      // APM ADC library initialization
#endif

    // initialise airspeed sensor
    airspeed.init();

    if (g.compass_enabled==true) {
        if (!compass.init() || !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }
    
    // make optflow available to libraries
    ahrs.set_optflow(&optflow);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

    // GPS Initialization
    gps.init(&DataFlash);

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs

    relay.init();

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs[1].initialised) {
        hal.uartC->println_P(msg);
    }
    if (num_gcs > 2 && gcs[2].initialised) {
        hal.uartD->println_P(msg);
    }

    startup_ground();
    if (should_log(MASK_LOG_CMD))
        Log_Write_Startup(TYPE_GROUNDSTART_MSG);

    // choose the nav controller
    set_nav_controller();

    set_mode(MANUAL);

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();

    // initialise sensor
#if OPTFLOW == ENABLED
    optflow.init();
#endif

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
    if (!g.skip_gyro_cal) {
        demo_servos(1);
    }

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

    // initialise mission library
    mission.init();

    // Makes the servos wiggle - 3 times signals ready to fly
    // -----------------------
    if (!g.skip_gyro_cal) {
        demo_servos(3);
    }

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    failsafe.last_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    hal.uartA->set_blocking_writes(false);
    hal.uartB->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);
    if (hal.uartD != NULL) {
        hal.uartD->set_blocking_writes(false);
    }

    gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to FLY."));
}

static enum FlightMode get_previous_mode() {
    return previous_mode; 
}

static void set_mode(enum FlightMode mode)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    if(g.auto_trim > 0 && control_mode == MANUAL)
        trim_control_surfaces();

    // perform any cleanup required for prev flight mode
    exit_mode(control_mode);

    // cancel inverted flight
    auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    auto_state.next_wp_no_crosstrack = true;

    // reset landing check
    auto_state.checked_for_autoland = false;

    // reset go around command
    auto_state.commanded_go_around = false;

    // zero locked course
    steer_state.locked_course_err = 0;

    // set mode
    previous_mode = control_mode;
    control_mode = mode;

    if (previous_mode == AUTOTUNE && control_mode != AUTOTUNE) {
        // restore last gains
        autotune_restore();
    }

    // zero initial pitch and highest airspeed on mode change
    auto_state.highest_airspeed = 0;
    auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    auto_state.fbwa_tdrag_takeoff_mode = false;

    switch(control_mode)
    {
    case INITIALISING:
        auto_throttle_mode = true;
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
        auto_throttle_mode = false;
        break;

    case AUTOTUNE:
        auto_throttle_mode = false;
        autotune_start();
        break;

    case ACRO:
        auto_throttle_mode = false;
        acro_state.locked_roll = false;
        acro_state.locked_pitch = false;
        break;

    case CRUISE:
        auto_throttle_mode = true;
        cruise_state.locked_heading = false;
        cruise_state.lock_timer_ms = 0;
        set_target_altitude_current();
        break;

    case FLY_BY_WIRE_B:
        auto_throttle_mode = true;
        set_target_altitude_current();
        break;

    case CIRCLE:
        // the altitude to circle at is taken from the current altitude
        auto_throttle_mode = true;
        next_WP_loc.alt = current_loc.alt;
        break;

    case AUTO:
        auto_throttle_mode = true;
        next_WP_loc = prev_WP_loc = current_loc;
        // start or resume the mission, based on MIS_AUTORESET
        mission.start_or_resume();
        break;

    case RTL:
        auto_throttle_mode = true;
        prev_WP_loc = current_loc;
        do_RTL();
        break;

    case LOITER:
        auto_throttle_mode = true;
        do_loiter_at_location();
        break;

    case GUIDED:
        auto_throttle_mode = true;
        guided_throttle_passthru = false;
        set_guided_WP();
        break;
    }

    // start with throttle suppressed in auto_throttle modes
    throttle_suppressed = auto_throttle_mode;

    if (should_log(MASK_LOG_MODE))
        Log_Write_Mode(control_mode);

    // reset attitude integrators on mode change
    rollController.reset_I();
    pitchController.reset_I();
    yawController.reset_I();    
    steerController.reset_I();    
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
static bool mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
    case CIRCLE:
    case STABILIZE:
    case TRAINING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case AUTO:
    case RTL:
    case LOITER:
        set_mode((enum FlightMode)mode);
        return true;
    }
    return false;
}

// exit_mode - perform any cleanup required when leaving a flight mode
static void exit_mode(enum FlightMode mode)
{
    // stop mission when we leave auto
    if (mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
    }
}

static void check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if(failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS) {
        if (failsafe.state == FAILSAFE_SHORT &&
                   (tnow - failsafe.ch3_timer_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_LONG);
        } else if (g.gcs_heartbeat_fs_enabled != GCS_FAILSAFE_OFF && 
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs[0].last_radio_status_remrssi_ms != 0 &&
                   (tnow - gcs[0].last_radio_status_remrssi_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        }
    } else {
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - failsafe.last_heartbeat_ms) < g.short_fs_timeout*1000) {
            failsafe.state = FAILSAFE_NONE;
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.ch3_failsafe) {
            failsafe.state = FAILSAFE_NONE;
        }
    }
}

static void check_short_failsafe()
{
    // only act on changes
    // -------------------
    if(failsafe.state == FAILSAFE_NONE) {
        if(failsafe.ch3_failsafe) {                                              // The condition is checked and the flag ch3_failsafe is set in radio.pde
            failsafe_short_on_event(FAILSAFE_SHORT);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.ch3_failsafe) {
            failsafe_short_off_event();
        }
    }
}


static void startup_INS_ground(bool do_accel_init)
{
#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        hal.scheduler->delay(1000);
    }
    
    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    AP_InertialSensor::Start_style style;
    if (g.skip_gyro_cal && !do_accel_init) {
        style = AP_InertialSensor::WARM_START;
    } else {
        style = AP_InertialSensor::COLD_START;
    }

    if (style == AP_InertialSensor::COLD_START) {
        gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move plane"));
        mavlink_delay(100);
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
    ahrs.set_wind_estimation(true);

    ins.init(style, ins_sample_rate);
    if (do_accel_init) {
        ins.init_accel();
        ahrs.set_trim(Vector3f(0, 0, 0));
    }
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    init_barometer();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed(true);
    } else {
        gcs_send_text_P(SEVERITY_LOW,PSTR("NO airspeed"));
    }
}

// updates the status of the notify objects
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

static void resetPerfData(void) 
{
    mainLoop_count                  = 0;
    G_Dt_max                        = 0;
    G_Dt_min                        = 0;
    perf_mon_timer                  = millis();
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
    // at SERIAL1_BAUD.
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud));
    }
#endif
}


static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->print_P(PSTR("Manual"));
        break;
    case CIRCLE:
        port->print_P(PSTR("Circle"));
        break;
    case STABILIZE:
        port->print_P(PSTR("Stabilize"));
        break;
    case TRAINING:
        port->print_P(PSTR("Training"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case FLY_BY_WIRE_A:
        port->print_P(PSTR("FBW_A"));
        break;
    case AUTOTUNE:
        port->print_P(PSTR("AUTOTUNE"));
        break;
    case FLY_BY_WIRE_B:
        port->print_P(PSTR("FBW_B"));
        break;
    case CRUISE:
        port->print_P(PSTR("CRUISE"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case LOITER:
        port->print_P(PSTR("Loiter"));
        break;
    case GUIDED:
        port->print_P(PSTR("Guided"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

static void print_comma(void)
{
    cliSerial->print_P(PSTR(","));
}


/*
  write to a servo
 */
static void servo_write(uint8_t ch, uint16_t pwm)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (!g.hil_servos) {
        if (ch < 8) {
            RC_Channel::rc_channel(ch)->radio_out = pwm;
        }
        return;
    }
#endif
    hal.rcout->enable_ch(ch);
    hal.rcout->write(ch, pwm);
}

/*
  should we log a message type now?
 */
static bool should_log(uint32_t mask)
{
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = ahrs.get_armed() || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
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
static void telemetry_send(void)
{
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.send_frames((uint8_t)control_mode, 
                                (AP_Frsky_Telem::FrSkyProtocol)g.serial2_protocol.get());
#endif
}


/*
  return throttle percentage from 0 to 100
 */
static uint8_t throttle_percentage(void)
{
    // to get the real throttle we need to use norm_output() which
    // returns a number from -1 to 1.
    return constrain_int16(50*(channel_throttle->norm_output()+1), 0, 100);
}
