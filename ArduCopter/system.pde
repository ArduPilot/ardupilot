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
    hal.scheduler->reboot(false);
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

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors.armed()) {
        motors.armed(false);
        motors.output();
    }

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void init_ardupilot()
{
    if (!hal.gpio->usb_connected()) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }

    // Console serial port
    //
    // The console port buffers are defined to be sufficiently large to support
    // the MAVLink protocol efficiently
    //
#if HIL_MODE != HIL_MODE_DISABLED
    // we need more memory for HIL, as we get a much higher packet rate
    hal.uartA->begin(map_baudrate(g.serial0_baud), 256, 256);
#else
    // use a bit less for non-HIL operation
    hal.uartA->begin(map_baudrate(g.serial0_baud), 512, 128);
#endif

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);
#endif

#if GPS2_ENABLE
    if (hal.uartE != NULL) {
        hal.uartE->begin(38400, 256, 16);
    }
#endif

    cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    /*
      run the timer a bit slower on APM2 to reduce the interrupt load
      on the CPU
     */
    hal.scheduler->set_timer_speed(500);
#endif

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // load parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // init EPM cargo gripper
#if EPM_ENABLED == ENABLED
    epm.init();
#endif

    // initialise notify system
    // disable external leds if epm is enabled because of pin conflict on the APM
    notify.init(true);

    // initialise battery monitor
    battery.init();
    
    rssi_analog_source      = hal.analogin->channel(g.rssi_pin);

    barometer.init();

    // init the GCS
    gcs[0].init(hal.uartA);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();

#if CONFIG_HAL_BOARD != HAL_BOARD_APM2
    // we have a 2nd serial port for telemetry on all boards except
    // APM2. We actually do have one on APM2 but it isn't necessary as
    // a MUX is used
    gcs[1].setup_uart(hal.uartC, map_baudrate(g.serial1_baud), 128, 128);
#endif

#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (g.serial2_protocol == SERIAL2_FRSKY_DPORT || 
        g.serial2_protocol == SERIAL2_FRSKY_SPORT) {
        frsky_telemetry.init(hal.uartD, g.serial2_protocol);
    } else {
        gcs[2].setup_uart(hal.uartD, map_baudrate(g.serial2_baud), 128, 128);
    }
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    mavlink_system.type = 2; //MAV_QUADROTOR;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_HIGH, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_HIGH, PSTR("ERASING LOGS"));
        do_erase_logs();
        gcs[0].reset_cli_timeout();
    }
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

    // initialise which outputs Servo and Relay events can use
    ServoRelayEvents.set_channel_mask(~motors.get_motor_mask());

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    apm1_adc.Init();           // APM ADC library initialization
 #endif // CONFIG_ADC

    // Do GPS init
    gps.init(&DataFlash);

    if(g.compass_enabled)
        init_compass();

    // initialise attitude and position controllers
    attitude_control.set_dt(MAIN_LOOP_SECONDS);
    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
    if(g.optflow_enabled) {
        init_optflow();
    }

    // initialise inertial nav
    inertial_nav.init();

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs[1].initialised) {
        hal.uartC->println_P(msg);
    }
    if (num_gcs > 2 && gcs[2].initialised) {
        hal.uartD->println_P(msg);
    }
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

    // initialise mission library
    mission.init();

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();

#if FRAME_CONFIG == HELI_FRAME
    // trad heli specific initialisation
    heli_init();
#endif

    startup_ground(true);

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
  #ifdef LOG_FROM_STARTUP
    // start dataflash
    start_logging();
  #endif
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    hal.uartA->set_blocking_writes(false);
    hal.uartB->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);
    if (hal.uartD != NULL) {
        hal.uartD->set_blocking_writes(false);
    }

    cliSerial->print_P(PSTR("\nReady to FLY "));

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground(bool force_gyro_cal)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(force_gyro_cal?AP_InertialSensor::COLD_START:AP_InertialSensor::WARM_START,
             ins_sample_rate);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // set landed flag
    set_land_complete(true);
    set_land_complete_maybe(true);
}

// returns true if the GPS is ok and home position is set
static bool GPS_ok()
{
    if (ap.home_is_set && gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
        !gps_glitch.glitching() && !failsafe.gps &&
        !ekf_check_state.bad_compass && !failsafe.ekf) {
        return true;
    }else{
        return false;
    }
}

// update_auto_armed - update status of auto_armed flag
static void update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(manual_flight_mode(control_mode) && g.rc_3.control_in == 0 && !failsafe.radio) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        
#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors.armed() && g.rc_3.control_in != 0 && motors.motor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && g.rc_3.control_in != 0) {
            set_auto_armed(true);
        }
#endif // HELI_FRAME
    }
}

static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap.usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (ap.usb_connected) {
        hal.uartA->begin(map_baudrate(g.serial0_baud));
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud));
    }
#endif
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
