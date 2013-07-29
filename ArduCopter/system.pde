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
#if USB_MUX_PIN > 0
    // on the APM2 board we have a mux thet switches UART0 between
    // USB and the board header. If the right ArduPPM firmware is
    // installed we can detect if USB is connected using the
    // USB_MUX_PIN
    pinMode(USB_MUX_PIN, INPUT);

    ap_system.usb_connected = !digitalRead(USB_MUX_PIN);
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
#if HIL_MODE != HIL_MODE_DISABLED
    // we need more memory for HIL, as we get a much higher packet rate
    hal.uartA->begin(SERIAL0_BAUD, 256, 256);
#else
    // use a bit less for non-HIL operation
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);
#endif

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

    relay.init(); 

#if COPTER_LEDS == ENABLED
    copter_leds_init();
#endif

    // load parameters from EEPROM
    load_parameters();

#if HIL_MODE != HIL_MODE_ATTITUDE
    barometer.init();
#endif

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
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
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
        gcs0.reset_cli_timeout();
    }
#endif

#if FRAME_CONFIG == HELI_FRAME
    motors.servo_manual = false;
    motors.init_swash();              // heli initialisation
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

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

    // initialise inertial nav
    inertial_nav.init();

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (!barometer.healthy) {
        // the barometer becomes healthy when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer();
#endif

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

#if FRAME_CONFIG == HELI_FRAME
    // initialise controller filters
    init_rate_controllers();
#endif // HELI_FRAME

    // initialize commands
    // -------------------
    init_commands();

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();

    startup_ground();

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

    cliSerial->print_P(PSTR("\nReady to FLY "));
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground(void)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             flash_leds);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

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

    // set landed flag
    set_land_complete(true);
}

// returns true if the GPS is ok and home position is set
static bool GPS_ok()
{
    if (g_gps != NULL && ap.home_is_set && g_gps->status() == GPS::GPS_OK_FIX_3D) {
        return true;
    }else{
        return false;
    }
}

// returns true or false whether mode requires GPS
static bool mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case LOITER: 
        case RTL:
        case CIRCLE:
        case POSITION:
            return true;
        default:
            return false;
    }   

    return false;
}

// set_mode - change flight mode and perform any necessary initialisation
// returns true if mode was succesfully set
// STABILIZE, ACRO and LAND can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
static bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;

    // report the GPS and Motor arming status
    // To-Do: this should be initialised somewhere else related to the LEDs
    led_mode = NORMAL_LEDS;

    switch(mode) {
        case ACRO:
            success = true;
            ap.manual_throttle = true;
            ap.manual_attitude = true;
            set_yaw_mode(ACRO_YAW);
            set_roll_pitch_mode(ACRO_RP);
            set_throttle_mode(ACRO_THR);
            set_nav_mode(NAV_NONE);
            // reset acro axis targets to current attitude
            if(g.axis_enabled){
                roll_axis   = 0;
                pitch_axis  = 0;
                nav_yaw     = 0;
            }
            break;

        case STABILIZE:
            success = true;
            ap.manual_throttle = true;
            ap.manual_attitude = true;
            set_yaw_mode(YAW_HOLD);
            set_roll_pitch_mode(ROLL_PITCH_STABLE);
            set_throttle_mode(THROTTLE_MANUAL_TILT_COMPENSATED);
            set_nav_mode(NAV_NONE);
            break;

        case ALT_HOLD:
            success = true;
            ap.manual_throttle = false;
            ap.manual_attitude = true;
            set_yaw_mode(ALT_HOLD_YAW);
            set_roll_pitch_mode(ALT_HOLD_RP);
            set_throttle_mode(ALT_HOLD_THR);
            set_nav_mode(NAV_NONE);
            break;

        case AUTO:
            // check we have a GPS and at least one mission command (note the home position is always command 0)
            if (GPS_ok() && g.command_total > 1) {
                success = true;
                ap.manual_throttle = false;
                ap.manual_attitude = false;
                // roll-pitch, throttle and yaw modes will all be set by the first nav command
                init_commands();            // clear the command queues. will be reloaded when "run_autopilot" calls "update_commands" function
            }
            break;

        case CIRCLE:
            if (GPS_ok()) {
                success = true;
                ap.manual_throttle = false;
                ap.manual_attitude = false;
                set_roll_pitch_mode(CIRCLE_RP);
                set_throttle_mode(CIRCLE_THR);
                set_nav_mode(CIRCLE_NAV);
                set_yaw_mode(CIRCLE_YAW);
            }
            break;

        case LOITER:
            if (GPS_ok()) {
                success = true;
                ap.manual_throttle = false;
                ap.manual_attitude = false;
                set_yaw_mode(LOITER_YAW);
                set_roll_pitch_mode(LOITER_RP);
                set_throttle_mode(LOITER_THR);
                set_nav_mode(LOITER_NAV);
            }
            break;

        case POSITION:
            if (GPS_ok()) {
                success = true;
                ap.manual_throttle = true;
                ap.manual_attitude = false;
                set_yaw_mode(POSITION_YAW);
                set_roll_pitch_mode(POSITION_RP);
                set_throttle_mode(POSITION_THR);
                set_nav_mode(POSITION_NAV);
            }
            break;

        case GUIDED:
            if (GPS_ok()) {
                success = true;
                ap.manual_throttle = false;
                ap.manual_attitude = false;
                set_yaw_mode(get_wp_yaw_mode(false));
                set_roll_pitch_mode(GUIDED_RP);
                set_throttle_mode(GUIDED_THR);
                set_nav_mode(GUIDED_NAV);
            }
            break;

        case LAND:
            success = true;
            // To-Do: it is messy to set manual_attitude here because the do_land function is reponsible for setting the roll_pitch_mode
            ap.manual_attitude = !GPS_ok();
            ap.manual_throttle = false;
            do_land(NULL);  // land at current location
            break;

        case RTL:
            if (GPS_ok()) {
                success = true;
                ap.manual_throttle = false;
                ap.manual_attitude = false;
                do_RTL();
            }
            break;

        case OF_LOITER:
            if (g.optflow_enabled) {
                success = true;
                ap.manual_throttle = false;
                ap.manual_attitude = false;
                set_yaw_mode(OF_LOITER_YAW);
                set_roll_pitch_mode(OF_LOITER_RP);
                set_throttle_mode(OF_LOITER_THR);
                set_nav_mode(OF_LOITER_NAV);
            }
            break;

        // THOR
        // These are the flight modes for Toy mode
        // See the defines for the enumerated values
        case TOY_A:
            success = true;
            ap.manual_throttle = false;
            ap.manual_attitude = true;
            set_yaw_mode(YAW_TOY);
            set_roll_pitch_mode(ROLL_PITCH_TOY);
            set_throttle_mode(THROTTLE_AUTO);
            set_nav_mode(NAV_NONE);

            // save throttle for fast exit of Alt hold
            saved_toy_throttle = g.rc_3.control_in;
            break;

        case TOY_M:
            success = true;
            ap.manual_throttle = false;
            ap.manual_attitude = true;
            set_yaw_mode(YAW_TOY);
            set_roll_pitch_mode(ROLL_PITCH_TOY);
            set_nav_mode(NAV_NONE);
            set_throttle_mode(THROTTLE_HOLD);
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        if(ap.manual_attitude) {
            // We are under manual attitude control so initialise nav parameter for when we next enter an autopilot mode
            reset_nav_params();
        }
        control_mode = mode;
        Log_Write_Mode(control_mode);
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLGHT_MODE,mode);
    }

    // return success or failure
    return success;
}

static void
init_simple_bearing()
{
    initial_simple_bearing = ahrs.yaw_sensor;
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, initial_simple_bearing);
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
        if(control_mode <= ACRO && g.rc_3.control_in == 0 && !ap.failsafe_radio) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && g.rc_3.control_in != 0) {
            set_auto_armed(true);
        }
    }
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
    //cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}

#if USB_MUX_PIN > 0
static void check_usb_mux(void)
{
    bool usb_check = !digitalRead(USB_MUX_PIN);
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
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case POSITION:
        port->print_P(PSTR("POSITION"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
    case TOY_M:
        port->print_P(PSTR("TOY_M"));
        break;
    case TOY_A:
        port->print_P(PSTR("TOY_A"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}
