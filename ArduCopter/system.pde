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
static int8_t   planner_mode(uint8_t argc, const Menu::arg *argv);      // in planner.pde

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv)
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
    {"logs",                process_logs},
    {"setup",               setup_mode},
    {"test",                test_mode},
    {"help",                main_menu_help},
    {"planner",             planner_mode}
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
    // the MAVLink protocol efficiently
    //
    Serial.begin(SERIAL0_BAUD, 128, 256);

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    Serial1.begin(38400, 256, 16);
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
    DDRL |= B00000100;                                                  // Set Port L, pin 2 to output for the relay
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
    gcs0.init(&Serial);

#if USB_MUX_PIN > 0
    if (!usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        Serial.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#else
    // we have a 2nd serial port for telemetry
    Serial3.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 256);
    gcs3.init(&Serial3);
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

#ifdef RADIO_OVERRIDE_DEFAULTS
    {
        int16_t rc_override[8] = RADIO_OVERRIDE_DEFAULTS;
        APM_RC.setHIL(rc_override);
    }
#endif

#if FRAME_CONFIG == HELI_FRAME
    motors.servo_manual = false;
    motors.init_swash();              // heli initialisation
#endif

    RC_Channel::set_apm_rc(&APM_RC);
    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs

    timer_scheduler.init( &isr_registry );

    // initialise the analog port reader
    AP_AnalogSource_Arduino::init_timer(&timer_scheduler);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    adc.Init(&timer_scheduler);           // APM ADC library initialization
 #endif // CONFIG_ADC

    barometer.init(&timer_scheduler);

#endif // HIL_MODE

    // Do GPS init
    g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(GPS::GPS_ENGINE_AIRBORNE_1G);
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
        digitalWrite(A_LED_PIN, LED_ON);                        // turn on setup-mode LED
        Serial.printf_P(PSTR("\nCLI:\n\n"));
        run_cli();
    }
#else
    Serial.printf_P(PSTR("\nPress ENTER 3 times for CLI\n\n"));
#endif // CLI_ENABLED

    GPS_enabled = false;

#if HIL_MODE == HIL_MODE_DISABLED
    // Read in the GPS
    for (byte counter = 0;; counter++) {
        g_gps->update();
        if (g_gps->status() != 0) {
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
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

    // initialize commands
    // -------------------
    init_commands();

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();

    // init the Z damopener
    // --------------------
#if ACCEL_ALT_HOLD != 0
    init_z_damper();
#endif


    startup_ground();

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
    Log_Write_Data(10, (float)g.pi_stabilize_roll.kP());
    Log_Write_Data(11, (float)g.pi_stabilize_roll.kI());

    Log_Write_Data(12, (float)g.pid_rate_roll.kP());
    Log_Write_Data(13, (float)g.pid_rate_roll.kI());
    Log_Write_Data(14, (float)g.pid_rate_roll.kD());
    Log_Write_Data(15, (float)g.stabilize_d.get());

    Log_Write_Data(16, (float)g.pi_loiter_lon.kP());
    Log_Write_Data(17, (float)g.pi_loiter_lon.kI());

    Log_Write_Data(18, (float)g.pid_nav_lon.kP());
    Log_Write_Data(19, (float)g.pid_nav_lon.kI());
    Log_Write_Data(20, (float)g.pid_nav_lon.kD());

    Log_Write_Data(21, (int32_t)g.auto_slew_rate.get());

    Log_Write_Data(22, (float)g.pid_loiter_rate_lon.kP());
    Log_Write_Data(23, (float)g.pid_loiter_rate_lon.kI());
    Log_Write_Data(24, (float)g.pid_loiter_rate_lon.kD());
#endif


///////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
#ifdef AP_LIMITS

    // AP_Limits modules are stored as a _linked list_. That allows us to define an infinite number of modules
    // and also to allocate no space until we actually need to.

    // The linked list looks (logically) like this
    //   [limits module] -> [first limit module] -> [second limit module] -> [third limit module] -> NULL


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

    SendDebug("\nReady to FLY ");
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

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // reset the leds
    // ---------------------------
    clear_leds();

    // when we re-calibrate the gyros,
    // all previous I values are invalid
    reset_I_all();
}

/*
 * #define YAW_HOLD             0
 * #define YAW_ACRO             1
 * #define YAW_AUTO             2
 * #define YAW_LOOK_AT_HOME     3
 *
 * #define ROLL_PITCH_STABLE    0
 * #define ROLL_PITCH_ACRO  1
 * #define ROLL_PITCH_AUTO		2
 *
 * #define THROTTLE_MANUAL  0
 * #define THROTTLE_HOLD        1
 * #define THROTTLE_AUTO		2
 *
 */

static void set_mode(byte mode)
{
    // if we don't have GPS lock
    if(home_is_set == false) {
        // THOR
        // We don't care about Home if we don't have lock yet in Toy mode
        if(mode == TOY_A || mode == TOY_M || mode == OF_LOITER) {
            // nothing
        }else if (mode > ALT_HOLD) {
            mode = STABILIZE;
        }
    }

    // nothing but OF_LOITER for OptFlow only
    if (g.optflow_enabled && GPS_enabled == false) {
        if (mode > ALT_HOLD && mode != OF_LOITER)
            mode = STABILIZE;
    }

    control_mode            = mode;
    control_mode            = constrain(control_mode, 0, NUM_MODES - 1);

    // used to stop fly_aways
    // set to false if we have low throttle
    motors.auto_armed(g.rc_3.control_in > 0);

    // clearing value used in interactive alt hold
    reset_throttle_counter = 0;

    // clearing value used to force the copter down in landing mode
    landing_boost = 0;

    // do we want to come to a stop or pass a WP?
    slow_wp = false;

    // do not auto_land if we are leaving RTL
    loiter_timer = 0;

    // if we change modes, we must clear landed flag
    land_complete   = false;

    // have we acheived the proper altitude before RTL is enabled
    rtl_reached_alt = false;
    // debug to Serial terminal
    //Serial.println(flight_mode_strings[control_mode]);

    // report the GPS and Motor arming status
    led_mode = NORMAL_LEDS;

    switch(control_mode)
    {
    case ACRO:
        yaw_mode                = YAW_HOLD;
        roll_pitch_mode = ROLL_PITCH_ACRO;
        throttle_mode   = THROTTLE_MANUAL;
        break;

    case STABILIZE:
        yaw_mode                = YAW_HOLD;
        roll_pitch_mode = ROLL_PITCH_STABLE;
        throttle_mode   = THROTTLE_MANUAL;
        break;

    case ALT_HOLD:
        yaw_mode                = ALT_HOLD_YAW;
        roll_pitch_mode = ALT_HOLD_RP;
        throttle_mode   = ALT_HOLD_THR;

        force_new_altitude(max(current_loc.alt, 100));
        break;

    case AUTO:
        yaw_mode                = AUTO_YAW;
        roll_pitch_mode = AUTO_RP;
        throttle_mode   = AUTO_THR;

        // loads the commands from where we left off
        init_commands();
        break;

    case CIRCLE:
        yaw_mode                = CIRCLE_YAW;
        roll_pitch_mode = CIRCLE_RP;
        throttle_mode   = CIRCLE_THR;
        set_next_WP(&current_loc);
        circle_WP               = next_WP;
        circle_angle    = 0;
        break;

    case LOITER:
        yaw_mode                = LOITER_YAW;
        roll_pitch_mode = LOITER_RP;
        throttle_mode   = LOITER_THR;
        set_next_WP(&current_loc);
        break;

    case POSITION:
        yaw_mode                = YAW_HOLD;
        roll_pitch_mode = ROLL_PITCH_AUTO;
        throttle_mode   = THROTTLE_MANUAL;
        set_next_WP(&current_loc);
        break;

    case GUIDED:
        yaw_mode                = YAW_AUTO;
        roll_pitch_mode = ROLL_PITCH_AUTO;
        throttle_mode   = THROTTLE_AUTO;
        next_WP                 = current_loc;
        set_next_WP(&guided_WP);
        break;

    case LAND:
        yaw_mode                = LOITER_YAW;
        roll_pitch_mode = LOITER_RP;
        throttle_mode   = THROTTLE_AUTO;
        do_land();
        break;

    case RTL:
        yaw_mode                = RTL_YAW;
        roll_pitch_mode = RTL_RP;
        throttle_mode   = RTL_THR;
        rtl_reached_alt = false;
        set_next_WP(&current_loc);
        set_new_altitude(get_RTL_alt());
        break;

    case OF_LOITER:
        yaw_mode                = OF_LOITER_YAW;
        roll_pitch_mode = OF_LOITER_RP;
        throttle_mode   = OF_LOITER_THR;
        set_next_WP(&current_loc);
        break;

    // THOR
    // These are the flight modes for Toy mode
    // See the defines for the enumerated values
    case TOY_A:
        yaw_mode                = YAW_TOY;
        roll_pitch_mode = ROLL_PITCH_TOY;
        throttle_mode   = THROTTLE_AUTO;
        wp_control              = NO_NAV_MODE;

        // save throttle for fast exit of Alt hold
        saved_toy_throttle = g.rc_3.control_in;

        // hold the current altitude
        set_new_altitude(current_loc.alt);
        break;

    case TOY_M:
        yaw_mode                = YAW_TOY;
        roll_pitch_mode = ROLL_PITCH_TOY;
        wp_control              = NO_NAV_MODE;
        throttle_mode   = THROTTLE_MANUAL;
        break;

    default:
        break;
    }

    if(failsafe) {
        // this is to allow us to fly home without interactive throttle control
        throttle_mode = THROTTLE_AUTO;
        // does not wait for us to be in high throttle, since the
        // Receiver will be outputting low throttle
        motors.auto_armed(true);
    }

    if(roll_pitch_mode <= ROLL_PITCH_ACRO) {
        // We are under manual attitude control
        // remove the navigation from roll and pitch command
        reset_nav_params();
        // remove the wind compenstaion
        reset_wind_I();
        // Clears the alt hold compensation
        reset_throttle_I();
    }

    Log_Write_Mode(control_mode);
}

static void set_failsafe(boolean mode)
{
    // only act on changes
    // -------------------
    if(failsafe != mode) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe = mode;

        if (failsafe == false) {
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
    initial_simple_bearing = ahrs.yaw_sensor;
}

static void update_throttle_cruise(int16_t tmp)
{
    if(tmp != 0) {
        g.throttle_cruise += tmp;
        reset_throttle_I();
    }

    // recalc kp
    //g.pid_throttle.kP((float)g.throttle_cruise.get() / 981.0);
    //Serial.printf("kp:%1.4f\n",kp);
}

#if CLI_SLIDER_ENABLED == ENABLED && CLI_ENABLED == ENABLED
static boolean
check_startup_for_CLI()
{
    return (digitalRead(SLIDE_SWITCH_PIN) == 0);
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
        Serial.begin(SERIAL0_BAUD);
    } else {
        Serial.begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
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

#ifndef DESKTOP_BUILD
/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    static AP_AnalogSource_Arduino vcc(ANALOG_PIN_VCC);
    return vcc.read_vcc();
}
#endif
