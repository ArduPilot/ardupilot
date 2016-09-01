// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
#include "version.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Copter::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n");
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] = {
//   command		function called
//   =======        ===============
    {"logs",                MENU_FUNC(process_logs)},
    {"setup",               MENU_FUNC(setup_mode)},
    {"test",                MENU_FUNC(test_mode)},
    {"reboot",              MENU_FUNC(reboot_board)},
    {"help",                MENU_FUNC(main_menu_help)},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Copter::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Copter::run_cli(AP_HAL::UARTDriver *port)
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

static void mavlink_delay_cb_static()
{
    copter.mavlink_delay_cb();
}


static void failsafe_check_static()
{
    copter.failsafe_check();
}

void Copter::init_ardupilot()
{
    if (!hal.gpio->usb_connected()) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }

    // initialise serial port
    serial_manager.init_console();

    // init vehicle capabilties
    init_capabilities();

    cliSerial->printf("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n",
                      (unsigned)hal.util->available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // load parameters from EEPROM
    load_parameters();

    GCS_MAVLINK::set_dataflash(&DataFlash);

    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);
    
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

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();

    // setup telem slots with serial ports
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        gcs[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }

#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky, and pass a number of parameters to the library
    frsky_telemetry.init(serial_manager, FIRMWARE_STRING " " FRAME_CONFIG_STRING,
                         FRAME_MAV_TYPE,
                         &g.fs_batt_voltage, &g.fs_batt_mah, &ap.value);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // update motor interlock state
    update_using_interlock();

#if FRAME_CONFIG == HELI_FRAME
    // trad heli specific initialisation
    heli_init();
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
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.init(&DataFlash, serial_manager);

    if(g.compass_enabled)
        init_compass();

#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow);
#endif

    // init Location class
    Location_Class::set_ahrs(&ahrs);
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    Location_Class::set_terrain(&terrain);
    wp_nav.set_terrain(&terrain);
#endif
    wp_nav.set_avoidance(&avoid);

    attitude_control.parameter_sanity_check();
    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
    init_optflow();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(&DataFlash, serial_manager);
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precision landing
    init_precland();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    if (g.cli_enabled) {
        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
        cliSerial->println(msg);
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println(msg);
        }
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println(msg);
        }
    }
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs_send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);

    // initialise rangefinder
    init_rangefinder();

    // initialise AP_RPM library
    rpm_sensor.init();

    // initialise mission library
    mission.init();

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();

    startup_INS_ground();

    // set landed flags
    set_land_complete(true);
    set_land_complete_maybe(true);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    failsafe_enable();

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

    cliSerial->print("\nReady to FLY ");

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Copter::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if successfully calibrated
bool Copter::calibrate_gyros()
{
    // gyro offset calibration
    copter.ins.init_gyro();

    // reset ahrs gyro bias
    if (copter.ins.gyro_calibrated_ok_all()) {
        copter.ahrs.reset_gyro_drift();
        return true;
    }

    return false;
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Copter::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Copter::optflow_position_ok()
{
#if OPTFLOW != ENABLED
    return false;
#else
    // return immediately if optflow is not enabled or EKF not used
    if (!optflow.enabled() || !ahrs.have_inertial_nav()) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors.armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
#endif
}

// update_auto_armed - update status of auto_armed flag
void Copter::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
#if FRAME_CONFIG == HELI_FRAME 
        // if helicopters are on the ground, and the motor is switched off, auto-armed should be false
        // so that rotor runup is checked again before attempting to take-off
        if(ap.land_complete && !motors.rotor_runup_complete()) {
            set_auto_armed(false);
        }
#endif // HELI_FRAME
    }else{
        // arm checks
        
#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors.armed() && !ap.throttle_zero && motors.rotor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        // if motors are armed and we are in throw mode, then auto_ermed should be true
        if(motors.armed() && (!ap.throttle_zero || control_mode == THROW)) {
            set_auto_armed(true);
        }
#endif // HELI_FRAME
    }
}

void Copter::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap.usb_connected = usb_check;
}

/*
  should we log a message type now?
 */
bool Copter::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = motors.armed() || DataFlash.log_while_disarmed();
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}
