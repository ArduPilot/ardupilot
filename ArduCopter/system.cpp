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
    hal.scheduler->register_delay_callback(nullptr, 5);

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors->armed()) {
        motors->armed(false);
        motors->output();
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

    // initialise stats module
    g2.stats.init();

    gcs().set_dataflash(&DataFlash);

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    
    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs_chan[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);
    
    BoardConfig.init();

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

    // initialise notify system
    notify.init(true);
    notify_flight_mode(control_mode);

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
        gcs_chan[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }

#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky, and pass a number of parameters to the library
    char firmware_buf[50];
    snprintf(firmware_buf, sizeof(firmware_buf), FIRMWARE_STRING " %s", get_frame_string());
    frsky_telemetry.init(serial_manager, firmware_buf,
                         get_frame_mav_type(),
                         &g.fs_batt_voltage, &g.fs_batt_mah, &ap.value);
#endif

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

    // default frame class to match firmware if possible
    set_default_frame_class();

    // allocate the motors class
    allocate_motors();

    init_rc_out();              // sets up motors and output to escs

    // initialise which outputs Servo and Relay events can use
    ServoRelayEvents.set_channel_mask(~motors->get_motor_mask());

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // give AHRS the rnage beacon sensor
    ahrs.set_beacon(&g2.beacon);

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
    wp_nav->set_terrain(&terrain);
#endif
#if AC_AVOID_ENABLED == ENABLED
    wp_nav->set_avoidance(&avoid);
#endif

    attitude_control->parameter_sanity_check();
    pos_control->set_dt(MAIN_LOOP_SECONDS);

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
        cliSerial->printf("%s\n", msg);
        if (gcs_chan[1].initialised && (gcs_chan[1].get_uart() != nullptr)) {
            gcs_chan[1].get_uart()->printf("%s\n", msg);
        }
        if (num_gcs > 2 && gcs_chan[2].initialised && (gcs_chan[2].get_uart() != nullptr)) {
            gcs_chan[2].get_uart()->printf("%s\n", msg);
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

    // init proximity sensor
    init_proximity();

    // init beacons used for non-gps position estimation
    init_beacon();

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

    cliSerial->printf("\nReady to FLY ");

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
    if (!motors->armed()) {
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
    if (!motors->armed()) {
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
        if(!motors->armed()) {
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
        if(ap.land_complete && !motors->rotor_runup_complete()) {
            set_auto_armed(false);
        }
#endif // HELI_FRAME
    }else{
        // arm checks
        
#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors->armed() && !ap.throttle_zero && motors->rotor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        // if motors are armed and we are in throw mode, then auto_ermed should be true
        if(motors->armed() && (!ap.throttle_zero || control_mode == THROW)) {
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
    bool ret = motors->armed() || DataFlash.log_while_disarmed();
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}

// default frame_class to match firmware if possible
void Copter::set_default_frame_class()
{
    if (FRAME_CONFIG == HELI_FRAME &&
        g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_DUAL) {
        g2.frame_class.set(AP_Motors::MOTOR_FRAME_HELI);
    }
}

// return MAV_TYPE corresponding to frame class
uint8_t Copter::get_frame_mav_type()
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_UNDEFINED:
            return MAV_TYPE_QUADROTOR;
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
            return MAV_TYPE_HEXAROTOR;
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
            return MAV_TYPE_OCTOROTOR;
        case AP_Motors::MOTOR_FRAME_HELI:
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            return MAV_TYPE_HELICOPTER;
        case AP_Motors::MOTOR_FRAME_TRI:
            return MAV_TYPE_TRICOPTER;
        case AP_Motors::MOTOR_FRAME_SINGLE:
        case AP_Motors::MOTOR_FRAME_COAX:
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            return MAV_TYPE_COAXIAL;
    }
    // unknown frame so return generic
    return MAV_TYPE_GENERIC;
}

// return string corresponding to frame_class
const char* Copter::get_frame_string()
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_QUAD:
            return "QUAD";
        case AP_Motors::MOTOR_FRAME_HEXA:
            return "HEXA";
        case AP_Motors::MOTOR_FRAME_Y6:
            return "Y6";
        case AP_Motors::MOTOR_FRAME_OCTA:
            return "OCTA";
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
            return "OCTA_QUAD";
        case AP_Motors::MOTOR_FRAME_HELI:
            return "HELI";
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            return "HELI_DUAL";
        case AP_Motors::MOTOR_FRAME_TRI:
            return "TRI";
        case AP_Motors::MOTOR_FRAME_SINGLE:
            return "SINGLE";
        case AP_Motors::MOTOR_FRAME_COAX:
            return "COAX";
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            return "TAILSITTER";
        case AP_Motors::MOTOR_FRAME_UNDEFINED:
        default:
            return "UNKNOWN";
    }
}

/*
  allocate the motors class
 */
void Copter::allocate_motors(void)
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
#if FRAME_CONFIG != HELI_FRAME
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        default:
            motors = new AP_MotorsMatrix(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = new AP_MotorsTri(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsTri::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors = new AP_MotorsSingle(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_COAX:
            motors = new AP_MotorsCoax(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors = new AP_MotorsTailsitter(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
#else // FRAME_CONFIG == HELI_FRAME
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors = new AP_MotorsHeli_Dual(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
            
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors = new AP_MotorsHeli_Single(MAIN_LOOP_RATE);
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;            
#endif
    }
    if (motors == nullptr) {
        AP_HAL::panic("Unable to allocate FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    AP_AHRS_View *ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_HAL::panic("Unable to allocate AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo *ac_var_info;

#if FRAME_CONFIG != HELI_FRAME
    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, MAIN_LOOP_SECONDS);
    ac_var_info = AC_AttitudeControl_Multi::var_info;
#else
    attitude_control = new AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors, MAIN_LOOP_SECONDS);
    ac_var_info = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_HAL::panic("Unable to allocate AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);
        
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control,
                                    g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                                    g.p_pos_xy, g.pi_vel_xy);
    if (pos_control == nullptr) {
        AP_HAL::panic("Unable to allocate PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (wp_nav == nullptr) {
        AP_HAL::panic("Unable to allocate WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (wp_nav == nullptr) {
        AP_HAL::panic("Unable to allocate CircleNav");
    }
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file();
    
    // now setup some frame-class specific defaults
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
    case AP_Motors::MOTOR_FRAME_Y6:
        attitude_control->get_rate_roll_pid().kP().set_default(0.1);
        attitude_control->get_rate_roll_pid().kD().set_default(0.006);
        attitude_control->get_rate_pitch_pid().kP().set_default(0.1);
        attitude_control->get_rate_pitch_pid().kD().set_default(0.006);
        attitude_control->get_rate_yaw_pid().kP().set_default(0.15);
        attitude_control->get_rate_yaw_pid().kI().set_default(0.015);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        attitude_control->get_rate_yaw_pid().filt_hz().set_default(100);
        break;
    default:
        break;
    }

    if (upgrading_frame_params) {
        // do frame specific upgrade. This is only done the first time we run the new firmware
#if FRAME_CONFIG == HELI_FRAME
        SRV_Channels::upgrade_motors_servo(Parameters::k_param_motors, 12, CH_1);
        SRV_Channels::upgrade_motors_servo(Parameters::k_param_motors, 13, CH_2);
        SRV_Channels::upgrade_motors_servo(Parameters::k_param_motors, 14, CH_3);
        SRV_Channels::upgrade_motors_servo(Parameters::k_param_motors, 15, CH_4);
#else
        if (g2.frame_class == AP_Motors::MOTOR_FRAME_TRI) {
            const AP_Param::ConversionInfo tri_conversion_info[] = {
                { Parameters::k_param_motors, 32, AP_PARAM_INT16, "SERVO7_TRIM" },
                { Parameters::k_param_motors, 33, AP_PARAM_INT16, "SERVO7_MIN" },
                { Parameters::k_param_motors, 34, AP_PARAM_INT16, "SERVO7_MAX" },
                { Parameters::k_param_motors, 35, AP_PARAM_FLOAT, "MOT_YAW_SV_ANGLE" },
            };
            // we need to use CONVERT_FLAG_FORCE as the SERVO7_* parameters will already be set from RC7_*
            AP_Param::convert_old_parameters(tri_conversion_info, ARRAY_SIZE(tri_conversion_info), AP_Param::CONVERT_FLAG_FORCE);
            const AP_Param::ConversionInfo tri_conversion_info_rev { Parameters::k_param_motors, 31, AP_PARAM_INT8,  "SERVO7_REVERSED" };
            AP_Param::convert_old_parameter(&tri_conversion_info_rev, 1, AP_Param::CONVERT_FLAG_REVERSE | AP_Param::CONVERT_FLAG_FORCE);
            // AP_MotorsTri was converted from having nested var_info to one level
            AP_Param::convert_parent_class(Parameters::k_param_motors, motors, motors->var_info);
        }
#endif
    }

    // upgrade parameters. This must be done after allocating the objects
    convert_pid_parameters();
}
