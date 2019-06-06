#include "Copter.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

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
    // initialise serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // load parameters from EEPROM
    load_parameters();

    // time per loop - this gets updated in the main loop() based on
    // actual loop rate
    G_Dt = 1.0 / scheduler.get_loop_rate_hz();

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    
    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);


    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);
    
    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

    // init winch and wheel encoder
    winch_init();

    // initialise notify system
    notify.init();
    notify_flight_mode();

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts(serial_manager);

#if OSD_ENABLED == ENABLED
    osd.init();
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
#if FRAME_CONFIG == HELI_FRAME
    input_manager.set_loop_rate(scheduler.get_loop_rate_hz());
#endif

    init_rc_in();               // sets up rc channels from radio

    // default frame class to match firmware if possible
    set_default_frame_class();

    // allocate the motors class
    allocate_motors();

    // sets up motors and output to escs
    init_rc_out();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

#if BEACON_ENABLED == ENABLED
    // give AHRS the range beacon sensor
    ahrs.set_beacon(&g2.beacon);
#endif

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow);
#endif

    // init Location class
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    Location::set_terrain(&terrain);
    wp_nav->set_terrain(&terrain);
#endif

    attitude_control->parameter_sanity_check();
    pos_control->set_dt(scheduler.get_loop_period_s());

    // init the optical flow sensor
    init_optflow();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(serial_manager);
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precision landing
    init_precland();
#endif

    // initialise landing gear position
    landinggear.init();

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs().send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // initialise rangefinder
    init_rangefinder();

    // init proximity sensor
    init_proximity();

#if BEACON_ENABLED == ENABLED
    // init beacons used for non-gps position estimation
    g2.beacon.init();
#endif

    // init visual odometry
    init_visual_odom();

#if RPM_ENABLED == ENABLED
    // initialise AP_RPM library
    rpm_sensor.init();
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // initialise mission library
    mode_auto.mission.init();
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
    // initialize SmartRTL
    g2.smart_rtl.init();
#endif

    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&copter, &Copter::Log_Write_Vehicle_Startup_Messages, void));

    // initialise rc channels including setting mode
    rc().init();

    startup_INS_ground();

#ifdef ENABLE_SCRIPTING
    if (!g2.scripting.init()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Scripting failed to start");
    }
#endif // ENABLE_SCRIPTING

    // set landed flags
    set_land_complete(true);
    set_land_complete_maybe(true);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // disable safety if requested
    BoardConfig.init_safety();

    hal.console->printf("\nReady to FLY ");

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
#if OPTFLOW != ENABLED && VISUAL_ODOMETRY_ENABLED != ENABLED
    return false;
#else
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
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
        if(flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
        // if helicopters are on the ground, and the motor is switched off, auto-armed should be false
        // so that rotor runup is checked again before attempting to take-off
        if(ap.land_complete && motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED && ap.using_interlock) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors->armed() && ap.using_interlock) {
            if(!ap.throttle_zero && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                set_auto_armed(true);
            }
        // if motors are armed and throttle is above zero auto_armed should be true
        // if motors are armed and we are in throw mode, then auto_armed should be true
        } else if (motors->armed() && !ap.using_interlock) {
            if(!ap.throttle_zero || control_mode == THROW) {
                set_auto_armed(true);
            }
        }
    }
}

/*
  should we log a message type now?
 */
bool Copter::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

// default frame_class to match firmware if possible
void Copter::set_default_frame_class()
{
    if (FRAME_CONFIG == HELI_FRAME &&
        g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_DUAL &&
        g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_QUAD) {
        g2.frame_class.set(AP_Motors::MOTOR_FRAME_HELI);
    }
}

// return MAV_TYPE corresponding to frame class
MAV_TYPE Copter::get_frame_mav_type()
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
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            return MAV_TYPE_HELICOPTER;
        case AP_Motors::MOTOR_FRAME_TRI:
            return MAV_TYPE_TRICOPTER;
        case AP_Motors::MOTOR_FRAME_SINGLE:
        case AP_Motors::MOTOR_FRAME_COAX:
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            return MAV_TYPE_COAXIAL;
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
            return MAV_TYPE_DODECAROTOR;
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
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            return "HELI_QUAD";
        case AP_Motors::MOTOR_FRAME_TRI:
            return "TRI";
        case AP_Motors::MOTOR_FRAME_SINGLE:
            return "SINGLE";
        case AP_Motors::MOTOR_FRAME_COAX:
            return "COAX";
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            return "TAILSITTER";
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
            return "DODECA_HEXA";
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
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
        default:
            motors = new AP_MotorsMatrix(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = new AP_MotorsTri(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTri::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors = new AP_MotorsSingle(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_COAX:
            motors = new AP_MotorsCoax(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors = new AP_MotorsTailsitter(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
#else // FRAME_CONFIG == HELI_FRAME
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors = new AP_MotorsHeli_Dual(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;

        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            motors = new AP_MotorsHeli_Quad(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Quad::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
            
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors = new AP_MotorsHeli_Single(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
#endif
    }
    if (motors == nullptr) {
        AP_HAL::panic("Unable to allocate FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_HAL::panic("Unable to allocate AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo *ac_var_info;

#if FRAME_CONFIG != HELI_FRAME
    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    ac_var_info = AC_AttitudeControl_Multi::var_info;
#else
    attitude_control = new AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    ac_var_info = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_HAL::panic("Unable to allocate AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);
        
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    if (pos_control == nullptr) {
        AP_HAL::panic("Unable to allocate PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (wp_nav == nullptr) {
        AP_HAL::panic("Unable to allocate WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_HAL::panic("Unable to allocate LoiterNav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

#if MODE_CIRCLE_ENABLED == ENABLED
    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_HAL::panic("Unable to allocate CircleNav");
    }
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);
#endif

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);
    
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

    // brushed 16kHz defaults to 16kHz pulses
    if (motors->get_pwm_type() == AP_Motors::PWM_TYPE_BRUSHED) {
        g.rc_speed.set_default(16000);
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
#if FRAME_CONFIG == HELI_FRAME
    convert_tradheli_parameters();
#endif
}

bool Copter::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
