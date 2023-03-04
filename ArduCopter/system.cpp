#include "Copter.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    copter.failsafe_check();
}

void Copter::init_ardupilot()
{

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // init cargo gripper
#if AP_GRIPPER_ENABLED
    g2.gripper.init();
#endif

    // init winch
#if AP_WINCH_ENABLED
    g2.winch.init();
#endif

    // initialise notify system
    notify.init();
    notify_flight_mode();

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();

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

    // initialise surface to be tracked in SurfaceTracking
    // must be before rc init to not override initial switch position
    surface_tracking.init((SurfaceTracking::Surface)copter.g2.surftrak_mode.get());

    // allocate the motors class
    allocate_motors();

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE);
    rc().init();

    // sets up motors and output to escs
    init_rc_out();

    // check if we should enter esc calibration mode
    esc_calibration_startup_check();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    g2.oa.init();
#endif

    attitude_control->parameter_sanity_check();

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // AP_OPTICALFLOW_ENABLED

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precision landing
    init_precland();
#endif

#if AP_LANDINGGEAR_ENABLED
    // initialise landing gear position
    landinggear.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // initialise rangefinder
    init_rangefinder();

#if HAL_PROXIMITY_ENABLED
    // init proximity sensor
    g2.proximity.init();
#endif

#if BEACON_ENABLED == ENABLED
    // init beacons used for non-gps position estimation
    g2.beacon.init();
#endif

#if AP_RPM_ENABLED
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

    startup_INS_ground();

#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    custom_control.init();
#endif

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

    // attempt to set the intial_mode, else set to STABILIZE
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        // set mode to STABILIZE will trigger mode change notification to pilot
        set_mode(Mode::Number::STABILIZE, ModeReason::UNAVAILABLE);
    }

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
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Copter::ekf_has_absolute_position() const
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

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Copter::ekf_has_relative_position() const
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled and dead reckoning is inactive
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        enabled = true;
    }
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
}

// returns true if the ekf has a good altitude estimate (required for modes which do AltHold)
bool Copter::ekf_alt_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow alt control with only dcm
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // require both vertical velocity and position
    return (filt_status.flags.vert_vel && filt_status.flags.vert_pos);
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
            if(!ap.throttle_zero || flightmode->mode_number() == Mode::Number::THROW) {
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
        case AP_Motors::MOTOR_FRAME_DECA:
        case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX:
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
        case AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING:
#if AP_SCRIPTING_ENABLED
            motors = new AP_MotorsMatrix_6DoF_Scripting(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_6DoF_Scripting::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
        case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
#if AP_SCRIPTING_ENABLED
            motors = new AP_MotorsMatrix_Scripting_Dynamic(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
#endif // AP_SCRIPTING_ENABLED
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
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo *ac_var_info;

#if FRAME_CONFIG != HELI_FRAME
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() == AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING) {
#if AP_SCRIPTING_ENABLED
        attitude_control = new AC_AttitudeControl_Multi_6DoF(*ahrs_view, aparm, *motors);
        ac_var_info = AC_AttitudeControl_Multi_6DoF::var_info;
#endif // AP_SCRIPTING_ENABLED
    } else {
        attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors);
        ac_var_info = AC_AttitudeControl_Multi::var_info;
    }
#else
    attitude_control = new AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors);
    ac_var_info = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_BoardConfig::allocation_error("AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);
        
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    if (pos_control == nullptr) {
        AP_BoardConfig::allocation_error("PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    wp_nav = new AC_WPNav_OA(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#else
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#endif
    if (wp_nav == nullptr) {
        AP_BoardConfig::allocation_error("WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_BoardConfig::allocation_error("LoiterNav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

#if MODE_CIRCLE_ENABLED == ENABLED
    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_BoardConfig::allocation_error("CircleNav");
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
        attitude_control->get_rate_yaw_pid().filt_D_hz().set_default(100);
        break;
    default:
        break;
    }

    // brushed 16kHz defaults to 16kHz pulses
    if (motors->is_brushed_pwm_type()) {
        g.rc_speed.set_default(16000);
    }
    
    // upgrade parameters. This must be done after allocating the objects
    convert_pid_parameters();
#if FRAME_CONFIG == HELI_FRAME
    convert_tradheli_parameters();
#endif

#if HAL_PROXIMITY_ENABLED
    // convert PRX to PRX1_ parameters
    convert_prx_parameters();
#endif

    // param count could have changed
    AP_Param::invalidate_count();
}

bool Copter::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
