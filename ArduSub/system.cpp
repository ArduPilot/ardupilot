#include "Sub.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    sub.mainloop_failsafe_check();
}

void Sub::init_ardupilot()
{
    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

#if AC_FENCE == ENABLED
    fence.init();
#endif

    // initialise notify system
    notify.init();

    // initialise battery monitor
    battery.init();

    barometer.init();

#if AP_FEATURE_BOARD_DETECT
    // Detection won't work until after BoardConfig.init()
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 0);
        break;
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        AP_Param::set_by_name("BARO_EXT_BUS", 1);
        break;
    default:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
        break;
    }
#elif CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
#endif
    celsius.init(barometer.external_bus());

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif


    init_rc_in();               // sets up rc channels from radio

    allocate_motors();          // allocate the motors class

    // initialise rc channels including setting mode
    rc().init();

    init_rc_out();              // sets up motors and output to escs

    init_joystick();            // joystick initialization

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

    // init Location class
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    Location::set_terrain(&terrain);
    wp_nav->set_terrain(&terrain);
#endif

#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
    // This step ncessary so the servo is properly initialized
    camera_mount.set_angle_targets(0, 0, 0);
    // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
    camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // Init baro and determine if we have external (depth) pressure sensor
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate(false);
    barometer.update();

    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER) {
            barometer.set_primary_baro(i);
            depth_sensor_idx = i;
            ap.depth_sensor_present = true;
            sensor_health.depth = barometer.healthy(depth_sensor_idx); // initialize health flag
            break; // Go with the first one we find
        }
    }

    if (!ap.depth_sensor_present) {
        // We only have onboard baro
        // No external underwater depth sensor detected
        barometer.set_primary_baro(0);
        ahrs.set_alt_measurement_noise(10.0f);  // Readings won't correspond with rest of INS
    } else {
        ahrs.set_alt_measurement_noise(0.1f);
    }

    leak_detector.init();

    last_pilot_heading = ahrs.yaw_sensor;

    // initialise rangefinder
#if RANGEFINDER_ENABLED == ENABLED
    init_rangefinder();
#endif

    // initialise AP_RPM library
#if RPM_ENABLED == ENABLED
    rpm_sensor.init();
#endif

    // initialise mission library
    mission.init();

    // initialise AP_Logger library
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));
#endif

    startup_INS_ground();

#ifdef ENABLE_SCRIPTING
    g2.scripting.init();
#endif // ENABLE_SCRIPTING

    g2.airspeed.init();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    mainloop_failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Sub::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_SUBMARINE);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if successfully calibrated
// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Sub::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Sub::ekf_position_ok()
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
    }

    // once armed we require a good absolute position and EKF must not be in const_pos_mode
    return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
{
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
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
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
    }
    return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
}

/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

// dummy method to avoid linking AFS
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#if HAL_ADSB_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif


/*
  allocate the motors class
 */
void Sub::allocate_motors(void)
{
    switch ((AP_Motors6DOF::sub_frame_t)g.frame_configuration.get()) {
        case AP_Motors6DOF::SUB_FRAME_BLUEROV1:
        case AP_Motors6DOF::SUB_FRAME_VECTORED:
        case AP_Motors6DOF::SUB_FRAME_VECTORED_6DOF:
        case AP_Motors6DOF::SUB_FRAME_VECTORED_6DOF_90DEG:
        case AP_Motors6DOF::SUB_FRAME_SIMPLEROV_3:
        case AP_Motors6DOF::SUB_FRAME_SIMPLEROV_4:
        case AP_Motors6DOF::SUB_FRAME_SIMPLEROV_5:
        case AP_Motors6DOF::SUB_FRAME_CUSTOM:
        default:
            motors = new AP_Motors6DOF(sub.scheduler.get_loop_rate_hz());
            motors_var_info = AP_Motors6DOF::var_info;
            break;
        case AP_Motors6DOF::SUB_FRAME_SCRIPTING:
#ifdef ENABLE_SCRIPTING
            // motors = new AP_MotorsMatrix_6DoF_ScriptingSub(sub.scheduler.get_loop_rate_hz());
            // motors_var_info = AP_MotorsMatrix_6DoF_ScriptingSub::var_info;
#endif // ENABLE_SCRIPTING
            break;
    }

    if (motors == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate FRAME_CLASS=%u", (unsigned)g.frame_configuration.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo *ac_var_info;

    attitude_control = new AC_AttitudeControl_Sub(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    ac_var_info = AC_AttitudeControl_Sub::var_info;

    if (attitude_control == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);

    pos_control = new AC_PosControl_Sub(*ahrs_view, inertial_nav, *motors, *attitude_control, scheduler.get_loop_period_s());
    if (pos_control == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (wp_nav == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate LoiterNav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate CircleNav");
    }
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);

    // upgrade parameters. This must be done after allocating the objects
    convert_old_parameters();

    // param count could have changed
    AP_Param::invalidate_count();

}
