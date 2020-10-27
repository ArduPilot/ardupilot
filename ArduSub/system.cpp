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

#if AP_FEATURE_BOARD_DETECT
    // Detection won't work until after BoardConfig.init()
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        AP_Param::set_by_name("GND_EXT_BUS", 0);
        celsius.init(0);
        break;
    default:
        AP_Param::set_by_name("GND_EXT_BUS", 1);
        celsius.init(1);
        break;
    }
#else
    AP_Param::set_default_by_name("GND_EXT_BUS", 1);
    celsius.init(1);
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

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise rc channels including setting mode
    rc().init();

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs
    init_joystick();            // joystick initialization

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

#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow);
#endif

    // init Location class
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    Location::set_terrain(&terrain);
    wp_nav.set_terrain(&terrain);
#endif

    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
#if OPTFLOW == ENABLED
    init_optflow();
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

    // disable safety if requested
    BoardConfig.init_safety();

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
    if (!motors.armed()) {
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
    if (!motors.armed()) {
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
