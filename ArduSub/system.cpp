#include "Sub.h"
#include "version.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void mavlink_delay_cb_static()
{
    sub.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    sub.mainloop_failsafe_check();
}

void Sub::init_ardupilot()
{
    // initialise serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit " FIRMWARE_STRING
                      "\n\nFree RAM: %u\n",
                      (unsigned)hal.util->available_memory());

    // load parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // initialise serial port
    serial_manager.init();

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

    // initialise notify system
    notify.init(true);

    // initialise battery monitor
    battery.init();

    barometer.init();

    celsius.init();

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();

    // setup telem slots with serial ports
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        gcs_chan[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    gcs().set_dataflash(&DataFlash);

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs
    init_joystick();            // joystick initialization

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

    if (g.compass_enabled) {
        init_compass();
    }

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

#if AVOIDANCE_ENABLED == ENABLED
    wp_nav.set_avoidance(&avoid);
#endif

    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
#if OPTFLOW == ENABLED
    init_optflow();
#endif

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(&DataFlash, serial_manager);
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // read Baro pressure at ground
    //-----------------------------
    init_barometer(false);
    barometer.update();

    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER && barometer.healthy(i)) {
            barometer.set_primary_baro(i);
            ap.depth_sensor_present = true;
            break;
        }
    }

    if (!ap.depth_sensor_present) {
        // We only have onboard baro
        // No external underwater depth sensor detected
        barometer.set_primary_baro(0);
        EKF2.set_baro_alt_noise(10.0f); // Readings won't correspond with rest of INS
        EKF3.set_baro_alt_noise(10.0f);
    } else {
        EKF2.set_baro_alt_noise(0.1f);
        EKF3.set_baro_alt_noise(0.1f);
    }

    leak_detector.init();

    // backwards compatibility
    if (attitude_control.get_accel_yaw_max() < 110000.0f) {
        attitude_control.save_accel_yaw_max(110000.0f);
    }

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

    startup_INS_ground();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    mainloop_failsafe_enable();

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

    // init vehicle capabilties
    init_capabilities();

    hal.console->print("\nInit complete");

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

// calibrate gyros - returns true if succesfully calibrated
bool Sub::calibrate_gyros()
{
    // gyro offset calibration
    sub.ins.init_gyro();

    // reset ahrs gyro bias
    if (sub.ins.gyro_calibrated_ok_all()) {
        sub.ahrs.reset_gyro_drift();
        return true;
    }

    return false;
}

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
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
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

void Sub::check_usb_mux(void)
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
bool Sub::should_log(uint32_t mask)
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
