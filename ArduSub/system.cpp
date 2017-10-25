#include "Sub.h"

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

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        fwver.fw_string,
                        (unsigned)hal.util->available_memory());

    // load parameters from EEPROM
    load_parameters();

    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // Detection won't work until after BoardConfig.init()
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        AP_Param::set_by_name("GND_EXT_BUS", 0);
        break;
    default:
        AP_Param::set_by_name("GND_EXT_BUS", 1);
        break;
    }
#else
    AP_Param::set_default_by_name("GND_EXT_BUS", 1);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    
    // initialise serial port
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

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

    // setup telem slots with serial ports
    gcs().setup_uarts(serial_manager);

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
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

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
    camera_mount.init(serial_manager);
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // Init baro and determine if we have external (depth) pressure sensor
    init_barometer(false);
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
        EKF2.set_baro_alt_noise(10.0f); // Readings won't correspond with rest of INS
        EKF3.set_baro_alt_noise(10.0f);
    } else {
        EKF2.set_baro_alt_noise(0.1f);
        EKF3.set_baro_alt_noise(0.1f);
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

    // initialise DataFlash library
    DataFlash.set_mission(&mission);
    DataFlash.setVehicle_Startup_Log_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));

    startup_INS_ground();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    mainloop_failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // init vehicle capabilties
    init_capabilities();

    // disable safety if requested
    BoardConfig.init_safety();    
    
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

// calibrate gyros - returns true if successfully calibrated
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

/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = DataFlash.logging_started();
    return DataFlash.should_log(mask);
#else
    return false;
#endif
}
