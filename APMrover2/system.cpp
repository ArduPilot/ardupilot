/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
    We will determine later if we are actually on the ground and process a
    ground start in that case.

*****************************************************************************/

#include "Rover.h"
#include <AP_Common/AP_FWVersion.h>

static void mavlink_delay_cb_static()
{
    rover.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    rover.failsafe_check();
}

void Rover::init_ardupilot()
{
    // initialise console serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM.
    //

    load_parameters();
#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs().setup_console();

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // init gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

    g2.fence.init();

    // initialise notify system
    notify.init();
    notify_mode(control_mode);

    battery.init();

    // Initialise RPM sensor
    rpm_sensor.init();

    rssi.init();

    g2.airspeed.init();

    g2.windvane.init(serial_manager);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if OSD_ENABLED == ENABLED
    osd.init();
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise compass
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // initialise rangefinder
    rangefinder.init(ROTATION_NONE);

    // init proximity sensor
    init_proximity();

    // init beacons used for non-gps position estimation
    init_beacon();

    // init visual odometry
    init_visual_odom();

    // and baro for EKF
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    set_control_channels();  // setup radio channels and outputs ranges
    init_rc_in();            // sets up rc channels deadzone
    g2.motors.init();        // init motors including setting servo out channels ranges
    SRV_Channels::enable_aux_servos();

    // init wheel encoders
    g2.wheel_encoder.init();

    relay.init();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // initialize SmartRTL
    g2.smart_rtl.init();

    // initialise object avoidance
    g2.oa.init();

    startup_ground();

    Mode *initial_mode = mode_from_mode_num((enum Mode::Number)g.initial_mode.get());
    if (initial_mode == nullptr) {
        initial_mode = &mode_initializing;
    }
    set_mode(*initial_mode, ModeReason::INITIALISED);

    // initialise rc channels
    rc().init();

    rover.g2.sailboat.init();

    // disable safety if requested
    BoardConfig.init_safety();

    // flag that initialisation has completed
    initialised = true;

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif
}

//*********************************************************************************
// This function does all the calibrations, etc. that we need during a ground start
//*********************************************************************************
void Rover::startup_ground(void)
{
    set_mode(mode_initializing, ModeReason::INITIALISED);

    gcs().send_text(MAV_SEVERITY_INFO, "<startup_ground> Ground start");

    #if(GROUND_START_DELAY > 0)
        gcs().send_text(MAV_SEVERITY_NOTICE, "<startup_ground> With delay");
        delay(GROUND_START_DELAY * 1000);
    #endif

    // IMU ground start
    //------------------------
    //

    startup_INS_ground();

    // initialise mission library
    mode_auto.mission.init();

    // initialise AP_Logger library
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&rover, &Rover::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

#ifdef ENABLE_SCRIPTING
    g2.scripting.init();
#endif // ENABLE_SCRIPTING

    // we don't want writes to the serial port to cause us to pause
    // so set serial ports non-blocking once we are ready to drive
    serial_manager.set_blocking_writes_all(false);

    gcs().send_text(MAV_SEVERITY_INFO, "Ready to drive");
}

// update the ahrs flyforward setting which can allow
// the vehicle's movements to be used to estimate heading
void Rover::update_ahrs_flyforward()
{
    bool flyforward = false;

    // boats never use movement to estimate heading
    if (!is_boat()) {
        // throttle threshold is 15% or 1/2 cruise throttle
        bool throttle_over_thresh = g2.motors.get_throttle() > MIN(g.throttle_cruise * 0.50f, 15.0f);
        // desired speed threshold of 1m/s
        bool desired_speed_over_thresh = g2.attitude_control.speed_control_active() && (g2.attitude_control.get_desired_speed() > 0.5f);
        if (throttle_over_thresh || (is_positive(g2.motors.get_throttle()) && desired_speed_over_thresh)) {
            uint32_t now = AP_HAL::millis();
            // if throttle over threshold start timer
            if (flyforward_start_ms == 0) {
                flyforward_start_ms = now;
            }
            // if throttle over threshold for 2 seconds set flyforward to true
            flyforward = (now - flyforward_start_ms > 2000);
        } else {
            // reset timer
            flyforward_start_ms = 0;
        }
    }

    ahrs.set_fly_forward(flyforward);
}

bool Rover::set_mode(Mode &new_mode, ModeReason reason)
{
    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        return true;
    }

    Mode &old_mode = *control_mode;
    if (!new_mode.enter()) {
        // Log error that we failed to enter desired flight mode
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE,
                                 LogErrorCode(new_mode.mode_number()));
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed");
        return false;
    }

    control_mode = &new_mode;

    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    g2.fence.manual_recovery_start();

#if CAMERA == ENABLED
    camera.set_is_auto_mode(control_mode->mode_number() == Mode::Number::AUTO);
#endif

    old_mode.exit();

    control_mode_reason = reason;
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    gcs().send_message(MSG_HEARTBEAT);

    notify_mode(control_mode);
    return true;
}

bool Rover::set_mode(const uint8_t new_mode, ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    Mode *mode = rover.mode_from_mode_num((enum Mode::Number)new_mode);
    if (mode == nullptr) {
        return false;
    }
    return rover.set_mode(*mode, reason);
}

void Rover::startup_INS_ground(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Beginning INS calibration. Do not move vehicle");
    hal.scheduler->delay(100);

    ahrs.init();
    // say to EKF that rover only move by going forward
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();
}

// update notify with mode change
void Rover::notify_mode(const Mode *mode)
{
    AP_Notify::flags.autopilot_mode = mode->is_autopilot_mode();
    notify.flags.flight_mode = mode->mode_number();
    notify.set_flight_mode_str(mode->name4());
}

/*
  check a digital pin for high,low (1/0)
 */
uint8_t Rover::check_digital_pin(uint8_t pin)
{
    // ensure we are in input mode
    hal.gpio->pinMode(pin, HAL_GPIO_INPUT);

    // enable pullup
    hal.gpio->write(pin, 1);

    return hal.gpio->read(pin);
}

/*
  should we log a message type now?
 */
bool Rover::should_log(uint32_t mask)
{
    return logger.should_log(mask);
}

// returns true if vehicle is a boat
// this affects whether the vehicle tries to maintain position after reaching waypoints
bool Rover::is_boat() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BOAT);
}
