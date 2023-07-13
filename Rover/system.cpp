/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
    We will determine later if we are actually on the ground and process a
    ground start in that case.

*****************************************************************************/

#include "Rover.h"

static void failsafe_check_static()
{
    rover.failsafe_check();
}

void Rover::init_ardupilot()
{
#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // init gripper
#if AP_GRIPPER_ENABLED
    g2.gripper.init();
#endif

    // initialise notify system
    notify.init();
    notify_mode(control_mode);

    battery.init();

#if AP_RPM_ENABLED
    // Initialise RPM sensor
    rpm_sensor.init();
#endif

    rssi.init();

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

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

    // initialise rangefinder
    rangefinder.set_log_rfnd_bit(MASK_LOG_RANGEFINDER);
    rangefinder.init(ROTATION_NONE);

#if HAL_PROXIMITY_ENABLED
    // init proximity sensor
    g2.proximity.init();
#endif

#if AP_BEACON_ENABLED
    // init beacons used for non-gps position estimation
    g2.beacon.init();
#endif

    // and baro for EKF
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    init_rc_in();            // sets up rc channels deadzone
    g2.motors.init(get_frame_type());        // init motors including setting servo out channels ranges
    SRV_Channels::enable_aux_servos();

    // init wheel encoders
    g2.wheel_encoder.init();

#if HAL_TORQEEDO_ENABLED
    // init torqeedo motor driver
    g2.torqeedo.init();
#endif

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // AP_OPTICALFLOW_ENABLED

#if AP_RELAY_ENABLED
    relay.init();
#endif

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#if AC_PRECLAND_ENABLED
    // initialise precision landing
    init_precland();
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
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().convert_options(RC_Channel::AUX_FUNC::SAVE_TRIM, RC_Channel::AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC);
    rc().init();

    rover.g2.sailboat.init();

    // boat should loiter after completing a mission to avoid drifting off
    if (is_boat()) {
        rover.g2.mis_done_behave.set_default(ModeAuto::Mis_Done_Behave::MIS_DONE_BEHAVE_LOITER);
    }

    // flag that initialisation has completed
    initialised = true;
}

//*********************************************************************************
// This function does all the calibrations, etc. that we need during a ground start
//*********************************************************************************
void Rover::startup_ground(void)
{
    set_mode(mode_initializing, ModeReason::INITIALISED);

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

#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED
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

// Check if this mode can be entered from the GCS
bool Rover::gcs_mode_enabled(const Mode::Number mode_num) const
{
    // List of modes that can be blocked, index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::MANUAL,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::STEERING,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::FOLLOW,
        (uint8_t)Mode::Number::SIMPLE,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::RTL,
        (uint8_t)Mode::Number::SMART_RTL,
        (uint8_t)Mode::Number::GUIDED,
#if MODE_DOCK_ENABLED == ENABLED
        (uint8_t)Mode::Number::DOCK
#endif
    };

    return !block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list));
}

bool Rover::set_mode(Mode &new_mode, ModeReason reason)
{
    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        return true;
    }

    // Check if GCS mode change is disabled via parameter
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled((Mode::Number)new_mode.mode_number())) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"Mode change to %s denied, GCS entry disabled (FLTMODE_GCSBLOCK)", new_mode.name4());
        return false;
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

#if AP_FENCE_ENABLED
    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    fence.manual_recovery_start();
#endif

#if AP_CAMERA_ENABLED
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
        notify_no_such_mode(new_mode);
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
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::GROUND);

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

#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>
#if HAL_ADSB_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif
