#include "Plane.h"
#include <AP_Common/AP_FWVersion.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void mavlink_delay_cb_static()
{
    plane.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    plane.failsafe_check();
}

void Plane::init_ardupilot()
{
    // initialise serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM
    //
    load_parameters();

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // set sensors to HIL mode
        ins.set_hil_mode();
        compass.set_hil_mode();
        barometer.set_hil_mode();
    }
#endif

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    set_control_channels();

    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);


    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // setup any board specific drivers
    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // initialise rc channels including setting mode
    rc().init();

    relay.init();

    // initialise notify system
    notify.init();
    notify_mode(*control_mode);

    init_rc_out_main();
    
    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro
    barometer.init();

    // initialise rangefinder
    rangefinder.set_log_rfnd_bit(MASK_LOG_SONAR);
    rangefinder.init(ROTATION_PITCH_270);

    // initialise battery monitoring
    battery.init();

    rpm_sensor.init();

    // setup telem slots with serial ports
    gcs().setup_uarts(serial_manager);

#if OSD_ENABLED == ENABLED
    osd.init();
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise airspeed sensor
    airspeed.init();

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if OPTFLOW == ENABLED
    // make optflow available to libraries
    if (optflow.enabled()) {
        ahrs.set_optflow(&optflow);
    }
#endif

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

    // GPS Initialization
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    init_rc_in();               // sets up rc channels from radio

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(serial_manager);
#endif

#if LANDING_GEAR_ENABLED == ENABLED
    // initialise landing gear position
    g2.landing_gear.init();
    gear.last_auto_cmd = -1;
    gear.last_cmd = -1;
#endif

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    quadplane.setup();

    AP_Param::reload_defaults_file(true);
    
    startup_ground();

    // don't initialise aux rc output until after quadplane is setup as
    // that can change initial values of channels
    init_rc_out_aux();
    
    // choose the nav controller
    set_nav_controller();

    set_mode_by_number((enum Mode::Number)g.initial_mode.get(), MODE_REASON_UNKNOWN);

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();

    // initialise sensor
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        optflow.init(-1);
    }
#endif

// init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

    // disable safety if requested
    BoardConfig.init_safety();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void Plane::startup_ground(void)
{
    set_mode(mode_initializing, MODE_REASON_UNKNOWN);

#if (GROUND_START_DELAY > 0)
    gcs().send_text(MAV_SEVERITY_NOTICE,"Ground start with delay");
    delay(GROUND_START_DELAY * 1000);
#else
    gcs().send_text(MAV_SEVERITY_INFO,"Ground start");
#endif

    //INS ground start
    //------------------------
    //
    startup_INS_ground();

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();

    // initialise AP_Logger library
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

#ifdef ENABLE_SCRIPTING
    if (!g2.scripting.init()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Scripting failed to start");
    }
#endif // ENABLE_SCRIPTING

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    failsafe.last_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    gcs().send_text(MAV_SEVERITY_INFO,"Ground start complete");
}


bool Plane::set_mode(Mode &new_mode, const mode_reason_t reason)
{
    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        return true;
    }

#if !QAUTOTUNE_ENABLED
    if (&new_mode == &plane.mode_qautotune) {
        gcs().send_text(MAV_SEVERITY_INFO,"QAUTOTUNE disabled");
        set_mode(plane.mode_qhover, MODE_REASON_UNAVAILABLE);
        return false;
    }
#endif

    // backup current control_mode and previous_mode
    Mode &old_previous_mode = *previous_mode;
    Mode &old_mode = *control_mode;
    const mode_reason_t previous_mode_reason_backup = previous_mode_reason;

    // update control_mode assuming success
    // TODO: move these to be after enter() once start_command_callback() no longer checks control_mode
    previous_mode = control_mode;
    control_mode = &new_mode;
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

    // attempt to enter new mode
    if (!new_mode.enter()) {
        // Log error that we failed to enter desired flight mode
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed");

        // we failed entering new mode, roll back to old
        previous_mode = &old_previous_mode;
        control_mode = &old_mode;

        control_mode_reason = previous_mode_reason;
        previous_mode_reason = previous_mode_reason_backup;

        // currently, only Q modes can fail enter(). This will likely change in the future and all modes
        // should be changed to check dependencies and fail early before depending on changes in Mode::set_mode()
        if (control_mode->is_vtol_mode()) {
            // ignore result because if we fail we risk looping at the qautotune check above
            control_mode->enter();
        }
        return false;
    }

    if (previous_mode == &mode_autotune) {
        // restore last gains
        autotune_restore();
    }

    // exit previous mode
    old_mode.exit();

    // record reasons
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

    // log and notify mode change
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    notify_mode(*control_mode);

    return true;
}

bool Plane::set_mode_by_number(const Mode::Number new_mode_number, const mode_reason_t reason)
{
    Mode *new_mode = plane.mode_from_mode_num(new_mode_number);
    if (new_mode == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Error: invalid mode number: %d", new_mode_number);
        return false;
    }
    return set_mode(*new_mode, reason);
}

void Plane::check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if (failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS && flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
        uint32_t radio_timeout_ms = failsafe.last_valid_rc_ms;
        if (failsafe.state == FAILSAFE_SHORT) {
            // time is relative to when short failsafe enabled
            radio_timeout_ms = failsafe.short_timer_ms;
        }
        if (failsafe.rc_failsafe &&
            (tnow - radio_timeout_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_LONG, MODE_REASON_RADIO_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == &mode_auto &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, MODE_REASON_GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, MODE_REASON_GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs().chan(0).last_radio_status_remrssi_ms != 0 &&
                   (tnow - gcs().chan(0).last_radio_status_remrssi_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, MODE_REASON_GCS_FAILSAFE);
        }
    } else {
        uint32_t timeout_seconds = g.fs_timeout_long;
        if (g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
            // avoid dropping back into short timeout
            timeout_seconds = g.fs_timeout_short;
        }
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - failsafe.last_heartbeat_ms) < timeout_seconds*1000) {
            failsafe_long_off_event(MODE_REASON_GCS_FAILSAFE);
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.rc_failsafe) {
            failsafe_long_off_event(MODE_REASON_RADIO_FAILSAFE);
        }
    }
}

void Plane::check_short_failsafe()
{
    // only act on changes
    // -------------------
    if (g.fs_action_short != FS_ACTION_SHORT_DISABLED &&
       failsafe.state == FAILSAFE_NONE &&
       flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // The condition is checked and the flag rc_failsafe is set in radio.cpp
        if(failsafe.rc_failsafe) {
            failsafe_short_on_event(FAILSAFE_SHORT, MODE_REASON_RADIO_FAILSAFE);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.rc_failsafe || g.fs_action_short == FS_ACTION_SHORT_DISABLED) {
            failsafe_short_off_event(MODE_REASON_RADIO_FAILSAFE);
        }
    }
}


void Plane::startup_INS_ground(void)
{
#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        while (barometer.get_last_update() == 0) {
            // the barometer begins updating when we get the first
            // HIL_STATE message
            gcs().send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
            hal.scheduler->delay(1000);
        }
    }
#endif

    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
    } else {
        gcs().send_text(MAV_SEVERITY_ALERT, "Skipping INS calibration");
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
    ahrs.set_wind_estimation(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        airspeed.calibrate(true);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING,"No airspeed");
    }
}

// sets notify object flight mode information
void Plane::notify_mode(const Mode& mode)
{
    notify.flags.flight_mode = mode.mode_number();
    notify.set_flight_mode_str(mode.name4());
}

/*
  should we log a message type now?
 */
bool Plane::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    return logger.should_log(mask);
#else
    return false;
#endif
}

/*
  return throttle percentage from 0 to 100 for normal use and -100 to 100 when using reverse thrust
 */
int8_t Plane::throttle_percentage(void)
{
    if (quadplane.in_vtol_mode()) {
        return quadplane.throttle_percentage();
    }
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (!have_reverse_thrust()) {
        return constrain_int16(throttle, 0, 100);
    }
    return constrain_int16(throttle, -100, 100);
}

/*
  update AHRS soft arm state and log as needed
 */
void Plane::change_arm_state(void)
{
    Log_Arm_Disarm();
    update_soft_armed();
    quadplane.set_armed(hal.util->get_soft_armed());
}

/*
  arm motors
 */
bool Plane::arm_motors(const AP_Arming::Method method, const bool do_arming_checks)
{
    if (!arming.arm(method, do_arming_checks)) {
        return false;
    }

    change_arm_state();
    return true;
}

/*
  disarm motors
 */
bool Plane::disarm_motors(void)
{
    if (!arming.disarm()) {
        return false;
    }
    if (control_mode != &mode_auto) {
        // reset the mission on disarm if we are not in auto
        mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    throttle_suppressed = auto_throttle_mode;
    
    //only log if disarming was successful
    change_arm_state();

    // reload target airspeed which could have been modified by a mission
    plane.aparm.airspeed_cruise_cm.load();
    
#if QAUTOTUNE_ENABLED
    //save qautotune gains if enabled and success
    if (control_mode == &mode_qautotune) {
        quadplane.qautotune.save_tuning_gains();
    } else {
        quadplane.qautotune.reset();
    }
#endif

    return true;
}
