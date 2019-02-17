#include "Plane.h"

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
    notify_flight_mode(control_mode);

    init_rc_out_main();
    
    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro
    barometer.init();

    // initialise rangefinder
    rangefinder.init();

    // initialise battery monitoring
    battery.init();

    rpm_sensor.init();

    // setup telem slots with serial ports
    gcs().setup_uarts(serial_manager);

    // setup frsky
#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky, and pass a number of parameters to the library
    frsky_telemetry.init(MAV_TYPE_FIXED_WING);
#endif
#if DEVO_TELEM_ENABLED == ENABLED
    devo_telemetry.init();
#endif

#if OSD_ENABLED == ENABLED
    osd.init();
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise airspeed sensor
    airspeed.init();

    if (g.compass_enabled==true) {
        bool compass_ok = compass.init() && compass.read();
#if HIL_SUPPORT
    if (g.hil_mode != 0) {
        compass_ok = true;
    }
#endif
        if (!compass_ok) {
            hal.console->printf("Compass initialisation failed!\n");
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }
    
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

    set_mode((FlightMode)g.initial_mode.get(), MODE_REASON_UNKNOWN);

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
    set_mode(INITIALISING, MODE_REASON_UNKNOWN);

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

enum FlightMode Plane::get_previous_mode() {
    return previous_mode; 
}

void Plane::set_mode(enum FlightMode mode, mode_reason_t reason)
{
#if !QAUTOTUNE_ENABLED
    if (mode == QAUTOTUNE) {
        gcs().send_text(MAV_SEVERITY_INFO,"QAUTOTUNE disabled");
        mode = QHOVER;
    }
#endif

    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }

    if(g.auto_trim > 0 && control_mode == MANUAL) {
        trim_radio();
    }

    // perform any cleanup required for prev flight mode
    exit_mode(control_mode);

    // cancel inverted flight
    auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    auto_state.next_wp_crosstrack = false;

    // reset landing check
    auto_state.checked_for_autoland = false;
    
    // zero locked course
    steer_state.locked_course_err = 0;

    // reset crash detection
    crash_state.is_crashed = false;
    crash_state.impact_detected = false;

    // reset external attitude guidance
    guided_state.last_forced_rpy_ms.zero();
    guided_state.last_forced_throttle_ms = 0;

    // set mode
    previous_mode = control_mode;
    control_mode = mode;
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.update_control_mode(control_mode);
#endif
#if DEVO_TELEM_ENABLED == ENABLED
    devo_telemetry.update_control_mode(control_mode);
#endif

#if CAMERA == ENABLED
    camera.set_is_auto_mode(control_mode == AUTO);
#endif

    if (previous_mode == AUTOTUNE && control_mode != AUTOTUNE) {
        // restore last gains
        autotune_restore();
    }

    // zero initial pitch and highest airspeed on mode change
    auto_state.highest_airspeed = 0;
    auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    prev_WP_loc = current_loc;

    // new mode means new loiter
    loiter.start_time_ms = 0;

    // record time of mode change
    last_mode_change_ms = AP_HAL::millis();
    
    // assume non-VTOL mode
    auto_state.vtol_mode = false;
    auto_state.vtol_loiter = false;
    
    switch(control_mode)
    {
    case INITIALISING:
        throttle_allows_nudging = true;
        auto_throttle_mode = true;
        auto_navigation_mode = false;
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
        throttle_allows_nudging = false;
        auto_throttle_mode = false;
        auto_navigation_mode = false;
        break;

    case AUTOTUNE:
        throttle_allows_nudging = false;
        auto_throttle_mode = false;
        auto_navigation_mode = false;
        autotune_start();
        break;

    case ACRO:
        throttle_allows_nudging = false;
        auto_throttle_mode = false;
        auto_navigation_mode = false;
        acro_state.locked_roll = false;
        acro_state.locked_pitch = false;
        break;
        
    case CRUISE:
        throttle_allows_nudging = false;
        auto_throttle_mode = true;
        auto_navigation_mode = false;
        cruise_state.locked_heading = false;
        cruise_state.lock_timer_ms = 0;

#if SOARING_ENABLED == ENABLED
        // for ArduSoar soaring_controller
        g2.soaring_controller.init_cruising();
#endif
        
        set_target_altitude_current();
        break;

    case FLY_BY_WIRE_B:
        throttle_allows_nudging = false;
        auto_throttle_mode = true;
        auto_navigation_mode = false;

#if SOARING_ENABLED == ENABLED
        // for ArduSoar soaring_controller
        g2.soaring_controller.init_cruising();
#endif

        set_target_altitude_current();
        break;

    case CIRCLE:
        // the altitude to circle at is taken from the current altitude
        throttle_allows_nudging = false;
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        next_WP_loc.alt = current_loc.alt;
        break;

    case AUTO:
        throttle_allows_nudging = true;
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        if (quadplane.available() && quadplane.enable == 2) {
            auto_state.vtol_mode = true;
        } else {
            auto_state.vtol_mode = false;
        }
        next_WP_loc = prev_WP_loc = current_loc;
        // start or resume the mission, based on MIS_AUTORESET
        mission.start_or_resume();

#if SOARING_ENABLED == ENABLED
        g2.soaring_controller.init_cruising();
#endif
        break;

    case RTL:
        throttle_allows_nudging = true;
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        prev_WP_loc = current_loc;
        do_RTL(get_RTL_altitude());
        break;

    case LOITER:
        throttle_allows_nudging = true;
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        do_loiter_at_location();

#if SOARING_ENABLED == ENABLED		
        if (g2.soaring_controller.is_active() &&
            g2.soaring_controller.suppress_throttle()) {
			g2.soaring_controller.init_thermalling();
			g2.soaring_controller.get_target(next_WP_loc); // ahead on flight path
		}
#endif
		
        break;

    case AVOID_ADSB:
    case GUIDED:
        throttle_allows_nudging = true;
        auto_throttle_mode = true;
        auto_navigation_mode = true;
        guided_throttle_passthru = false;
        /*
          when entering guided mode we set the target as the current
          location. This matches the behaviour of the copter code
        */
        guided_WP_loc = current_loc;
        set_guided_WP();
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
    case QAUTOTUNE:
    case QACRO:
        throttle_allows_nudging = true;
        auto_navigation_mode = false;
        if (!quadplane.init_mode()) {
            control_mode = previous_mode;
        } else {
            auto_throttle_mode = false;
            auto_state.vtol_mode = true;
        }
        break;
    }

    // start with throttle suppressed in auto_throttle modes
    throttle_suppressed = auto_throttle_mode;

    adsb.set_is_auto_mode(auto_navigation_mode);

    logger.Write_Mode(control_mode, control_mode_reason);

    // update notify with flight mode change
    notify_flight_mode(control_mode);

    // reset steering integrator on mode change
    steerController.reset_I();    

    // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
    control_failsafe();
}

// exit_mode - perform any cleanup required when leaving a flight mode
void Plane::exit_mode(enum FlightMode mode)
{
    // stop mission when we leave auto
    switch (mode) {
    case AUTO:
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();

            if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND &&
                !quadplane.is_vtol_land(mission.get_current_nav_cmd().id))
            {
                landing.restart_landing_sequence();
            }
        }
        auto_state.started_flying_in_auto_ms = 0;
        break;
#if QAUTOTUNE_ENABLED
    case QAUTOTUNE:
        quadplane.qautotune.stop();
        break;
#endif
    default:
        break;
    }
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
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == AUTO &&
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
        hal.scheduler->delay(100);
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

// updates the status of the notify objects
// should be called at 50hz
void Plane::update_notify()
{
    notify.update();
}

// sets notify object flight mode information
void Plane::notify_flight_mode(enum FlightMode mode)
{
    AP_Notify::flags.flight_mode = mode;

    // set flight mode string
    switch (mode) {
    case MANUAL:
        notify.set_flight_mode_str("MANU");
        break;
    case CIRCLE:
        notify.set_flight_mode_str("CIRC");
        break;
    case STABILIZE:
        notify.set_flight_mode_str("STAB");
        break;
    case TRAINING:
        notify.set_flight_mode_str("TRAN");
        break;
    case ACRO:
        notify.set_flight_mode_str("ACRO");
        break;
    case FLY_BY_WIRE_A:
        notify.set_flight_mode_str("FBWA");
        break;
    case FLY_BY_WIRE_B:
        notify.set_flight_mode_str("FBWB");
        break;
    case CRUISE:
        notify.set_flight_mode_str("CRUS");
        break;
    case AUTOTUNE:
        notify.set_flight_mode_str("ATUN");
        break;
    case AUTO:
        notify.set_flight_mode_str("AUTO");
        break;
    case RTL:
        notify.set_flight_mode_str("RTL ");
        break;
    case LOITER:
        notify.set_flight_mode_str("LOITER");
        break;
    case AVOID_ADSB:
        notify.set_flight_mode_str("AVOI");
        break;
    case GUIDED:
        notify.set_flight_mode_str("GUID");
        break;
    case INITIALISING:
        notify.set_flight_mode_str("INIT");
        break;
    case QSTABILIZE:
        notify.set_flight_mode_str("QSTB");
        break;
    case QHOVER:
        notify.set_flight_mode_str("QHOV");
        break;
    case QLOITER:
        notify.set_flight_mode_str("QLOT");
        break;
    case QLAND:
        notify.set_flight_mode_str("QLND");
        break;
    case QRTL:
        notify.set_flight_mode_str("QRTL");
        break;
    case QAUTOTUNE:
        notify.set_flight_mode_str("QAUTOTUNE");
        break;
    default:
        notify.set_flight_mode_str("----");
        break;
    }
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
bool Plane::arm_motors(const AP_Arming::ArmingMethod method, const bool do_arming_checks)
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
    if (control_mode != AUTO) {
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
    quadplane.qautotune.save_tuning_gains();
#endif

    return true;
}
