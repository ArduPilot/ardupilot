#include "Plane.h"

#include "qautotune.h"

static void failsafe_check_static()
{
    plane.failsafe_check();
}

void Plane::init_ardupilot()
{

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    rollController.convert_pid();
    pitchController.convert_pid();

    // initialise rc channels including setting mode
#if HAL_QUADPLANE_ENABLED
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, (quadplane.enabled() && quadplane.option_is_set(QuadPlane::OPTION::AIRMODE_UNUSED) && (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr)) ? RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE : RC_Channel::AUX_FUNC::ARMDISARM);
#else
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
#endif
    rc().init();

#if AP_RELAY_ENABLED
    relay.init();
#endif

    // initialise notify system
    notify.init();
    notify_mode(*control_mode);

    init_rc_out_main();

    // init baro
    barometer.init();

#if AP_RANGEFINDER_ENABLED
    // initialise rangefinder
    rangefinder.set_log_rfnd_bit(MASK_LOG_SONAR);
    rangefinder.init(ROTATION_PITCH_270);
#endif

    // initialise battery monitoring
    battery.init();

#if AP_RSSI_ENABLED
    rssi.init();
#endif

#if AP_RPM_ENABLED
    rpm_sensor.init();
#endif

    // setup telem slots with serial ports
    gcs().setup_uarts();


#if OSD_ENABLED
    osd.init();
#endif

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_fixedwing_parameters(&aparm);
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

    // GPS Initialization
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    init_rc_in();               // sets up rc channels from radio

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#if AP_LANDINGGEAR_ENABLED
    // initialise landing gear position
    g2.landing_gear.init();
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

#if HAL_QUADPLANE_ENABLED
    quadplane.setup();
#endif

    AP_Param::reload_defaults_file(true);

    set_mode(mode_initializing, ModeReason::INITIALISED);

#if (GROUND_START_DELAY > 0)
    gcs().send_text(MAV_SEVERITY_NOTICE,"Ground start with delay");
    delay(GROUND_START_DELAY * 1000);
#else
    gcs().send_text(MAV_SEVERITY_INFO,"Ground start");
#endif

    //INS ground start
    //------------------------
    //
    startup_INS();

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();

    // initialise AP_Logger library
#if HAL_LOGGING_ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    gcs().sysid_myggcs_seen(AP_HAL::millis());

    // don't initialise aux rc output until after quadplane is setup as
    // that can change initial values of channels
    init_rc_out_aux();

    if (g2.oneshot_mask != 0) {
        hal.rcout->set_output_mode(g2.oneshot_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
    }
    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());

    set_mode_by_number((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED);

    // set the correct flight mode
    // ---------------------------
    rc().reset_mode_switch();

    // initialise sensor
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        optflow.init(-1);
    }
#endif

#if AC_PRECLAND_ENABLED
    // scheduler table specifies 400Hz, but we can call it no faster
    // than the scheduler loop rate:
    g2.precland.init(MIN(400, scheduler.get_loop_rate_hz()));
#endif

#if AP_ICENGINE_ENABLED
    g2.ice_control.init();
#endif

}

#if AP_FENCE_ENABLED
/*
  return true if a mode reason is an automatic mode change due to
  landing sequencing.
 */
static bool mode_reason_is_landing_sequence(const ModeReason reason)
{
    switch (reason) {
    case ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND:
    case ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL:
    case ModeReason::QRTL_INSTEAD_OF_RTL:
    case ModeReason::QLAND_INSTEAD_OF_RTL:
        return true;
    default:
        break;
    }
    return false;
}
#endif // AP_FENCE_ENABLED

// Check if this mode can be entered from the GCS
bool Plane::gcs_mode_enabled(const Mode::Number mode_num) const
{
    // List of modes that can be blocked, index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::MANUAL,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::STABILIZE,
        (uint8_t)Mode::Number::TRAINING,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::FLY_BY_WIRE_A,
        (uint8_t)Mode::Number::FLY_BY_WIRE_B,
        (uint8_t)Mode::Number::CRUISE,
        (uint8_t)Mode::Number::AUTOTUNE,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::TAKEOFF,
        (uint8_t)Mode::Number::AVOID_ADSB,
        (uint8_t)Mode::Number::GUIDED,
        (uint8_t)Mode::Number::THERMAL,
#if HAL_QUADPLANE_ENABLED
        (uint8_t)Mode::Number::QSTABILIZE,
        (uint8_t)Mode::Number::QHOVER,
        (uint8_t)Mode::Number::QLOITER,
        (uint8_t)Mode::Number::QACRO,
#if QAUTOTUNE_ENABLED
        (uint8_t)Mode::Number::QAUTOTUNE
#endif
#endif
    };

    return !block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list));
}

bool Plane::set_mode(Mode &new_mode, const ModeReason reason)
{

    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        // only make happy noise if using a different method to switch, this stops beeping for repeated change mode requests from GCS
        if ((reason != control_mode_reason) && (reason != ModeReason::INITIALISED)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

#if HAL_QUADPLANE_ENABLED
    if (new_mode.is_vtol_mode() && !plane.quadplane.available()) {
        // dont try and switch to a Q mode if quadplane is not enabled and initalized
        gcs().send_text(MAV_SEVERITY_INFO,"Q_ENABLE 0");
        // make sad noise
        if (reason != ModeReason::INITIALISED) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return false;
    }

#else
    if (new_mode.is_vtol_mode()) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        gcs().send_text(MAV_SEVERITY_INFO,"HAL_QUADPLANE_ENABLED=0");
        // make sad noise
        if (reason != ModeReason::INITIALISED) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return false;
    }
#endif  // HAL_QUADPLANE_ENABLED

#if AP_FENCE_ENABLED
    // may not be allowed to change mode if recovering from fence breach
    if (hal.util->get_soft_armed() &&
        fence.enabled() &&
        fence.option_enabled(AC_Fence::OPTIONS::DISABLE_MODE_CHANGE) &&
        fence.get_breaches() &&
        in_fence_recovery() &&
        !mode_reason_is_landing_sequence(reason)) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"Mode change to %s denied, in fence recovery", new_mode.name());
        AP_Notify::events.user_mode_change_failed = 1;
        return false;
    }
#endif

    // Check if GCS mode change is disabled via parameter
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled(new_mode.mode_number())) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"Mode change to %s denied, GCS entry disabled (FLTMODE_GCSBLOCK)", new_mode.name());
        return false;
    }

    // backup current control_mode and previous_mode
    Mode &old_previous_mode = *previous_mode;
    Mode &old_mode = *control_mode;

    // update control_mode assuming success
    // TODO: move these to be after enter() once start_command_callback() no longer checks control_mode
    previous_mode = control_mode;
    control_mode = &new_mode;
    const ModeReason  old_previous_mode_reason = previous_mode_reason;
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
        previous_mode_reason = old_previous_mode_reason;

        // make sad noise
        if (reason != ModeReason::INITIALISED) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return false;
    }

    // exit previous mode
    old_mode.exit();

    // log and notify mode change
#if HAL_LOGGING_ENABLED
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
#endif
    notify_mode(*control_mode);
    gcs().send_message(MSG_HEARTBEAT);

    // make happy noise
    if (reason != ModeReason::INITIALISED) {
        AP_Notify::events.user_mode_change = 1;
    }
    return true;
}

bool Plane::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");

    return set_mode_by_number(static_cast<Mode::Number>(new_mode), reason);
}

bool Plane::set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason)
{
    Mode *new_mode = plane.mode_from_mode_num(new_mode_number);
    if (new_mode == nullptr) {
        notify_no_such_mode(new_mode_number);
        return false;
    }
    return set_mode(*new_mode, reason);
}

void Plane::check_long_failsafe()
{
    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    const uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if (failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS && flight_stage != AP_FixedWing::FlightStage::LAND) {
        uint32_t radio_timeout_ms = failsafe.last_valid_rc_ms;
        if (failsafe.state == FAILSAFE_SHORT) {
            // time is relative to when short failsafe enabled
            radio_timeout_ms = failsafe.short_timer_ms;
        }
        if (failsafe.rc_failsafe &&
            (tnow - radio_timeout_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_LONG, ModeReason::RADIO_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == &mode_auto &&
                   gcs_last_seen_ms != 0 &&
                   (tnow - gcs_last_seen_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        } else if ((g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT ||
                    g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI) &&
                   gcs_last_seen_ms != 0 &&
                   (tnow - gcs_last_seen_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs().chan(0) != nullptr &&
                   gcs().chan(0)->last_radio_status_remrssi_ms() != 0 &&
                   (tnow - gcs().chan(0)->last_radio_status_remrssi_ms()) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        }
    } else {
        uint32_t timeout_seconds = g.fs_timeout_long;
        if (g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
            // avoid dropping back into short timeout
            timeout_seconds = g.fs_timeout_short;
        }
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - gcs_last_seen_ms) < timeout_seconds*1000) {
            failsafe_long_off_event(ModeReason::GCS_FAILSAFE);
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.rc_failsafe) {
            failsafe_long_off_event(ModeReason::RADIO_FAILSAFE);
        }
    }
}

void Plane::check_short_failsafe()
{
    // only act on changes
    // -------------------
    if (g.fs_action_short != FS_ACTION_SHORT_DISABLED &&
       failsafe.state == FAILSAFE_NONE &&
       flight_stage != AP_FixedWing::FlightStage::LAND) {
        // The condition is checked and the flag rc_failsafe is set in radio.cpp
        if(failsafe.rc_failsafe) {
            failsafe_short_on_event(FAILSAFE_SHORT, ModeReason::RADIO_FAILSAFE);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.rc_failsafe || g.fs_action_short == FS_ACTION_SHORT_DISABLED) {
            failsafe_short_off_event(ModeReason::RADIO_FAILSAFE);
        }
    }
}


void Plane::startup_INS(void)
{
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
    } else {
        gcs().send_text(MAV_SEVERITY_ALERT, "Skipping INS calibration");
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::FIXED_WING);
    ahrs.set_wind_estimation_enabled(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();
}

// sets notify object flight mode information
void Plane::notify_mode(const Mode& mode)
{
    notify.flags.flight_mode = mode.mode_number();
    notify.set_flight_mode_str(mode.name4());
}

#if HAL_LOGGING_ENABLED
/*
  should we log a message type now?
 */
bool Plane::should_log(uint32_t mask)
{
    return logger.should_log(mask);
}
#endif

/*
  return throttle percentage from 0 to 100 for normal use and -100 to 100 when using reverse thrust
 */
int8_t Plane::throttle_percentage(void)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() && !quadplane.tailsitter.in_vtol_transition()) {
        return quadplane.motors->get_throttle_out() * 100.0;
    }
#endif
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (!have_reverse_thrust()) {
        return constrain_int16(throttle, 0, 100);
    }
    return constrain_int16(throttle, -100, 100);
}
