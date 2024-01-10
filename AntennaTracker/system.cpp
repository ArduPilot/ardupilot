#include "Tracker.h"

// mission storage
static const StorageAccess wp_storage(StorageManager::StorageMission);

void Tracker::init_ardupilot()
{
    // initialise stats module
    stats.init();

    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // initialise notify
    notify.init();
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.pre_arm_gps_check = true;

    // initialise battery
    battery.init();

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();
    // update_send so that if the first packet we receive happens to
    // be an arm message we don't trigger an internal error when we
    // try to initialise stream rates in the main loop.
    gcs().update_send();

#if HAL_LOGGING_ENABLED
    log_init();
#endif

#if AP_SCRIPTING_ENABLED
    scripting.init();
#endif // AP_SCRIPTING_ENABLED

    // initialise compass
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // GPS Initialization
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    ahrs.init();
    ahrs.set_fly_forward(false);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    barometer.calibrate();

#if HAL_LOGGING_ENABLED
    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&tracker, &Tracker::Log_Write_Vehicle_Startup_Messages, void));
#endif

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    // initialise servos
    init_servos();

    // use given start positions - useful for indoor testing, and
    // while waiting for GPS lock
    // sanity check location
    if (fabsf(g.start_latitude) <= 90.0f && fabsf(g.start_longitude) <= 180.0f) {
        current_loc.lat = g.start_latitude * 1.0e7f;
        current_loc.lng = g.start_longitude * 1.0e7f;
    } else {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Ignoring invalid START_LATITUDE or START_LONGITUDE parameter");
    }

    // see if EEPROM has a default location as well
    if (current_loc.lat == 0 && current_loc.lng == 0) {
        get_home_eeprom(current_loc);
    }

    hal.scheduler->delay(1000); // Why????

    Mode *newmode = mode_from_mode_num((Mode::Number)g.initial_mode.get());
    if (newmode == nullptr) {
        newmode = &mode_manual;
    }
    set_mode(*newmode, ModeReason::STARTUP);

    if (g.startup_delay > 0) {
        // arm servos with trim value to allow them to start up (required
        // for some servos)
        prepare_servos();
    }
}

/*
  fetch HOME from EEPROM
*/
bool Tracker::get_home_eeprom(Location &loc) const
{
    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (g.command_total.get() == 0) {
        return false;
    }

    // read WP position
    loc = {
        int32_t(wp_storage.read_uint32(5)),
        int32_t(wp_storage.read_uint32(9)),
        int32_t(wp_storage.read_uint32(1)),
        Location::AltFrame::ABSOLUTE
    };

    return true;
}

bool Tracker::set_home_eeprom(const Location &temp)
{
    wp_storage.write_byte(0, 0);
    wp_storage.write_uint32(1, temp.alt);
    wp_storage.write_uint32(5, temp.lat);
    wp_storage.write_uint32(9, temp.lng);

    // Now have a home location in EEPROM
    g.command_total.set_and_save(1); // At most 1 entry for HOME
    return true;
}

bool Tracker::set_home(const Location &temp)
{
    // check EKF origin has been set
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        if (!ahrs.set_home(temp)) {
            return false;
        }
    }

    if (!set_home_eeprom(temp)) {
        return false;
    }

    current_loc = temp;

    return true;
}

void Tracker::arm_servos()
{
    hal.util->set_soft_armed(true);
#if HAL_LOGGING_ENABLED
    logger.set_vehicle_armed(true);
#endif
}

void Tracker::disarm_servos()
{
    hal.util->set_soft_armed(false);
#if HAL_LOGGING_ENABLED
    logger.set_vehicle_armed(false);
#endif
}

/*
  setup servos to trim value after initialising
 */
void Tracker::prepare_servos()
{
    start_time_ms = AP_HAL::millis();
    SRV_Channels::set_output_limit(SRV_Channel::k_tracker_yaw, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_tracker_pitch, SRV_Channel::Limit::TRIM);
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
}

void Tracker::set_mode(Mode &newmode, const ModeReason reason)
{
    control_mode_reason = reason;

    if (mode == &newmode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    mode = &newmode;

    if (mode->requires_armed_servos()) {
        arm_servos();
    } else {
        disarm_servos();
    }

#if HAL_LOGGING_ENABLED
	// log mode change
	logger.Write_Mode((uint8_t)mode->number(), reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);

    nav_status.bearing = ahrs.yaw_sensor * 0.01f;
}

bool Tracker::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    Mode *fred = nullptr;
    switch ((Mode::Number)new_mode) {
    case Mode::Number::INITIALISING:
        return false;
    case Mode::Number::AUTO:
        fred = &mode_auto;
        break;
    case Mode::Number::MANUAL:
        fred = &mode_manual;
        break;
    case Mode::Number::SCAN:
        fred = &mode_scan;
        break;
    case Mode::Number::SERVOTEST:
        fred = &mode_servotest;
        break;
    case Mode::Number::STOP:
        fred = &mode_stop;
        break;
    case Mode::Number::GUIDED:
        fred = &mode_guided;
        break;
    }
    if (fred == nullptr) {
        return false;
    }
    set_mode(*fred, reason);
    return true;
}

#if HAL_LOGGING_ENABLED
/*
  should we log a message type now?
 */
bool Tracker::should_log(uint32_t mask)
{
    if (!logger.should_log(mask)) {
        return false;
    }
    return true;
}
#endif

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

#if AP_ADVANCEDFAILSAFE_ENABLED
// dummy method to avoid linking AFS
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) {return false;}
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#endif  // AP_ADVANCEDFAILSAFE_ENABLED
#if HAL_ADSB_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif
