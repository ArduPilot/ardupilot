#include "Tracker.h"

// mission storage
static const StorageAccess wp_storage(StorageManager::StorageMission);

static void mavlink_delay_cb_static()
{
    tracker.mavlink_delay_cb();
}

void Tracker::init_tracker()
{
    // initialise console serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    // Check the EEPROM format version before loading any parameters from EEPROM
    load_parameters();

    // initialise stats module
    stats.init();

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

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

#ifdef ENABLE_SCRIPTING
    if (!scripting.init()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Scripting failed to start");
    }
#endif // ENABLE_SCRIPTING

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

    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&tracker, &Tracker::Log_Write_Vehicle_Startup_Messages, void));

    // set serial ports non-blocking
    serial_manager.set_blocking_writes_all(false);

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

    gcs().send_text(MAV_SEVERITY_INFO,"Ready to track");
    hal.scheduler->delay(1000); // Why????

    switch (g.initial_mode) {
    case MANUAL:
        set_mode(MANUAL, MODE_REASON_STARTUP);
        break;

    case SCAN:
        set_mode(SCAN, MODE_REASON_STARTUP);
        break;

    case STOP:
        set_mode(STOP, MODE_REASON_STARTUP);
        break;

    case AUTO:
    default:
        set_mode(AUTO, MODE_REASON_STARTUP);
        break;
    }

    if (g.startup_delay > 0) {
        // arm servos with trim value to allow them to start up (required
        // for some servos)
        prepare_servos();
    }

    // disable safety if requested
    BoardConfig.init_safety();    
}

/*
  fetch HOME from EEPROM
*/
bool Tracker::get_home_eeprom(struct Location &loc)
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
    logger.set_vehicle_armed(true);
}

void Tracker::disarm_servos()
{
    hal.util->set_soft_armed(false);
    logger.set_vehicle_armed(false);
}

/*
  setup servos to trim value after initialising
 */
void Tracker::prepare_servos()
{
    start_time_ms = AP_HAL::millis();
    SRV_Channels::set_output_limit(SRV_Channel::k_tracker_yaw, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_tracker_pitch, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
}

void Tracker::set_mode(enum ControlMode mode, mode_reason_t reason)
{
    if (control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    control_mode = mode;

	switch (control_mode) {
    case AUTO:
    case MANUAL:
    case SCAN:
    case SERVO_TEST:
        arm_servos();
        break;

    case STOP:
    case INITIALISING:
        disarm_servos();
        break;
    }

	// log mode change
	logger.Write_Mode(control_mode, reason);
  gcs().send_message(MSG_HEARTBEAT);

    nav_status.bearing = ahrs.yaw_sensor * 0.01f;
}

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
