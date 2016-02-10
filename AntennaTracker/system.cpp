// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

// mission storage
static const StorageAccess wp_storage(StorageManager::StorageMission);

static void mavlink_snoop_static(const mavlink_message_t* msg)
{
    tracker.mavlink_snoop(msg);
}

static void mavlink_delay_cb_static()
{
    tracker.mavlink_delay_cb();
}

void Tracker::init_tracker()
{
    // initialise console serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit " THISFIRMWARE
                               "\n\nFree RAM: %u\n",
                          hal.util->available_memory());

    // Check the EEPROM format version before loading any parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // initialise serial ports
    serial_manager.init();

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // init the GCS and start snooping for vehicle data
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);
    gcs[0].set_snoop(mavlink_snoop_static);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // setup serial port for telem1 and start snooping for vehicle data
    gcs[1].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);
    gcs[1].set_snoop(mavlink_snoop_static);

    // setup serial port for telem2 and start snooping for vehicle data
    gcs[2].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 1);
    gcs[2].set_snoop(mavlink_snoop_static);

    // setup serial port for fourth telemetry port (not used by default) and start snooping for vehicle data
    gcs[3].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 2);
    gcs[3].set_snoop(mavlink_snoop_static);

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    if (g.compass_enabled==true) {
        if (!compass.init() || !compass.read()) {
            hal.console->println("Compass initialisation failed!");
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }

    // GPS Initialization
    gps.init(NULL, serial_manager);

    ahrs.init();
    ahrs.set_fly_forward(false);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    init_barometer();

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
        gcs_send_text(MAV_SEVERITY_NOTICE, "Ignoring invalid START_LATITUDE or START_LONGITUDE parameter");
    }

    // see if EEPROM has a default location as well
    if (current_loc.lat == 0 && current_loc.lng == 0) {
        get_home_eeprom(current_loc);
    }

    init_capabilities();

    gcs_send_text(MAV_SEVERITY_INFO,"Ready to track");
    hal.scheduler->delay(1000); // Why????

    set_mode(AUTO); // tracking

    if (g.startup_delay > 0) {
        // arm servos with trim value to allow them to start up (required
        // for some servos)
        prepare_servos();
    }

    // calibrate pressure on startup by default
    nav_status.need_altitude_calibration = true;
}

// updates the status of the notify objects
// should be called at 50hz
void Tracker::update_notify()
{
    notify.update();
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
    loc.options = wp_storage.read_byte(0);
    loc.alt = wp_storage.read_uint32(1);
    loc.lat = wp_storage.read_uint32(5);
    loc.lng = wp_storage.read_uint32(9);

    return true;
}

void Tracker::set_home_eeprom(struct Location temp)
{
    wp_storage.write_byte(0, temp.options);
    wp_storage.write_uint32(1, temp.alt);
    wp_storage.write_uint32(5, temp.lat);
    wp_storage.write_uint32(9, temp.lng);

    // Now have a home location in EEPROM
    g.command_total.set_and_save(1); // At most 1 entry for HOME
}

void Tracker::set_home(struct Location temp)
{
    set_home_eeprom(temp);
    current_loc = temp;
    GCS_MAVLINK::send_home_all(temp);
}

void Tracker::arm_servos()
{    
    channel_yaw.enable_out();
    channel_pitch.enable_out();
}

void Tracker::disarm_servos()
{
    channel_yaw.disable_out();
    channel_pitch.disable_out();
}

/*
  setup servos to trim value after initialising
 */
void Tracker::prepare_servos()
{
    start_time_ms = AP_HAL::millis();
    channel_yaw.radio_out = channel_yaw.radio_trim;
    channel_pitch.radio_out = channel_pitch.radio_trim;
    channel_yaw.output();
    channel_pitch.output();
}

void Tracker::set_mode(enum ControlMode mode)
{
    if(control_mode == mode) {
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
	DataFlash.Log_Write_Mode(control_mode);
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool Tracker::mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case AUTO:
    case MANUAL:
    case SCAN:
    case SERVO_TEST:
    case STOP:
        set_mode((enum ControlMode)mode);
        return true;
    }
    return false;
}

void Tracker::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
}

/*
  should we log a message type now?
 */
bool Tracker::should_log(uint32_t mask)
{
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    return true;
}
