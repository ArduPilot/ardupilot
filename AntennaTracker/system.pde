// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// mission storage
static const StorageAccess wp_storage(StorageManager::StorageMission);

static void init_tracker()
{
    hal.uartA->begin(SERIAL0_BAUD, 128, SERIAL_BUFSIZE);

    // gps port
    hal.uartB->begin(38400, 256, 16);

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    hal.util->available_memory());

    // Check the EEPROM format version before loading any parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // reset the uartA baud rate after parameter load
    hal.uartA->begin(map_baudrate(g.serial0_baud));

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // init the GCS
    gcs[0].init(hal.uartA);

    // set up snooping on other mavlink destinations
    gcs[0].set_snoop(mavlink_snoop);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial1_baud), 128, SERIAL1_BUFSIZE);
    gcs[1].setup_uart(hal.uartC, map_baudrate(g.serial1_baud), 128, SERIAL1_BUFSIZE);

    mavlink_system.sysid = g.sysid_this_mav;

    if (g.compass_enabled==true) {
        if (!compass.init() || !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }

    // GPS Initialization
    gps.init(NULL);

    ahrs.init();
    ahrs.set_fly_forward(false);

    ins.init(AP_InertialSensor::WARM_START, ins_sample_rate);
    ahrs.reset();

    init_barometer();

    hal.uartA->set_blocking_writes(false);
    hal.uartB->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

    // initialise servos
    init_servos();

    // use given start positions - useful for indoor testing, and
    // while waiting for GPS lock
    current_loc.lat = g.start_latitude * 1.0e7f;
    current_loc.lng = g.start_longitude * 1.0e7f;

    // see if EEPROM has a default location as well
    if (current_loc.lat == 0 && current_loc.lng == 0) {
        get_home_eeprom(current_loc);
    }

    gcs_send_text_P(SEVERITY_LOW,PSTR("\nReady to track."));
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

// Level the tracker by calibrating the INS
// Requires that the tracker be physically 'level' and horizontal
static void calibrate_ins()
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move tracker"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, ins_sample_rate);
    ins.init_accel();
    ahrs.set_trim(Vector3f(0, 0, 0));
    ahrs.reset();
    init_barometer();
}

// updates the status of the notify objects
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

/*
  fetch HOME from EEPROM
*/
static bool get_home_eeprom(struct Location &loc)
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

static void set_home_eeprom(struct Location temp)
{
    wp_storage.write_byte(0, temp.options);
    wp_storage.write_uint32(1, temp.alt);
    wp_storage.write_uint32(5, temp.lat);
    wp_storage.write_uint32(9, temp.lng);

    // Now have a home location in EEPROM
    g.command_total.set_and_save(1); // At most 1 entry for HOME
}

static void set_home(struct Location temp)
{
    set_home_eeprom(temp);
    current_loc = temp;
}

static void arm_servos()
{    
    channel_yaw.enable_out();
    channel_pitch.enable_out();
}

static void disarm_servos()
{
    channel_yaw.disable_out();
    channel_pitch.disable_out();
}

/*
  setup servos to trim value after initialising
 */
static void prepare_servos()
{
    start_time_ms = hal.scheduler->millis();
    channel_yaw.radio_out = channel_yaw.radio_trim;
    channel_pitch.radio_out = channel_pitch.radio_trim;
    channel_yaw.output();
    channel_pitch.output();
}

static void set_mode(enum ControlMode mode)
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
        arm_servos();
        break;

    case STOP:
    case INITIALISING:
        disarm_servos();
        break;
    }
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
static bool mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case AUTO:
    case MANUAL:
    case SCAN:
    case STOP:
        set_mode((enum ControlMode)mode);
        return true;
    }
    return false;
}

static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud));
    }
#endif
}
