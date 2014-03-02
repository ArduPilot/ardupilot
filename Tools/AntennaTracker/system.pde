// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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

    // reset the uartA baud rate after parameter load
    hal.uartA->begin(map_baudrate(g.serial0_baud, SERIAL0_BAUD));

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // init the GCS
    gcs0.init(hal.uartA);
    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD),
                     128, SERIAL2_BUFSIZE);
    gcs3.init(hal.uartC);

    mavlink_system.sysid = g.sysid_this_mav;

    if (g.compass_enabled==true) {
        if (!compass.init() || !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }

	// Do GPS init
	g_gps = &g_gps_driver;

    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_STATIONARY);

    mavlink_system.compid = 4;
    mavlink_system.type = MAV_TYPE_ANTENNA_TRACKER;

    ahrs.init();
    ahrs.set_fly_forward(false);

    ins.init(AP_InertialSensor::WARM_START, ins_sample_rate);
    ahrs.reset();

    init_barometer();

    hal.uartA->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

    // setup antenna control PWM channels
    channel_yaw.set_angle(18000); // Yaw is expected to drive antenna azimuth -180-0-180
    channel_pitch.set_angle(9000); // Pitch is expected to drive elevation -90-0-90

    channel_yaw.output_trim();
    channel_pitch.output_trim();

    channel_yaw.calc_pwm();
    channel_pitch.calc_pwm();

    channel_yaw.enable_out();
    channel_pitch.enable_out();

    home_loc = get_home_eeprom(); // GPS may update this later

    gcs_send_text_P(SEVERITY_LOW,PSTR("\nReady to track."));
    hal.scheduler->delay(1000); // Why????
}

// Level the tracker by calibrating the INS
// Requires that the tracker be physically 'level' and horizontal
static void calibrate_ins()
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move tracker"));
    ahrs.init();
    ahrs.set_fly_forward(true);
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
 *  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}

/*
  fetch HOME from EEPROM
*/
static struct Location get_home_eeprom()
{
    struct Location temp;
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (g.command_total.get() == 0) {
        memset(&temp, 0, sizeof(temp));
        temp.id = CMD_BLANK;
    }else{
        // read WP position
        mem = WP_START_BYTE;
        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);
    }

    return temp;
}

static void set_home_eeprom(struct Location temp)
{
    uint16_t mem = WP_START_BYTE;

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);

    // Now have a home location in EEPROM
    g.command_total.set_and_save(1); // At most 1 entry for HOME
}

static void set_home(struct Location temp)
{
    if (g.compass_enabled)
        compass.set_initial_location(temp.lat, temp.lng);
    set_home_eeprom(temp);
    home_loc = temp;
}
