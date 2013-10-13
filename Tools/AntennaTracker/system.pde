// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_tracker()
{
    hal.uartA->begin(SERIAL0_BAUD, 128, SERIAL_BUFSIZE);

    // gps port
    hal.uartB->begin(38400, 256, 16);

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());

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
    channel_yaw.set_angle(4500);
    channel_pitch.set_angle(4500);

    channel_yaw.output_trim();
    channel_pitch.output_trim();

    channel_yaw.calc_pwm();
    channel_pitch.calc_pwm();

    channel_yaw.enable_out();
    channel_pitch.enable_out();

    gcs_send_text_P(SEVERITY_LOW,PSTR("\nReady to track."));
    hal.scheduler->delay(1000);
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


