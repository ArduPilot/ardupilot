// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// filter altitude from the barometer with a low pass filter
static LowPassFilterInt32 altitude_filter;


static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();

    // filter at 100ms sampling, with 0.7Hz cutoff frequency
    altitude_filter.set_cutoff_frequency(0.1, 0.7);

    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// read the barometer and return the updated altitude in centimeters
// above the calibration altitude
static int32_t read_barometer(void)
{
    barometer.read();
    return altitude_filter.apply(barometer.get_altitude() * 100.0);
}

static void init_sonar(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    sonar.Init(&apm1_adc);
#else
    sonar.Init(NULL);
#endif
}

// read the sonars
static void read_sonars(void)
{
    if (!sonar.enabled()) {
        // this makes it possible to disable sonar at runtime
        return;
    }

    if (should_log(MASK_LOG_SONAR))
        Log_Write_Sonar();
}

/*
  ask airspeed sensor for a new value
 */
static void read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        calc_airspeed_errors();
    }
}

static void zero_airspeed(void)
{
    airspeed.calibrate();
    gcs_send_text_P(SEVERITY_LOW,PSTR("zero airspeed calibrated"));
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
static void read_battery(void)
{
    battery.read();

    if (!usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->voltage_average() * 50;
    receiver_rssi = constrain_int16(ret, 0, 255);
}


/*
  return current_loc.alt adjusted for ALT_OFFSET
  This is useful during long flights to account for barometer changes
  from the GCS, or to adjust the flying height of a long mission
 */
static int32_t adjusted_altitude_cm(void)
{
    return current_loc.alt - (g.alt_offset*100);
}

