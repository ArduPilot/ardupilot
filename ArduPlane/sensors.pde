// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();

    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

static void init_sonar(void)
{
    sonar.init();
}

// read the sonars
static void read_sonars(void)
{
    sonar.update();

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
        if (should_log(MASK_LOG_IMU)) {
            Log_Write_Airspeed();
        }
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
    compass.set_current(battery.current_amps());

    if (!usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
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

