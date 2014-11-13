// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();

    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

static void init_rangefinder(void)
{
    rangefinder.init();
}

/*
  read the rangefinder and update height estimate
 */
static void read_rangefinder(void)
{
    rangefinder.update();

    if (should_log(MASK_LOG_SONAR))
        Log_Write_Sonar();

    rangefinder_height_update();
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

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }
    }

    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }
}

static void zero_airspeed(bool in_startup)
{
    airspeed.calibrate(in_startup);
    read_airspeed();
    // update barometric calibration with new airspeed supplied temperature
    barometer.update_calibration();
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

