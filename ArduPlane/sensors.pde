// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// filter altitude from the barometer with a low pass filter
static LowPassFilterInt32 altitude_filter;


static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate(mavlink_delay);

    // filter at 100ms sampling, with 0.7Hz cutoff frequency
    altitude_filter.set_cutoff_frequency(0.1, 0.7);

    ahrs.set_barometer(&barometer);
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// read the barometer and return the updated altitude in centimeters
// above the calibration altitude
static int32_t read_barometer(void)
{
    barometer.read();
    return altitude_filter.apply(barometer.get_altitude() * 100.0);
}

// in M/S * 100
static void read_airspeed(void)
{
    airspeed.read();
    calc_airspeed_errors();
}

static void zero_airspeed(void)
{
    airspeed.calibrate(mavlink_delay);
    gcs_send_text_P(SEVERITY_LOW,PSTR("zero airspeed calibrated"));
}

static void read_battery(void)
{
    if(g.battery_monitoring == 0) {
        battery_voltage1 = 0;
        return;
    }

    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        static AP_AnalogSource_Arduino batt_volt_pin(g.battery_volt_pin);
        // this copes with changing the pin at runtime
        batt_volt_pin.set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_pin.read_average());
    }
    if(g.battery_monitoring == 4) {
        static AP_AnalogSource_Arduino batt_curr_pin(g.battery_curr_pin);
        // this copes with changing the pin at runtime
        batt_curr_pin.set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_pin.read_average());
        current_total1   += current_amps1 * (float)delta_ms_medium_loop * 0.0002778;                                    // .0002778 is 1/3600 (conversion to hours)
    }

#if BATTERY_EVENT == ENABLED
    if(battery_voltage1 < LOW_VOLTAGE) low_battery_event();
    if(g.battery_monitoring == 4 && current_total1 > g.pack_capacity) low_battery_event();
#endif
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    RSSI_pin.set_pin(g.rssi_pin);
    float ret = RSSI_pin.read();
    receiver_rssi = constrain(ret, 0, 255);
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
