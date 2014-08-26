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

    uint16_t distance_cm = rangefinder.distance_cm();
    int16_t max_distance_cm = rangefinder.max_distance_cm();
    if (rangefinder.healthy() && distance_cm < max_distance_cm) {
        // correct the range for attitude (multiply by DCM.c.z, which
        // is cos(roll)*cos(pitch))
        float corrected_range = distance_cm * 0.01f * ahrs.get_dcm_matrix().c.z;
        if (rangefinder_state.in_range_count == 0) {
            // we've just come back into range, start with this value
            rangefinder_state.height_estimate = corrected_range;
        } else {
            // low pass filter to reduce noise. This runs at 50Hz, so
            // converges fast. We don't want too much filtering
            // though, as it would introduce lag which would delay flaring
            rangefinder_state.height_estimate = 0.75f * rangefinder_state.height_estimate + 0.25f * corrected_range;
        }
        // we consider ourselves to be fully in range when we have 10
        // good samples (0.2s)
        if (rangefinder_state.in_range_count < 10) {
            rangefinder_state.in_range_count++;
        } else {
            rangefinder_state.in_range = true;
        }
    } else {
        rangefinder_state.in_range_count = 0;
        rangefinder_state.in_range = false;
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

