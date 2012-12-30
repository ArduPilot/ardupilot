// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

static void ReadSCP1000(void) {
}

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar->calculate_scaler(g.sonar_type, 3.3);
  #else
    sonar->calculate_scaler(g.sonar_type, 5.0);
  #endif
}
 #endif

static void init_barometer(void)
{
    barometer.calibrate();
    ahrs.set_barometer(&barometer);
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static int32_t read_barometer(void)
{
    barometer.read();
    return baro_filter.apply(barometer.get_altitude() * 100.0);
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    int16_t temp_alt = sonar.read();

    if(temp_alt >= sonar.min_distance && temp_alt <= sonar.max_distance * 0.70) {
        sonar_alt_ok = true;
    }else{
        sonar_alt_ok = false;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = cos_pitch_x * cos_roll_x;
    temp = max(temp, 0.707);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void init_compass()
{
    compass.set_orientation(MAG_ORIENTATION);                                                   // set compass's orientation on aircraft
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_compass(&compass);
#endif
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    if( optflow.init() == false ) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("\nFailed to Init OptFlow "));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }else{
        // suspend timer while we set-up SPI communication
        hal.scheduler->suspend_timer_procs();

        optflow.set_orientation(OPTFLOW_ORIENTATION);   // set optical flow sensor's orientation on aircraft
        optflow.set_frame_rate(2000);                   // set minimum update rate (which should lead to maximum low light performance
        optflow.set_resolution(OPTFLOW_RESOLUTION);     // set optical flow sensor's resolution
        optflow.set_field_of_view(OPTFLOW_FOV);         // set optical flow sensor's field of view

        // resume timer
        hal.scheduler->resume_timer_procs();
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
#define BATTERY_FS_COUNTER  100     // 100 iterations at 10hz is 10 seconds
static void read_battery(void)
{
    static uint8_t low_battery_counter = 0;

    if(g.battery_monitoring == 0) {
        battery_voltage1 = 0;
        return;
    }

    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        batt_volt_analog_source->set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_analog_source->read_average());
    }
    if(g.battery_monitoring == 4) {
        batt_curr_analog_source->set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_analog_source->read_average());
        current_total1   += current_amps1 * 0.02778;            // called at 100ms on average, .0002778 is 1/3600 (conversion to hours)
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    if (!ap.low_battery && ( battery_voltage1 < g.low_voltage || (g.battery_monitoring == 4 && current_total1 > g.pack_capacity))) {
        low_battery_counter++;
        if( low_battery_counter >= BATTERY_FS_COUNTER ) {
            low_battery_counter = BATTERY_FS_COUNTER;   // ensure counter does not overflow
            low_battery_event();
        }
    }else{
        // reset low_battery_counter in case it was a temporary voltage dip
        low_battery_counter = 0;
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->read_latest();
    receiver_rssi = constrain(ret, 0, 255);
}
