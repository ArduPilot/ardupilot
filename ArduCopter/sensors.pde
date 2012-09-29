// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

static void ReadSCP1000(void) {
}

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar.calculate_scaler(g.sonar_type, 3.3);
  #else
    sonar.calculate_scaler(g.sonar_type, 5.0);
  #endif
}
 #endif

static void init_barometer(void)
{
    barometer.calibrate(mavlink_delay);
    ahrs.set_barometer(&barometer);
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static int32_t read_barometer(void)
{
    barometer.read();
    return baro_filter.apply(barometer.get_altitude() * 100.0);
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void init_compass()
{
    compass.set_orientation(MAG_ORIENTATION);                                                   // set compass's orientation on aircraft
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        Serial.println_P(PSTR("COMPASS INIT ERROR"));
        return;
    }
    ahrs.set_compass(&compass);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_compass(&compass);
#endif
}

static void init_optflow()
{
#ifdef OPTFLOW_ENABLED
    if( optflow.init(false, &timer_scheduler) == false ) {
        g.optflow_enabled = false;
        SendDebug("\nFailed to Init OptFlow ");
    }
    // suspend timer while we set-up SPI communication
    timer_scheduler.suspend_timer();

    optflow.set_orientation(OPTFLOW_ORIENTATION);                       // set optical flow sensor's orientation on aircraft
    optflow.set_frame_rate(2000);                                                       // set minimum update rate (which should lead to maximum low light performance
    optflow.set_resolution(OPTFLOW_RESOLUTION);                                 // set optical flow sensor's resolution
    optflow.set_field_of_view(OPTFLOW_FOV);                                     // set optical flow sensor's field of view

    // resume timer
    timer_scheduler.resume_timer();
#endif
}

static void read_battery(void)
{

    if(g.battery_monitoring == 0) {
        battery_voltage1 = 0;
        return;
    }

    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        static AP_AnalogSource_Arduino bat_pin(BATTERY_PIN_1);
        battery_voltage1 = BATTERY_VOLTAGE(bat_pin.read_average());
    }
    if(g.battery_monitoring == 4) {
        static AP_AnalogSource_Arduino current_pin(CURRENT_PIN_1);
        current_amps1    = CURRENT_AMPS(current_pin.read_average());
        current_total1   += current_amps1 * 0.02778;            // called at 100ms on average, .0002778 is 1/3600 (conversion to hours)
    }

#if BATTERY_EVENT == ENABLED
    //if(battery_voltage < g.low_voltage)
    //	low_battery_event();

    if((battery_voltage1 < g.low_voltage) || (g.battery_monitoring == 4 && current_total1 > g.pack_capacity)) {
        low_battery_event();

 #if COPTER_LEDS == ENABLED
        if ( bitRead(g.copter_leds_mode, 3) ) {         // Only Activate if a battery is connected to avoid alarm on USB only
            if (battery_voltage1 > 1) {
                piezo_on();
            }else{
                piezo_off();
            }
        }


    }else if ( bitRead(g.copter_leds_mode, 3) ) {
        piezo_off();
 #endif                // COPTER_LEDS
    }
#endif         //BATTERY_EVENT
}

//v: 10.9453, a: 17.4023, mah: 8.2
