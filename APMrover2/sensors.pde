// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if LITE == DISABLED
// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

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
	int flashcount = 0;
	long 			ground_pressure = 0;
	int 			ground_temperature;

	while (ground_pressure == 0 || !barometer.healthy) {
		barometer.read(); 					// Get initial data from absolute pressure sensor
                ground_pressure 	= baro_filter.apply(barometer.get_pressure());
		//ground_pressure 	= barometer.get_pressure();
		ground_temperature 	= barometer.get_temperature();
		mavlink_delay(20);
		//Serial.printf("barometer.Press %ld\n", barometer.get_pressure());
	}

	for(int i = 0; i < 30; i++){		// We take some readings...

		#if HIL_MODE == HIL_MODE_SENSORS
        gcs_update(); 				// look for inbound hil packets
		#endif
        
        do {
            barometer.read(); 				// Get pressure sensor
        } while (!barometer.healthy);
		ground_pressure 	= baro_filter.apply(barometer.get_pressure());
                //ground_pressure		= (ground_pressure * 9l   + barometer.get_pressure()) / 10l;
		ground_temperature	= (ground_temperature * 9 + barometer.get_temperature()) / 10;

		mavlink_delay(20);
		if(flashcount == 5) {
			digitalWrite(C_LED_PIN, LED_OFF);
			digitalWrite(A_LED_PIN, LED_ON);
			digitalWrite(B_LED_PIN, LED_OFF);
		}

		if(flashcount >= 10) {
			flashcount = 0;
			digitalWrite(C_LED_PIN, LED_ON);
			digitalWrite(A_LED_PIN, LED_OFF);
			digitalWrite(B_LED_PIN, LED_ON);
		}
		flashcount++;
	}
	
	g.ground_pressure.set_and_save(ground_pressure);
	g.ground_temperature.set_and_save(ground_temperature / 10.0f);
	abs_pressure = ground_pressure;
	
    Serial.printf_P(PSTR("abs_pressure %ld\n"), abs_pressure);
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("barometer calibration complete."));
}

static int32_t read_barometer(void)
{
 	float x, scaling, temp;

	barometer.read();		// Get new data from absolute pressure sensor
	float abs_pressure = baro_filter.apply(barometer.get_pressure());

	//abs_pressure 			= (abs_pressure + barometer.get_pressure()) >> 1;		// Small filtering
	//abs_pressure 			= ((float)abs_pressure * .7) + ((float)barometer.get_pressure() * .3);		// large filtering
	scaling 				= (float)g.ground_pressure / (float)abs_pressure;
	temp 					= ((float)g.ground_temperature) + 273.15f;
	x 						= log(scaling) * temp * 29271.267f;
	return 	(x / 10);
}


// in M/S * 100
static void read_airspeed(void)
{
}

static void zero_airspeed(void)
{
}

#endif // HIL_MODE != HIL_MODE_ATTITUDE
#endif

static void read_battery(void)
{
	if(g.battery_monitoring == 0) {
		battery_voltage1 = 0;
		return;
	}
	
	if(g.battery_monitoring == 3 || g.battery_monitoring == 4) 
		battery_voltage1 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN_1)) * .1 + battery_voltage1 * .9;
	if(g.battery_monitoring == 4) {
		current_amps1	 = CURRENT_AMPS(analogRead(CURRENT_PIN_1)) * .1 + current_amps1 * .9; 	//reads power sensor current pin
		current_total1	 += current_amps1 * (float)delta_ms_medium_loop * 0.0002778;				// .0002778 is 1/3600 (conversion to hours)
	}
	
	#if BATTERY_EVENT == ENABLED
		if(battery_voltage1 < LOW_VOLTAGE)	low_battery_event();
		if(g.battery_monitoring == 4 && current_total1 > g.pack_capacity)	low_battery_event();
	#endif
}

