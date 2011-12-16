// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

static void init_barometer(void)
{
	int flashcount = 0;
	long 			ground_pressure = 0;
	int 			ground_temperature;

	while(ground_pressure == 0){
		barometer.read(); 					// Get initial data from absolute pressure sensor
		ground_pressure 	= barometer.get_pressure();
		ground_temperature 	= barometer.get_temperature();
		mavlink_delay(20);
		//Serial.printf("barometer.Press %ld\n", barometer.get_pressure());
	}

	for(int i = 0; i < 30; i++){		// We take some readings...

		#if HIL_MODE == HIL_MODE_SENSORS
			gcs_update(); 				// look for inbound hil packets
		#endif

		barometer.read(); 				// Get initial data from absolute pressure sensor
		ground_pressure		= (ground_pressure * 9l   + barometer.get_pressure()) / 10l;
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

static long read_barometer(void)
{
 	float x, scaling, temp;

	barometer.read();		// Get new data from absolute pressure sensor


	//abs_pressure 			= (abs_pressure + barometer.get_pressure()) >> 1;		// Small filtering
	abs_pressure 			= ((float)abs_pressure * .7) + ((float)barometer.get_pressure() * .3);		// large filtering
	scaling 				= (float)g.ground_pressure / (float)abs_pressure;
	temp 					= ((float)g.ground_temperature) + 273.15f;
	x 						= log(scaling) * temp * 29271.267f;
	return 	(x / 10);
}

// in M/S * 100
static void read_airspeed(void)
{
	#if GPS_PROTOCOL != GPS_PROTOCOL_IMU	// Xplane will supply the airspeed
		if (g.airspeed_offset == 0) {
            // runtime enabling of airspeed, we need to do instant
            // calibration before we can use it. This isn't as
            // accurate as the 50 point average in zero_airspeed(),
            // but it is better than using it uncalibrated
            airspeed_raw = pitot_analog_source.read();
            g.airspeed_offset.set_and_save(airspeed_raw);
		}
		airspeed_raw 		= (pitot_analog_source.read() * 0.1) + (airspeed_raw * 0.9);
		airspeed_pressure 	= max((airspeed_raw - g.airspeed_offset), 0);
		airspeed 			= sqrt(airspeed_pressure * g.airspeed_ratio) * 100;
	#endif

	calc_airspeed_errors();
}

static void zero_airspeed(void)
{
    float sum = 0;
    int c;
	airspeed_raw = pitot_analog_source.read();
	for(c = 0; c < 250; c++) {
		delay(2);
		sum += pitot_analog_source.read();
	}
    sum /= c;
	g.airspeed_offset.set_and_save((int16_t)sum);
}

#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void read_battery(void)
{
	battery_voltage1 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1 + battery_voltage1 * .9;
	battery_voltage2 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN2)) * .1 + battery_voltage2 * .9;
	battery_voltage3 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN3)) * .1 + battery_voltage3 * .9;
	battery_voltage4 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN4)) * .1 + battery_voltage4 * .9;

	if(g.battery_monitoring == 1) 
		battery_voltage = battery_voltage3; // set total battery voltage, for telemetry stream	
	if(g.battery_monitoring == 2) 
		battery_voltage = battery_voltage4;
	if(g.battery_monitoring == 3 || g.battery_monitoring == 4) 
		battery_voltage = battery_voltage1;
	if(g.battery_monitoring == 4) {
		current_amps	 = CURRENT_AMPS(analogRead(CURRENT_PIN_1)) * .1 + current_amps * .9; //reads power sensor current pin
		current_total	 += current_amps * (float)delta_ms_medium_loop * 0.000278;
	}
	
	#if BATTERY_EVENT == ENABLED
		if(battery_voltage < LOW_VOLTAGE)	low_battery_event();
		if(g.battery_monitoring == 4 && current_total > g.pack_capacity)	low_battery_event();
	#endif
}

