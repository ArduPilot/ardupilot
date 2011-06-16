// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

void init_barometer(void)
{
	#if HIL_MODE == HIL_MODE_SENSORS
		hil.update();					// look for inbound hil packets for initialization
	#endif


	// We take some readings...
	for(int i = 0; i < 200; i++){
		delay(25);

		#if HIL_MODE == HIL_MODE_SENSORS
			hil.update(); 				// look for inbound hil packets
		#endif

		// Get initial data from absolute pressure sensor
		ground_pressure 	= read_baro_filtered();
		ground_temperature	= (ground_temperature * 9 + barometer.Temp) / 10;

	}
	abs_pressure  		= ground_pressure;
	ground_temperature 	= barometer.Temp;

	//Serial.printf("abs_pressure %ld\n", ground_temperature);
	//SendDebugln("barometer calibration complete.");
}

long read_baro_filtered(void)
{
	long pressure = 0;

	// get new data from absolute pressure sensor
	barometer.Read();

	// add new data into our filter
	baro_filter[baro_filter_index] = barometer.Press;
	baro_filter_index++;

	// loop our filter
	if(baro_filter_index == BARO_FILTER_SIZE)
		baro_filter_index = 0;

	// zero out our accumulator

	// sum our filter
	for(byte i = 0; i < BARO_FILTER_SIZE; i++){
		pressure += baro_filter[i];
	}

	// average our sampels
	return pressure /= BARO_FILTER_SIZE;
}

long read_barometer(void)
{
 	float x, scaling, temp;

	abs_pressure = read_baro_filtered();

	//Serial.printf("%ld, %ld, %ld, %ld\n", barometer.RawTemp, barometer.RawPress, barometer.Press, abs_pressure);

	scaling 				= (float)ground_pressure / (float)abs_pressure;
	temp 					= ((float)ground_temperature / 10.0f) + 273.15f;
	x 						= log(scaling) * temp * 29271.267f;
	return 	(x / 10);
}

// in M/S * 100
void read_airspeed(void)
{

}

void zero_airspeed(void)
{

}

#endif // HIL_MODE != HIL_MODE_ATTITUDE

void read_battery(void)
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

	#if BATTERY_EVENT == 1
		if(battery_voltage < LOW_VOLTAGE)	low_battery_event();
		if(g.battery_monitoring == 4 && current_total > g.pack_capacity)	low_battery_event();
	#endif
}

//v: 10.9453, a: 17.4023, mah: 8.2
