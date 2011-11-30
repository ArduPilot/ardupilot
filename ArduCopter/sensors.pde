// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

static void ReadSCP1000(void) {}

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
	#if HIL_MODE == HIL_MODE_SENSORS
		gcs_update();					// look for inbound hil packets for initialization
	#endif

	ground_temperature = barometer.get_temperature();
	int i;

	// We take some readings...
	for(i = 0; i < 60; i++){
		delay(20);

		// get new data from absolute pressure sensor
		barometer.read();

		//Serial.printf("init %ld, %d, -, %ld, %ld\n", barometer.RawTemp, barometer.Temp, barometer.RawPress,  barometer.Press);
	}

	for(i = 0; i < 20; i++){
		delay(20);

		#if HIL_MODE == HIL_MODE_SENSORS
			gcs_update(); 				// look for inbound hil packets
		#endif

		// Get initial data from absolute pressure sensor
		barometer.read();
		ground_pressure = barometer.get_pressure();
		ground_temperature	= (ground_temperature * 9 + barometer.get_temperature()) / 10;
		//Serial.printf("init %ld, %d, -, %ld, %ld, -, %d, %ld\n", barometer.RawTemp, barometer.Temp, barometer.RawPress,  barometer.Press, ground_temperature, ground_pressure);
	}

	abs_pressure  		= ground_pressure;

	//Serial.printf("init %ld\n", abs_pressure);
	//SendDebugln("barometer calibration complete.");
}

/*
static int32_t read_baro_filtered(void)
{

	// get new data from absolute pressure sensor
	barometer.Read();

	return barometer.Press;

	int32_t pressure = 0;
	// add new data into our filter
	baro_filter[baro_filter_index] = barometer.Press;
	baro_filter_index++;

	// loop our filter
	if(baro_filter_index >= BARO_FILTER_SIZE)
		baro_filter_index = 0;

	// zero out our accumulator

	// sum our filter
	for(byte i = 0; i < BARO_FILTER_SIZE; i++){
		pressure += baro_filter[i];
	}


	// average our sampels
	return pressure /= BARO_FILTER_SIZE;
	//
}
*/
static int32_t read_barometer(void)
{
 	float x, scaling, temp;

	barometer.read();
	abs_pressure = barometer.get_pressure();


	//Serial.printf("%ld, %ld, %ld, %ld\n", barometer.RawTemp, barometer.RawPress, barometer.Press, abs_pressure);

	scaling 				= (float)ground_pressure / (float)abs_pressure;
	temp 					= ((float)ground_temperature / 10.0f) + 273.15f;
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
		current_total	 += current_amps * 0.0278;	// called at 100ms on average
	}

	#if BATTERY_EVENT == 1
	//if(battery_voltage < g.low_voltage)
	//	low_battery_event();

	if((battery_voltage < g.low_voltage) || (g.battery_monitoring == 4 && current_total > g.pack_capacity)){
		low_battery_event();

		#if PIEZO_LOW_VOLTAGE == 1
		// Only Activate if a battery is connected to avoid alarm on USB only
		if (battery_voltage1 > 1){
			piezo_on();
		}else{
			piezo_off();
		}
		#endif

	}else{
		#if PIEZO_LOW_VOLTAGE == 1
			piezo_off();
		#endif
	}
	#endif
}

//v: 10.9453, a: 17.4023, mah: 8.2
