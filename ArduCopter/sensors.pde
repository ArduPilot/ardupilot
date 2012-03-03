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
		ground_temperature	= (ground_temperature * 7 + barometer.get_temperature()) / 8;
		//Serial.printf("init %ld, %d, -, %ld, %ld, -, %d, %ld\n", barometer.RawTemp, barometer.Temp, barometer.RawPress,  barometer.Press, ground_temperature, ground_pressure);
	}
}

static void reset_baro(void)
{
	ground_pressure 	= barometer.get_pressure();
	ground_temperature	= barometer.get_temperature();
}

static int32_t read_barometer(void)
{
 	float x, scaling, temp;

	barometer.read();
	float abs_pressure = barometer.get_pressure();


	//Serial.printf("%ld, %ld, %ld, %ld\n", barometer.RawTemp, barometer.RawPress, barometer.Press, abs_pressure);

	scaling 				= (float)ground_pressure / abs_pressure;
	temp 					= ((float)ground_temperature / 10.0f) + 273.15f;
	x 						= log(scaling) * temp * 29271.267f;
	return 	(x / 10);
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void init_compass()
{
	compass.set_orientation(MAG_ORIENTATION);						// set compass's orientation on aircraft
	if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        Serial.println_P(PSTR("COMPASS INIT ERROR"));
        return;
    }
    dcm.set_compass(&compass);
    compass.get_offsets();					// load offsets to account for airframe magnetic interference
    compass.null_offsets_enable();
}

static void init_optflow()
{
#ifdef OPTFLOW_ENABLED
	if( optflow.init(false) == false ) {
	    g.optflow_enabled = false;
	    SendDebug("\nFailed to Init OptFlow ");
	}
	optflow.set_orientation(OPTFLOW_ORIENTATION);			// set optical flow sensor's orientation on aircraft
	optflow.set_frame_rate(2000);							// set minimum update rate (which should lead to maximum low light performance
	optflow.set_resolution(OPTFLOW_RESOLUTION);				// set optical flow sensor's resolution
	optflow.set_field_of_view(OPTFLOW_FOV);					// set optical flow sensor's field of view
	// setup timed read of sensor
	//timer_scheduler.register_process(&AP_OpticalFlow::read);
#endif
}

static void read_battery(void)
{

	if(g.battery_monitoring == 0){
		battery_voltage1 = 0;
		return;
	}

    if(g.battery_monitoring == 3 || g.battery_monitoring == 4)
		battery_voltage1 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN_1)) * .1 + battery_voltage1 * .9;
	if(g.battery_monitoring == 4) {
		current_amps1	 = CURRENT_AMPS(analogRead(CURRENT_PIN_1)) * .1 + current_amps1 * .9; 	//reads power sensor current pin
		current_total1	 += current_amps1 * 0.02778;	// called at 100ms on average, .0002778 is 1/3600 (conversion to hours)
	}

	#if BATTERY_EVENT == 1
	//if(battery_voltage < g.low_voltage)
	//	low_battery_event();

	if((battery_voltage1 < g.low_voltage) || (g.battery_monitoring == 4 && current_total1 > g.pack_capacity)){
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
