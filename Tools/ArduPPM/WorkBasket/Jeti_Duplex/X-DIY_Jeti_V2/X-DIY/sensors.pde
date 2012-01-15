void ReadSCP1000(void) {}


void read_airpressure(void){
 	double x;
 	
	APM_BMP085.Read(); 	// Get new data from absolute pressure sensor
	abs_press = APM_BMP085.Press;
	abs_press_filt = (abs_press); // + 2l * abs_press_filt) / 3l;		// Light filtering
	//temperature = (temperature * 9 + temp_unfilt) / 10;    We will just use the ground temp for the altitude calculation	 

	double p = (double)abs_press_gnd / (double)abs_press_filt;
	double temp = (float)ground_temperature / 10.f + 273.15f;
	x = log(p) * temp * 29271.267f;
	//x = log(p) * temp * 29.271267 * 1000;
	press_alt = (long)(x / 10) + ground_alt;		// Pressure altitude in centimeters
	//  Need to add comments for theory.....
}

// in M/S * 100
void read_airspeed(void)
{
	#if GPS_PROTOCOL != GPS_PROTOCOL_IMU	// Xplane will supply the airspeed
		airpressure_raw = ((float)APM_ADC.Ch(AIRSPEED_CH) * .25) + (airpressure_raw * .75);
		airpressure 	= (int)airpressure_raw - airpressure_offset;
		airpressure 	= max(airpressure, 0);
		airspeed 		= sqrt((float)airpressure * airspeed_ratio) * 100;
	#endif

	calc_airspeed_errors();
}

#if BATTERY_EVENT == 1
void read_battery(void)
{
	battery_voltage1 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1 + battery_voltage1 * .9;
	battery_voltage2 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN2)) * .1 + battery_voltage2 * .9;
	battery_voltage3 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN3)) * .1 + battery_voltage3 * .9;
	battery_voltage4 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN4)) * .1 + battery_voltage4 * .9;

	#if BATTERY_TYPE == 0
		 if(battery_voltage3 < LOW_VOLTAGE)
			low_battery_event();
		battery_voltage = battery_voltage3; // set total battery voltage, for telemetry stream
	#endif

	#if BATTERY_TYPE == 1
		if(battery_voltage4 < LOW_VOLTAGE)
			low_battery_event();
		battery_voltage = battery_voltage4; // set total battery voltage, for telemetry stream
	#endif	
}
#endif

void zero_airspeed(void)
{
	airpressure_raw = (float)APM_ADC.Ch(AIRSPEED_CH);
	for(int c = 0; c < 50; c++){
		delay(20);
		airpressure_raw = (airpressure_raw * .90) + ((float)APM_ADC.Ch(AIRSPEED_CH) * .10);	
	}
	airpressure_offset = airpressure_raw;	
}


