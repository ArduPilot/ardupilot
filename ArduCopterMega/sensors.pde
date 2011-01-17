void ReadSCP1000(void) {}


void init_pressure_ground(void)
{
	for(int i = 0; i < 300; i++){		// We take some readings...
		delay(20);
		APM_BMP085.Read(); 	// Get initial data from absolute pressure sensor
		abs_pressure_ground = (abs_pressure_ground * 9l + APM_BMP085.Press) / 10l;
		ground_temperature 	= (ground_temperature * 9 + APM_BMP085.Temp) / 10;		
	}
	abs_pressure  = APM_BMP085.Press;
}

void read_barometer(void){
 	float x, scaling, temp;
 	
	APM_BMP085.Read(); 	// Get new data from absolute pressure sensor
	
	//abs_pressure 			= (abs_pressure + APM_BMP085.Press) >> 1;		// Small filtering
	abs_pressure 			= ((float)abs_pressure * .9) + ((float)APM_BMP085.Press * .1);		// large filtering
	scaling 				= (float)abs_pressure_ground / (float)abs_pressure;
	temp 					= ((float)ground_temperature / 10.f) + 273.15;
	x 						= log(scaling) * temp * 29271.267f;
	current_loc.alt 		= (long)(x / 10) + home.alt + baro_offset;		// Pressure altitude in centimeters
}

// in M/S * 100
void read_airspeed(void)
{

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



void read_current(void)
{
	current_voltage 	= CURRENT_VOLTAGE(analogRead(CURRENT_PIN1)) * .1 + battery_voltage1 * .9; //reads power sensor voltage pin
	current_amps 		= CURRENT_AMPS(analogRead(CURRENT_PIN2)) * .1 + battery_voltage2 * .9; //reads power sensor current pin
}
