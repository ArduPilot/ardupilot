// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  /*
    #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
	    sonar.calculate_scaler(g.sonar_type, 3.3);
	#else
        sonar.calculate_scaler(g.sonar_type, 5.0);
	#endif
*/
}
#endif

#if LITE == DISABLED
// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

#endif // HIL_MODE != HIL_MODE_ATTITUDE
#endif

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
		current_amps1	 = CURRENT_AMPS(current_pin.read_average());
		current_total1	 += current_amps1 * (float)delta_ms_medium_loop * 0.0002778;				// .0002778 is 1/3600 (conversion to hours)
	}
	
	#if BATTERY_EVENT == ENABLED
		if(battery_voltage1 < LOW_VOLTAGE)	low_battery_event();
		if(g.battery_monitoring == 4 && current_total1 > g.pack_capacity)	low_battery_event();
	#endif
}

