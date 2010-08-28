/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Debug.pde
 Version  : v1.0, Aug 27, 2010
 Author(s): ArduCopter Team
             Ted Carancho (aeroquad), Jose Julio, Jordi Muñoz,
             Jani Hirvinen, Ken McEwans, Roberto Navoni,          
             Sandro Benigno, Chris Anderson
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.

* ************************************************************** *
ChangeLog:


* ************************************************************** *
TODO:


* ************************************************************** */


#if DEBUG_SUBSYSTEM == 1
void debug_subsystem()
{
	Serial.println("Begin Debug: Radio Subsystem ");
	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		Serial.print("Radio in ch1: ");
		Serial.print(radio_in[CH_ROLL], DEC);
		Serial.print("\tch2: ");
		Serial.print(radio_in[CH_PITCH], DEC);
		Serial.print("\tch3: ");
		Serial.print(radio_in[CH_THROTTLE], DEC);
		Serial.print("\tch4: ");
		Serial.print(radio_in[CH_RUDDER], DEC);
		Serial.print("\tch5: ");
		Serial.print(radio_in[4], DEC);
		Serial.print("\tch6: ");
		Serial.print(radio_in[5], DEC);
		Serial.print("\tch7: ");
		Serial.print(radio_in[6], DEC);
		Serial.print("\tch8: ");
		Serial.println(radio_in[7], DEC);
	}
}
#endif

#if DEBUG_SUBSYSTEM == 2
void debug_subsystem()
{
	Serial.println("Begin Debug: Servo Subsystem ");
	Serial.print("Reverse ROLL - CH1: ");
	Serial.println(reverse_roll, DEC);
	Serial.print("Reverse PITCH - CH2: ");
	Serial.println(reverse_pitch, DEC);
	Serial.print("Reverse THROTTLE - CH3: ");
	Serial.println(REVERSE_THROTTLE, DEC);
	Serial.print("Reverse RUDDER - CH4: ");
	Serial.println(reverse_rudder, DEC);

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	radio_max[0]				 = 	CH1_MAX;
	radio_max[1]				 = 	CH2_MAX;
	radio_max[2]				 = 	CH3_MAX;
	radio_max[3]				 = 	CH4_MAX;
	radio_max[4]				 = 	CH5_MAX;
	radio_max[5]				 = 	CH6_MAX;
	radio_max[6]				 = 	CH7_MAX;
	radio_max[7]				 = 	CH8_MAX;
	radio_min[0]				 = 	CH1_MIN;
	radio_min[1]				 = 	CH2_MIN;
	radio_min[2]				 = 	CH3_MIN;
	radio_min[3]				 = 	CH4_MIN;
	radio_min[4]				 = 	CH5_MIN;
	radio_min[5]				 = 	CH6_MIN;
	radio_min[6]				 = 	CH7_MIN;
	radio_min[7]				 = 	CH8_MIN;

	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		update_servo_switches();

		servo_out[CH_ROLL]	= ((radio_in[CH_ROLL]	- radio_trim[CH_ROLL])	* 45.0f	* reverse_roll) / 500;
		servo_out[CH_PITCH] = ((radio_in[CH_PITCH] - radio_trim[CH_PITCH]) * 45.0f	* reverse_roll) / 500;
		servo_out[CH_RUDDER] = ((radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 45.0f	* reverse_rudder) / 500;

		// write out the servo PWM values
		// ------------------------------
		set_servos_4();
		
		
		for(int y = 4; y < 8; y++) { 
			radio_out[y] = constrain(radio_in[y], 	radio_min[y], 	radio_max[y]);	
			APM_RC.OutputCh(y, radio_out[y]); // send to Servos
		}

		/*
		Serial.print("Servo_out ch1: ");
		Serial.print(servo_out[CH_ROLL], DEC);
		Serial.print("\tch2: ");
		Serial.print(servo_out[CH_PITCH], DEC);
		Serial.print("\tch3: ");
		Serial.print(servo_out[CH_THROTTLE], DEC);
		Serial.print("\tch4: ");
		Serial.print(servo_out[CH_RUDDER], DEC);
		*/
		///*
		Serial.print("ch1: ");
		Serial.print(radio_out[CH_ROLL], DEC);
		Serial.print("\tch2: ");
		Serial.print(radio_out[CH_PITCH], DEC);
		Serial.print("\tch3: ");
		Serial.print(radio_out[CH_THROTTLE], DEC);
		Serial.print("\tch4: ");
		Serial.print(radio_out[CH_RUDDER], DEC);
		Serial.print("\tch5: ");
		Serial.print(radio_out[4], DEC);
		Serial.print("\tch6: ");
		Serial.print(radio_out[5], DEC);
		Serial.print("\tch7: ");
		Serial.print(radio_out[6], DEC);
		Serial.print("\tch8: ");
		Serial.println(radio_out[7], DEC);
		
		//*/
	}
}
#endif


#if DEBUG_SUBSYSTEM == 3
void debug_subsystem()
{
	Serial.println("Begin Debug: Analog Sensor Subsystem ");
	
	Serial.print("AirSpeed sensor enabled: ");
	Serial.println(AIRSPEED_SENSOR, DEC);
		
	Serial.print("Enable Battery: ");
	Serial.println(BATTERY_EVENT, DEC);
	zero_airspeed();
	
	Serial.print("Air pressure offset:");
	Serial.println(airpressure_offset, DEC);

	while(1){
		delay(20);
		read_z_sensor();
		read_XY_sensors();
		read_battery();
		
		Serial.print("Analog IN:");
		Serial.print("  0:");
		Serial.print(analog0, DEC);
		Serial.print(", 1: ");
		Serial.print(analog1, DEC);
		Serial.print(", 2: ");
		Serial.print(analog2, DEC);
		Serial.print(", 3: ");
		Serial.print(airpressure_raw, DEC);
		
		Serial.print("\t\tSensors:");
		Serial.print("  airSpeed:");
		Serial.print(airspeed, DEC);
		Serial.print("m \tbattV:");
		Serial.print(battery_voltage, DEC);
		Serial.println("volts ");
	}
}
#endif

#if DEBUG_SUBSYSTEM == 4
void debug_subsystem()
{
	Serial.println("Begin Debug: GPS Subsystem ");
	
	while(1){
		delay(333);

		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		//update_GPS_light();
		
		GPS.Read();
		
		if (GPS.NewData){
			Serial.print("Lat:");
			Serial.print(GPS.Lattitude, DEC);
			Serial.print(" Lon:");
			Serial.print(GPS.Longitude, DEC);
			Serial.print(" Alt:");
			Serial.print(GPS.Altitude / 100, DEC);
			Serial.print("m, gs: ");
			Serial.print(GPS.Ground_Speed / 100, DEC);
			Serial.print(" COG:");
			Serial.print(GPS.Ground_Course / 1000l);
			Serial.print(" SAT:");
			Serial.print(GPS.NumSats, DEC);
			Serial.print(" FIX:");
			Serial.print(GPS.Fix, DEC);
			Serial.print(" TIM:");
			Serial.print(GPS.Time);
			Serial.println();
		}
	}
}
#endif

#if DEBUG_SUBSYSTEM == 5
void debug_subsystem()
{
	Serial.println("Begin Debug: GPS Subsystem, RAW OUTPUT");
	
	while(1){
		if(Serial1.available() > 0)	// Ok, let me see, the buffer is empty?
		{
			
			delay(60);	// wait for it all
			while(Serial1.available() > 0){
				byte incoming = Serial1.read();
				//Serial.print(incoming,DEC);
				Serial.print(incoming, HEX); // will output Hex values
				Serial.print(",");
			}
			Serial.println(" ");
		}

	}
}
#endif

#if DEBUG_SUBSYSTEM == 6
void debug_subsystem()
{
	Serial.println("Begin Debug: IMU Subsystem ");
	startup_IMU_ground();
	
	while(1){
		delay(20);
		read_AHRS();
		
		// We are using the IMU
		// ---------------------
		Serial.print("  roll:");
		Serial.print(roll_sensor / 100, DEC);	
		Serial.print("  pitch:");
		Serial.print(pitch_sensor / 100, DEC);
		Serial.print("  yaw:");
		Serial.println(yaw_sensor / 100, DEC);
		
	}
}
#endif

#if DEBUG_SUBSYSTEM == 7
void debug_subsystem()
{
	Serial.println("Begin Debug: Control Switch Test");
	
	while(1){
		delay(20);
		byte switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			
			switch(switchPosition)
			{
				case 1: // First position
				Serial.println("Position 1");

				break;
		
				case 2: // middle position
				Serial.println("Position 2");
				break;
		
				case 3: // last position
				Serial.println("Position 3");
				break;

				case 4: // last position
				Serial.println("Position 4");
				break;

				case 5: // last position
				Serial.println("Position 5 - Software Manual");
				break;

				case 6: // last position
				Serial.println("Position 6 - Hardware MUX, Manual");
				break;

			}
	
			oldSwitchPosition = switchPosition;
		}
	}
}
#endif

#if DEBUG_SUBSYSTEM == 8
void debug_subsystem()
{
	Serial.println("Begin Debug: Control Switch Test");
	
	while(1){
		delay(20);
		update_servo_switches();
		if (mix_mode == 0) {
			Serial.print("Mix:standard ");
			Serial.print("\t reverse_roll: ");
			Serial.print(reverse_roll, DEC);
			Serial.print("\t reverse_pitch: ");
			Serial.print(reverse_pitch, DEC);
			Serial.print("\t reverse_rudder: ");
			Serial.println(reverse_rudder, DEC);
		} else {
			Serial.print("Mix:elevons ");
			Serial.print("\t reverse_elevons: ");
			Serial.print(reverse_elevons, DEC);
			Serial.print("\t reverse_ch1_elevon: ");
			Serial.print(reverse_ch1_elevon, DEC);
			Serial.print("\t reverse_ch2_elevon: ");
			Serial.println(reverse_ch2_elevon, DEC);
		}
	}
}
#endif


#if DEBUG_SUBSYSTEM == 9
void debug_subsystem()
{
	Serial.println("Begin Debug: Relay");
	pinMode(RELAY_PIN, OUTPUT);
	
	while(1){
		delay(3000);
	
		Serial.println("Relay Position A");
		PORTL |= B00000100;
		delay(3000);
	
		Serial.println("Relay Position B");
		PORTL ^= B00000100;		
	}
}
#endif

#if DEBUG_SUBSYSTEM == 10
void debug_subsystem()
{
	Serial.println("Begin Debug: Magnatometer");
	
	while(1){
		delay(3000);
	}
}
#endif

#if DEBUG_SUBSYSTEM == 11
void debug_subsystem()
{
	ground_alt = 0;
	Serial.println("Begin Debug: Absolute Airpressure");	
	while(1){
		delay(20);
		read_airpressure();
		Serial.print("Air Pressure Altitude: ");
		Serial.print(press_alt / 100, DEC);
		Serial.println("meters");
	}
}
#endif

#if DEBUG_SUBSYSTEM == 12
void debug_subsystem()
{
	ground_alt = 0;
	Serial.println("Begin Debug: Display Waypoints");	
	delay(500);

	eeprom_busy_wait();
	uint8_t options 	= eeprom_read_byte((uint8_t *) EE_CONFIG);

	eeprom_busy_wait();
	int32_t hold = eeprom_read_dword((uint32_t *) EE_ALT_HOLD_HOME);

	// save the alitude above home option
	if(options & HOLD_ALT_ABOVE_HOME){
		Serial.print("Hold this altitude over home: ");
		Serial.print(hold / 100, DEC);
		Serial.println(" meters");
	}else{
		Serial.println("Maintain current altitude ");
	}
	
	Serial.print("Number of Waypoints: ");
	Serial.println(wp_total, DEC);

	Serial.print("Hit radius for Waypoints: ");
	Serial.println(wp_radius, DEC);

	Serial.print("Loiter radius around Waypoints: ");
	Serial.println(loiter_radius, DEC);
	Serial.println(" ");
	
	for(byte i = 0; i < wp_total; i++){
		struct Location temp = get_wp_with_index(i);
		print_waypoint(&temp, i);
	}
	
	while(1){
	}

}
#endif



#if DEBUG_SUBSYSTEM == 13
void debug_subsystem()
{
	Serial.println("Begin Debug: Throttle Subsystem ");
	read_radio();
	
	uint16_t low_pwm = radio_in[CH_THROTTLE];
	uint16_t high_pwm = radio_in[CH_THROTTLE];
	
	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		//update_throttle();
		set_servos_4();
		low_pwm = min(low_pwm, radio_in[CH_THROTTLE]);
		high_pwm = max(high_pwm, radio_in[CH_THROTTLE]);
		
		Serial.print("Radio_in: ");
		Serial.print(radio_in[CH_THROTTLE], DEC);
		Serial.print("\tPWM output: ");
		Serial.print(radio_out[CH_THROTTLE], DEC);
		Serial.print("\tThrottle: ");
		Serial.print(servo_out[CH_THROTTLE], DEC);
		Serial.print("\tPWM Min: ");
		Serial.print(low_pwm, DEC);
		Serial.print("\tPWM Max: ");
		Serial.println(high_pwm, DEC);
	}
}
#endif


#if DEBUG_SUBSYSTEM == 14
void debug_subsystem()
{
	Serial.println("Begin Debug: Radio Min Max ");
	uint16_t low_pwm[8];
	uint16_t high_pwm[8];
	uint8_t i;
	
	for(i = 0; i < 100; i++){
		delay(20);
		read_radio();
	}
	
	for(i = 0; i < 8; i++){
		radio_min[i] = 0;
		radio_max[i] = 2400;
	 	low_pwm[i]	= radio_in[i];
 		high_pwm[i]	= radio_in[i];
	}
	
	while(1){
		delay(100);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		for(i = 0; i < 8; i++){
			low_pwm[i] = min(low_pwm[i], radio_in[i]);
			high_pwm[i] = max(high_pwm[i], radio_in[i]);
		}

		for(i = 0; i < 8; i++){
			Serial.print("CH");
			Serial.print(i + 1, DEC);
			Serial.print(": ");
			low_pwm[i] = min(low_pwm[i], radio_in[i]);
			Serial.print(low_pwm[i], DEC);
			Serial.print("|");
			high_pwm[i] = max(high_pwm[i], radio_in[i]);
			Serial.print(high_pwm[i], DEC);
			Serial.print("   ");
		}
		Serial.println(" ");
	}
}
#endif


#if DEBUG_SUBSYSTEM == 15
void debug_subsystem()
{
	Serial.println("Begin Debug: EEPROM Dump ");
	uint16_t temp;
	for(int n = 0; n < 512; n++){
		for(int i = 0; i < 4; i++){
			temp = eeprom_read_word((uint16_t *) mem);
			mem += 2;
			Serial.print(temp, HEX);
			Serial.print("  ");
		}
		Serial.print("  ");
		for(int i = 0; i < 4; i++){
			temp = eeprom_read_word((uint16_t *) mem);
			mem += 2;
			Serial.print(temp, HEX);
			Serial.print("  ");
		}
	}
}
#endif
