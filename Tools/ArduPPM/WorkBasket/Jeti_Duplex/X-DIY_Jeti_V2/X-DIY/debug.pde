#if DEBUG_SUBSYSTEM == 1
void debug_subsystem()
{
	Serial.println("GPS Subsystem, RAW OUTPUT");
	
	while(1){
		if(Serial1.available() > 0)	// Ok, let me see, the buffer is empty?
		{
			delay(60);	// wait for it all
			while(Serial1.available() > 0){
				byte incoming = Serial1.read();
				//Serial.print(incoming,DEC);
				Serial.print(incoming, BYTE); // will output Hex values
				//Serial.print(",");
			}
			Serial.println(" ");
		}

	}
}
#endif

#if DEBUG_SUBSYSTEM == 2
void debug_subsystem()
{
	Serial.println("Control Switch");
	
	Serial.print("Control CH ");
	Serial.println(flight_mode_channel, DEC);

	while(1){
		delay(20);
		byte switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			Serial.printf_P(PSTR("Position %d\n"), i, switchPosition);
			oldSwitchPosition = switchPosition;
		}
	}
}
#endif

#if DEBUG_SUBSYSTEM == 3
void debug_subsystem()
{
	Serial.println("DIP Switch Test");
	
	while(1){
		delay(100);
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

