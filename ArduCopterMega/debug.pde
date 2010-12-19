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
	Serial.println(FLIGHT_MODE_CHANNEL, DEC);

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

