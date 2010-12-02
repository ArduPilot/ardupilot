/*
	Example of APM_Compass library (HMC5843 sensor).
	Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
*/

#include <Wire.h>
#include <AP_Compass_HMC5843.h> // Compass Library
#include <AP_Math.h>		// ArduPilot Mega Vector/Matrix math Library

AP_Compass_HMC5843 compass;

unsigned long timer;

void setup()
{	
	compass.init();	 // Initialization
	Serial.begin(38400);
	Serial.println("AP Compass library test (HMC5843)");
	delay(1000);
	timer = millis();
}

void loop()
{
	float tmp;
	
	if((millis()- timer) > 100){
		timer = millis();
		compass.read();
		compass.calculate(0, 0);	// roll = 0, pitch = 0 for this example
	    Serial.print("Heading:");
		Serial.print(compass.heading, DEC);
    	Serial.print("  (");
		Serial.print(compass.mag_x);
	    Serial.print(",");
		Serial.print(compass.mag_y);
    	Serial.print(",");
		Serial.print(compass.mag_z);
   		Serial.println(" )");
	}
}