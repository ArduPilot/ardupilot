/*
	Example of APM_Compass library (HMC5843 sensor).
	Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
*/

//#include <Wire.h>
#include <DCM.h> // Compass Library

unsigned long timer;

void setup()
{	
	DCM.init();	 // Initialization
	Serial.begin(38400);
	Serial.println("Compass library test (HMC5843)");
	delay(1000);
	timer = millis();
}

void loop()
{
	float tmp;
	
	if((millis()- timer) > 100){
		timer = millis();
		APM_Compass.Read();
		APM_Compass.Calculate(0, 0);	// roll = 0, pitch = 0 for this example
	  	  Serial.print("Heading:");
		Serial.print(ToDeg(APM_Compass.Heading));
    	Serial.print("  (");
		Serial.print(APM_Compass.Mag_X);
   		Serial.print(",");
		Serial.print(APM_Compass.Mag_Y);
		Serial.print(",");
		Serial.print(APM_Compass.Mag_Z);
		Serial.println(" )");
	}
}