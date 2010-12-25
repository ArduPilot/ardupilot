/*
	Example of RC_Channel library.
	Code by Jason Short. 2010 
	DIYDrones.com

*/

#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <RC_ChannelB.h> 	// ArduPilot Mega RC Library
#include <AP_Common.h>
#include <AP_EEProm.h>

RC_ChannelB rc[] = 
{
	RC_ChannelB()	
};

void setup()
{
	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");
	APM_RC.Init();		// APM Radio initialization

	delay(1000);
	// setup radio
	
}

void loop()	
{
	Serial.println("looping");
	delay(1000);
}
