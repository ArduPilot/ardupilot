/*
	Example of PID library.
	2010 Code by Jason Short. DIYDrones.com
*/

#include <PID.h> // ArduPilot Mega RC Library
#include <APM_RC.h> // ArduPilot RC Library

PID pid;
long radio_in;
long radio_trim;

void setup()
{
	Serial.begin(38400);
	Serial.println("ArduPilot Mega PID library test");
	APM_RC.Init();		// APM Radio initialization

	delay(1000);
	//rc.trim();
	radio_trim = APM_RC.InputCh(0);

	pid.set_P(1);
	pid.set_I(0);
	pid.set_D(.5);
	pid.set_imax(50);
}

void loop()
{
	delay(20);
	//rc.read_pwm();
	long error 		= APM_RC.InputCh(0) - radio_trim;
	long control 	= pid.get_pid(error, 20, 1);

	Serial.print("control: ");
	Serial.println(control,DEC);
}