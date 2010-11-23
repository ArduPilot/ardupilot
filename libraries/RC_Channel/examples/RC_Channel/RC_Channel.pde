/*
	Example of RC_Channel library.
	Code by Jason Short. 2010 
	DIYDrones.com

*/

#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <RC_Channel.h> 	// ArduPilot Mega RC Library

RC_Channel rc_1;
RC_Channel rc_2;
RC_Channel rc_3;
RC_Channel rc_4;

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3

void setup()
{
	Serial.begin(38400);
	Serial.println("ArduPilot RC Channel test");

	delay(1000);
	// setup radio
	
	// read eepom or set manually
	rc_1.set_radio_range(1100,1900);
	rc_2.set_radio_range(1100,1900);
	rc_3.set_radio_range(1100,1900);
	rc_4.set_radio_range(1100,1900);
	
	// set type of output, symmetrical angles or a number range;
	rc_1.set_angle(4500);
	rc_2.set_angle(4500);
	rc_3.set_range(0,1000);
	rc_4.set_angle(3000);
	
	// set midpoint value
	rc_1.set_trim(APM_RC.InputCh(CH_1));
	rc_2.set_trim(APM_RC.InputCh(CH_2));
	rc_3.set_trim(APM_RC.InputCh(CH_3));
	rc_4.set_trim(APM_RC.InputCh(CH_4));
	
}

void loop()	
{
	delay(20);
	rc_1.set_pwm(APM_RC.InputCh(CH_1));
	rc_2.set_pwm(APM_RC.InputCh(CH_2));
	rc_3.set_pwm(APM_RC.InputCh(CH_3));
	rc_4.set_pwm(APM_RC.InputCh(CH_4));
	print_pwm();
}

void print_pwm()
{
	Serial.print("ch1 ");
	Serial.print(rc_1.control_in, DEC);
	Serial.print("\tch2: ");
	Serial.print(rc_2.control_in, DEC);
	Serial.print("\tch3 :");
	Serial.print(rc_3.control_in, DEC);
	Serial.print("\tch4 :");
	Serial.print(rc_4.control_in, DEC);
}