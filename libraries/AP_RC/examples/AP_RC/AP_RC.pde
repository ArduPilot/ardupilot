/*
	Example of AP_RC library.
	Code by Jason Short. DIYDrones.com
*/

#include <AP_RC.h> // ArduPilot RC Library
AP_RC rc;

#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_RUDDER 3

int radio_in[4];

void setup()
{
	Serial.begin(38400);
	Serial.println("ArduPilot RC library test");
	rc.init();
	delay(1000);
}

void loop()	
{
	delay(20);
	for(int y = 0; y < 4; y++) { 
		radio_in[y] = rc.input_ch(y);
	}
	for(int y = 0; y < 4; y++) { 
		rc.output_ch_pwm(y, radio_in[y]); // send to Servos
	}
	print_pwm();
}

void print_pwm()
{
	Serial.print("ch1 ");
	Serial.print(radio_in[CH_ROLL], DEC);
	Serial.print("\tch2: ");
	Serial.print(radio_in[CH_PITCH], DEC);
	Serial.print("\tch3 :");
	Serial.print(radio_in[CH_THROTTLE], DEC);
	Serial.print("\tch4 :");
	Serial.println(radio_in[CH_RUDDER], DEC);
}