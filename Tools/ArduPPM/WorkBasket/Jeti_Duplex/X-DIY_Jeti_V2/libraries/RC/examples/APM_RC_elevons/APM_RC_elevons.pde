/*
	Example of APM2_RC library.
	Code by Jordi MuÒoz and Jose Julio. DIYDrones.com

	Print Input values and send Output to the servos
	(Works with last PPM_encoder firmware)
*/

#include <APM2_RC.h> // ArduPilot Mega RC Library
APM2_RC rc;

#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_RUDDER 3

void setup()
{
	Serial.begin(38400);
	Serial.println("ArduPilot Mega RC library test");
	
	//rc.set_channel_direction(CH_ROLL, -1);
	//rc.set_channel_direction(CH_PITCH, -1);
	//rc.set_channel_direction(CH_THROTTLE, -1);
	//rc.set_channel_direction(CH_RUDDER, -1);
	rc.set_mix_mode(1); // 1 = elevons, 0 = normal
	rc.init();
	delay(1000);
}

void loop()
{
	delay(20);
	rc.read_pwm();
	for(int y = 0; y < 8; y++) {
		rc.set_ch_pwm(y, rc.radio_in[y]); // send to Servos
	}
	//print_pwm();
}

void print_pwm()
{
	Serial.print("ch1 ");
	Serial.print(rc.radio_in[CH_ROLL], DEC);
	Serial.print("\tch2: ");
	Serial.print(rc.radio_in[CH_PITCH], DEC);
	Serial.print("\tch3 :");
	Serial.print(rc.radio_in[CH_THROTTLE], DEC);
	Serial.print("\tch4 :");
	Serial.print(rc.radio_in[CH_RUDDER], DEC);
	Serial.print("\tch5 ");
	Serial.print(rc.radio_in[4], DEC);
	Serial.print("\tch6: ");
	Serial.print(rc.radio_in[5], DEC);
	Serial.print("\tch7 :");
	Serial.print(rc.radio_in[6], DEC);
	Serial.print("\tch8 :");
	Serial.println(rc.radio_in[7], DEC);
}