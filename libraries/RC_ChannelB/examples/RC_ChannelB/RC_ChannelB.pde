/*
	Example of RC_Channel library.
	Code by Jason Short. 2010 
	DIYDrones.com

*/

#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <RC_Channel.h> 	// ArduPilot Mega RC Library


#define EE_RADIO_1 0x00	// all gains stored from here
#define EE_RADIO_2 0x06	// all gains stored from here
#define EE_RADIO_3 0x0C	// all gains stored from here
#define EE_RADIO_4 0x12	// all gains stored from here
#define EE_RADIO_5 0x18	// all gains stored from here
#define EE_RADIO_6 0x1E	// all gains stored from here
#define EE_RADIO_7 0x24	// all gains stored from here
#define EE_RADIO_8 0x2A	// all gains stored from here


RC_Channel rc_1(EE_RADIO_1);
RC_Channel rc_2(EE_RADIO_2);
RC_Channel rc_3(EE_RADIO_3);
RC_Channel rc_4(EE_RADIO_4);
RC_Channel rc_5(EE_RADIO_5);
RC_Channel rc_6(EE_RADIO_6);
RC_Channel rc_7(EE_RADIO_7);
RC_Channel rc_8(EE_RADIO_8);

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

void setup()
{
	Serial.begin(38400);
	Serial.println("ArduPilot RC Channel test");
	APM_RC.Init();		// APM Radio initialization

	delay(1000);
	// setup radio
	
	// read eepom or set manually
	/*
	rc_1.radio_min = 1100;
	rc_1.radio_max = 1900;
	
	rc_2.radio_min = 1100;
	rc_2.radio_max = 1900;

	rc_3.radio_min = 1100;
	rc_3.radio_max = 1900;

	rc_4.radio_min = 1100;
	rc_4.radio_max = 1900;
	*/


	rc_1.load_eeprom();
	rc_2.load_eeprom();
	rc_3.load_eeprom();
	rc_4.load_eeprom();
	rc_5.load_eeprom();
	rc_6.load_eeprom();
	rc_7.load_eeprom();
	rc_8.load_eeprom();
	
	print_radio_values();
	
	// set type of output, symmetrical angles or a number range;
	rc_1.set_angle(4500);
	rc_2.set_angle(4500);
	rc_3.set_range(0,1000);
	rc_4.set_angle(3000);
	rc_5.set_range(0,1000);
	rc_6.set_range(0,1000);
	rc_7.set_range(0,1000);
	rc_8.set_range(0,1000);
	
	// set midpoint value
	rc_1.set_pwm(APM_RC.InputCh(CH_1));
	rc_2.set_pwm(APM_RC.InputCh(CH_2));
	rc_4.set_pwm(APM_RC.InputCh(CH_4));

	rc_1.trim();
	rc_2.trim();
	rc_4.trim();
	
	
}

void loop()	
{
	delay(20);
	rc_1.set_pwm(APM_RC.InputCh(CH_1));
	rc_2.set_pwm(APM_RC.InputCh(CH_2));
	rc_3.set_pwm(APM_RC.InputCh(CH_3));
	rc_4.set_pwm(APM_RC.InputCh(CH_4));
	rc_5.set_pwm(APM_RC.InputCh(CH_5));
	rc_6.set_pwm(APM_RC.InputCh(CH_6));
	rc_7.set_pwm(APM_RC.InputCh(CH_7));
	rc_8.set_pwm(APM_RC.InputCh(CH_8));
	
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
	Serial.print("\tch5 :");
	Serial.print(rc_5.control_in, DEC);
	Serial.print("\tch6 :");
	Serial.print(rc_6.control_in, DEC);
	Serial.print("\tch7 :");
	Serial.print(rc_7.control_in, DEC);
	Serial.print("\tch8 :");
	Serial.println(rc_8.control_in, DEC);
}

void 
print_radio_values()
{
	Serial.print("CH1: ");
	Serial.print(rc_1.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_1.radio_max, DEC);

	Serial.print("CH2: ");
	Serial.print(rc_2.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_2.radio_max, DEC);

	Serial.print("CH3: ");
	Serial.print(rc_3.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_3.radio_max, DEC);

	Serial.print("CH4: ");
	Serial.print(rc_4.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_4.radio_max, DEC);

	Serial.print("CH5: ");
	Serial.print(rc_5.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_5.radio_max, DEC);

	Serial.print("CH6: ");
	Serial.print(rc_6.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_6.radio_max, DEC);

	Serial.print("CH7: ");
	Serial.print(rc_7.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_7.radio_max, DEC);

	Serial.print("CH8: ");
	Serial.print(rc_8.radio_min, DEC);
	Serial.print(" | ");
	Serial.println(rc_8.radio_max, DEC);
}
