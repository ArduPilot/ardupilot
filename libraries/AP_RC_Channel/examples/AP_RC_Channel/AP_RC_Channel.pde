/*
	Example of AP_RC_Channel library.
	Original Code by Jason Short. 2010
	Updates to support 2.6+ and channel curves by Ron Curry, 2012
	DIYDrones.com

*/

#include <AP_RC_Channel.h> 	// ArduPilot RC Library
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h> // ArduPilot Mega RC Library

Arduino_Mega_ISR_Registry isr_registry;
APM_RC_APM2 APM_RC;
/*
#define EE_RADIO_1 0x00	// all gains stored from here
#define EE_RADIO_2 0x06	// all gains stored from here
#define EE_RADIO_3 0x0C	// all gains stored from here
#define EE_RADIO_4 0x12	// all gains stored from here

AP_RC_Channel rc_1(EE_RADIO_1);
AP_RC_Channel rc_2(EE_RADIO_2);
AP_RC_Channel rc_3(EE_RADIO_3);
AP_RC_Channel rc_4(EE_RADIO_4);
*/
AP_RC_Channel rc_1;
AP_RC_Channel rc_2;
AP_RC_Channel rc_3;
AP_RC_Channel rc_4;
AP_RC_Channel rc_5;
AP_RC_Channel rc_6;
AP_RC_Channel rc_7;
AP_RC_Channel rc_8;

// curve1 demonstrates how the full range of the RC input is scaled to span only a range of 200 with increased resolution
//	Output is not clipped but rather scaled
static int curve1[] = { 3, 1400, 1500, 1600}; 

// curve2 demonstrates quick ramp with short throw from 1100 to 1400, then flat, high-res section from 1400 - 1600, then quick
//	ramp short throw from 1600 to 1950 - classic expo function as seen in most TX's
static int  curve2[] = {5, 1100, 1400, 1500, 1600, 1950};

// curve 3 demonstrats quick ramp/short thrown to 1300 then relative flat, high-rese section from 1300 to 1950 with a more
//	detailed definition
static int  curve3[] = {10, 1100, 1300, 1400, 1450, 1500, 1550, 1600, 1700, 1825, 1950};

// curve 4 demonstrates scaling to a larger PWM range in this case 900-2100 (from approx 1100-1900)
//	with other modifications this could be used to support output devices that require a different range of PWM values than normal
static int  curve4[] = {3, 100, 500, 900};


void setup()
{
    isr_registry.init();
	APM_RC.Init(&isr_registry);		// APM Radio initialization
	
	APM_RC.enable_out(CH_1);
	APM_RC.enable_out(CH_2);
	APM_RC.enable_out(CH_3);
	APM_RC.enable_out(CH_4);
	APM_RC.enable_out(CH_5);
	APM_RC.enable_out(CH_6);
	APM_RC.enable_out(CH_7);
	APM_RC.enable_out(CH_8);


	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");
	delay(500);

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

	// or

	rc_1.load_eeprom();
	rc_2.load_eeprom();
	rc_3.load_eeprom();
	rc_4.load_eeprom();

	*/

	// interactive setup
	setup_radio();

	print_radio_values();

	// set type of output, symmetrical angles or a number range;
	rc_1.set_range(0,1000);
	rc_2.set_range(0,1000);
	rc_3.set_range(0,1000);
	rc_4.set_range(0,1000);
	rc_5.set_range(0,1000);
	rc_6.set_angle(4500);
	rc_7.set_angle(4500);
	rc_8.set_angle(4500);
/*
	rc_1.dead_zone = 85;
	rc_2.dead_zone = 85;
	rc_3.dead_zone = 85;
	rc_4.dead_zone = 85;
	rc_5.dead_zone = 85;
	rc_6.dead_zone = 85;
	rc_7.dead_zone = 85;
	rc_8.dead_zone = 85;
*/
	for (byte i = 0; i < 30; i++){
		read_radio();
	}
	rc_1.trim();
	rc_2.trim();
	rc_4.trim();
	rc_5.trim();
	rc_6.trim();
	rc_7.trim();
	rc_8.trim();
	
	// Setup test curves
	rc_1.set_channel_curve(curve1);
	rc_2.set_channel_curve(curve2);
	rc_3.set_channel_curve(curve3);
	rc_4.set_channel_curve(curve4);
	rc_5.unset_channel_curve();
	rc_6.unset_channel_curve();
	rc_7.unset_channel_curve();
	rc_8.unset_channel_curve();
	
}

void loop()
{

	if (APM_RC.GetState() == 1)
	{
		read_radio();
		rc_1.servo_out = rc_1.control_in;
		rc_2.servo_out = rc_2.control_in;
		rc_3.servo_out = rc_3.control_in;
		rc_4.servo_out = rc_4.control_in;
		rc_5.servo_out = rc_5.control_in;
		rc_6.servo_out = rc_6.control_in;
		rc_7.servo_out = rc_7.control_in;
		rc_8.servo_out = rc_8.control_in;
		

		rc_1.calc_pwm();
		rc_2.calc_pwm();
		rc_3.calc_pwm();
		rc_4.calc_pwm();
		rc_5.calc_pwm();
		rc_6.calc_pwm();
		rc_7.calc_pwm();
		rc_8.calc_pwm();

		print_pwm();
		print_control_in();

		// send values to the PWM timers for output
		// ----------------------------------------
		APM_RC.OutputCh(CH_1, rc_1.radio_out); // send to Servos
		APM_RC.OutputCh(CH_2, rc_2.radio_out); // send to Servos
		APM_RC.OutputCh(CH_3, rc_3.radio_out); // send to Servos
		APM_RC.OutputCh(CH_4, rc_4.radio_out); // send to Servos
		APM_RC.OutputCh(CH_5, rc_5.radio_out); // send to Servos
		APM_RC.OutputCh(CH_6, rc_6.radio_out); // send to Servos
		APM_RC.OutputCh(CH_7, rc_7.radio_out); // send to Servos
		APM_RC.OutputCh(CH_8, rc_8.radio_out); // send to Servos
	}
}


void read_radio()
{
	rc_1.set_pwm(APM_RC.InputCh(CH_1));
	rc_2.set_pwm(APM_RC.InputCh(CH_2));
	rc_3.set_pwm(APM_RC.InputCh(CH_3));
	rc_4.set_pwm(APM_RC.InputCh(CH_4));
	rc_5.set_pwm(APM_RC.InputCh(CH_5));
	rc_6.set_pwm(APM_RC.InputCh(CH_6));
	rc_7.set_pwm(APM_RC.InputCh(CH_7));
	rc_8.set_pwm(APM_RC.InputCh(CH_8));
	//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"), rc_1.control_in, rc_2.control_in, rc_3.control_in, rc_4.control_in,
	//	rc_5.control_in, rc_6.control_in, rc_7.control_in, rc_8.control_in);
}

void print_pwm()
{
	Serial.print("\t1: ");
	Serial.print(rc_1.radio_out, DEC);
	Serial.print("\t2: ");
	Serial.print(rc_2.radio_out, DEC);
	Serial.print("\t3:");
	Serial.print(rc_3.radio_out, DEC);
	Serial.print("\t4:");
	Serial.print(rc_4.radio_in	, DEC);
/*	Serial.print("\t5");
	Serial.print(rc_5.radio_out, DEC);
	Serial.print("\t6: ");
	Serial.print(rc_6.radio_out, DEC);
	Serial.print("\t7:");
	Serial.print(rc_7.radio_out, DEC);
	Serial.print("\t8:");
	Serial.print(rc_8.radio_out	, DEC);
 */
}

// 1280
// 1536
// 1795
void print_control_in()
{
	Serial.print("\t1: ");
	Serial.print(rc_1.control_in, DEC);
	Serial.print("\t2: ");
	Serial.print(rc_2.control_in, DEC);
	Serial.print("\t3:");
	Serial.print(rc_3.control_in, DEC);
	Serial.print("\t4:");
	Serial.println(rc_4.control_in, DEC);
/*	Serial.print("\t5: ");
	Serial.print(rc_5.control_in, DEC);
	Serial.print("\t6: ");
	Serial.print(rc_6.control_in, DEC);
	Serial.print("\t7:");
	Serial.print(rc_7.control_in, DEC);
	Serial.print("\t8:");
	Serial.println(rc_8.control_in, DEC);
 */
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
/*	
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
 */
}


void
setup_radio()
{
	Serial.println("\n\nRadio Setup:");
	uint8_t i;

	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}

	rc_1.radio_min = rc_1.radio_in;
	rc_2.radio_min = rc_2.radio_in;
	rc_3.radio_min = rc_3.radio_in;
	rc_4.radio_min = rc_4.radio_in;
	rc_5.radio_min = rc_5.radio_in;
	rc_6.radio_min = rc_6.radio_in;
	rc_7.radio_min = rc_7.radio_in;
	rc_8.radio_min = rc_8.radio_in;

	rc_1.radio_max = rc_1.radio_in;
	rc_2.radio_max = rc_2.radio_in;
	rc_3.radio_max = rc_3.radio_in;
	rc_4.radio_max = rc_4.radio_in;
	rc_5.radio_max = rc_5.radio_in;
	rc_6.radio_max = rc_6.radio_in;
	rc_7.radio_max = rc_7.radio_in;
	rc_8.radio_max = rc_8.radio_in;

	rc_1.radio_trim = rc_1.radio_in;
	rc_2.radio_trim = rc_2.radio_in;
	rc_4.radio_trim = rc_4.radio_in;
	rc_5.radio_trim = rc_5.radio_in;
	rc_6.radio_trim = rc_6.radio_in;
	rc_7.radio_trim = rc_7.radio_in;
	rc_8.radio_trim = rc_8.radio_in;

	Serial.println("\nMove all controls to each extreme. Hit Enter to save:");
	while(1){

		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		rc_1.update_min_max();
		rc_2.update_min_max();
		rc_3.update_min_max();
		rc_4.update_min_max();
		rc_5.update_min_max();
		rc_6.update_min_max();
		rc_7.update_min_max();
		rc_8.update_min_max();

		if(Serial.available() > 0){
			//rc_3.radio_max += 250;
			Serial.flush();

			Serial.println("Radio calibrated, Showing control values:");
			break;
		}
	}
	return;
}