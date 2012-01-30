/*
	Example of AP_RC_Channel library.
	Code by Jason Short. 2010
	DIYDrones.com

*/

#include <AP_RC.h>			// ArduPilot RC Library
#include <AP_RC_Channel.h> 	// ArduPilot RC Library


#define EE_RADIO_1 0x00	// all gains stored from here
#define EE_RADIO_2 0x06	// all gains stored from here
#define EE_RADIO_3 0x0C	// all gains stored from here
#define EE_RADIO_4 0x12	// all gains stored from here

AP_RC_Channel rc_1(EE_RADIO_1);
AP_RC_Channel rc_2(EE_RADIO_2);
AP_RC_Channel rc_3(EE_RADIO_3);
AP_RC_Channel rc_4(EE_RADIO_4);

AP_RC rc;

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3

void setup()
{
	Serial.begin(115200);

	Serial.println("ArduPilot RC Channel test");
	rc.init();		// APM Radio initialization


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
	rc_1.set_angle(4500);
	rc_2.set_angle(4500);
	rc_3.set_range(0,1000);
	rc_4.set_angle(4500);

	rc_1.dead_zone = 85;
	rc_2.dead_zone = 85;
	rc_3.dead_zone = 85;
	rc_4.dead_zone = 85;

	for (byte i = 0; i < 30; i++){
		read_radio();
	}
	rc_1.trim();
	rc_2.trim();
	rc_4.trim();
}

void loop()
{
	delay(20);
	read_radio();
	rc_1.servo_out = rc_1.control_in;
	rc_2.servo_out = rc_2.control_in;
	rc_3.servo_out = rc_3.control_in;
	rc_4.servo_out = rc_4.control_in;

	rc_1.calc_pwm();
	rc_2.calc_pwm();
	rc_3.calc_pwm();
	rc_4.calc_pwm();

	print_pwm();
	print_control_in();

	// send values to the PWM timers for output
	// ----------------------------------------
	rc.output_ch_pwm(CH_1, rc_1.radio_out); // send to Servos
	rc.output_ch_pwm(CH_2, rc_2.radio_out); // send to Servos
	rc.output_ch_pwm(CH_3, rc_3.radio_out); // send to Servos
	rc.output_ch_pwm(CH_4, rc_4.radio_out); // send to Servos
}


void read_radio()
{
	rc_1.set_pwm(rc.input_ch(CH_1));
	rc_2.set_pwm(rc.input_ch(CH_2));
	rc_3.set_pwm(rc.input_ch(CH_3));
	rc_4.set_pwm(rc.input_ch(CH_4));
	//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"), rc_1.control_in, rc_2.control_in, rc_3.control_in, rc_4.control_in);
}

void print_pwm()
{
	Serial.print("1: ");
	Serial.print(rc_1.radio_out, DEC);
	Serial.print("\t2: ");
	Serial.print(rc_2.radio_out, DEC);
	Serial.print("\t3:");
	Serial.print(rc_3.radio_out, DEC);
	Serial.print("\t4:");
	Serial.print(rc_4.radio_out	, DEC);
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

	rc_1.radio_max = rc_1.radio_in;
	rc_2.radio_max = rc_2.radio_in;
	rc_3.radio_max = rc_3.radio_in;
	rc_4.radio_max = rc_4.radio_in;

	rc_1.radio_trim = rc_1.radio_in;
	rc_2.radio_trim = rc_2.radio_in;
	rc_4.radio_trim = rc_4.radio_in;

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

		if(Serial.available() > 0){
			//rc_3.radio_max += 250;
			Serial.flush();

			Serial.println("Radio calibrated, Showing control values:");
			break;
		}
	}
	return;
}