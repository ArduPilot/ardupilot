#include <avr/io.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h> 		// ArduPilot Mega RC Library
 /*
ArduPilotMega radio test tool
Authors:	Doug Weibel, Jose Julio

*/


// GENERAL VARIABLE DECLARATIONS
// --------------------------------------------


/* Radio values
		Channel assignments	
			1	Ailerons (rudder if no ailerons)
			2	Elevator
			3	Throttle
			4	Rudder (if we have ailerons)
			5	TBD
			6 	TBD
			7	TBD
			8	Mode
*/
int radio_in[8];								// current values from the transmitter - microseconds
float servo_out[] 	= 	{0,0,0,0,0,0,0};
unsigned long fast_loopTimer		= 0;			// current values to the servos - -45 to 45 degrees, except [3] is 0 to 100

Arduino_Mega_ISR_Registry isr_registry;
APM_RC_APM1 APM_RC;

void setup() {
	Serial.begin(115200);
    isr_registry.init();
	APM_RC.Init(&isr_registry);    // APM Radio initialization
}


void loop()
{
	
	// -----------------------------------------------------------------
	if (millis()-fast_loopTimer > 199) {
		fast_loopTimer		= millis();

		read_radio();

		Serial.print("ch1: ");
		Serial.print(radio_in[0]);
		Serial.print("   ch2: ");
		Serial.print(radio_in[1]);
		Serial.print("   ch3: ");
		Serial.print(radio_in[2]);
		Serial.print("   ch4: ");
		Serial.print(radio_in[3]);
		Serial.print("   ch5: ");
		Serial.print(radio_in[4]);
		Serial.print("   ch6: ");
		Serial.print(radio_in[5]);
		Serial.print("   ch7: ");
		Serial.print(radio_in[6]);  
		Serial.print("   ch8: ");
		Serial.println(radio_in[7]);


	}
}


void read_radio()
{

	for (int y=0;y<8;y++) radio_in[y] = APM_RC.InputCh(y);

}
