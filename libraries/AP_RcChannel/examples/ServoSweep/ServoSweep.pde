/*
	Example of RC_Channel library.
	Code by James Goppert/ Jason Short. 2010 
	DIYDrones.com

*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_RcChannel.h> 	// ArduPilot Mega RC Library
#include <AP_EEProm.h>
#include <APM_RC.h>

FastSerialPort0(Serial); // make sure this procees variable declarations

// test settings
uint8_t nChannels = 1;

// channel configuration
AP_RcChannel rc[] = 
{
	AP_RcChannel("ROLL",APM_RC,0,100.0),
	/*
	AP_RcChannel("PITCH",APM_RC,1,45),
	AP_RcChannel("THR",APM_RC,2,100),
	AP_RcChannel("YAW",APM_RC,3,45),
	AP_RcChannel("CH5",APM_RC,4,1),
	AP_RcChannel("CH6",APM_RC,5,1),
	AP_RcChannel("CH7",APM_tC,6,1),
	AP_RcChannel("CH8",APM_RC,7,1)
	*/
};
// test position
float testPosition = 0;
int8_t testSign = 1;

void setup()
{
	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");
	APM_RC.Init();		// APM Radio initialization
}

void loop()	
{
	// set channel positions
	Serial.println("In Loop");

	for (int i=0;i<nChannels;i++) rc[i].setPosition(testPosition);
	Serial.printf("\ntestPosition (%f)\n",testPosition);
	for (int i=0;i<nChannels;i++) Serial.printf("%7s\t",rc[i].getName());
	Serial.println();
	Serial.printf("pwm      :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7d\t",rc[i].getPwm());
	Serial.println();
	Serial.printf("position :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7.2f\t",rc[i].getPosition());
	Serial.println();

	// update test value
	for (int i=0;i<nChannels;i++)
	{
		testPosition += testSign*.05;
		if (testPosition > 1)
		{
			testPosition = 1;
			testSign = -1;
		}
		else if (testPosition < -1)
		{
			testPosition = -1;
			testSign = 1;
		}
	}

	delay(500);
}
