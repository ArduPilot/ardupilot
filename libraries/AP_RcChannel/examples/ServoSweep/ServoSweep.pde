/*
	Example of RC_Channel library.
	Code by James Goppert/ Jason Short. 2010 
	DIYDrones.com

*/

#define AP_DISPLAYMEM
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_RcChannel.h> 	// ArduPilot Mega RC Library
#include <AP_EEProm.h>
#include <APM_RC.h>

FastSerialPort0(Serial); // make sure this procees variable declarations

// test settings
uint8_t nChannels = 8;

// channel configuration
AP_RcChannel rcCh[] = 
{
	AP_RcChannel("ROLL",APM_RC,0,100.0),
	AP_RcChannel("PITCH",APM_RC,1,45),
	AP_RcChannel("THR",APM_RC,2,100),
	AP_RcChannel("YAW",APM_RC,3,45),
	AP_RcChannel("CH5",APM_RC,4,1),
	AP_RcChannel("CH6",APM_RC,5,1),
	AP_RcChannel("CH7",APM_RC,6,1),
	AP_RcChannel("CH8",APM_RC,7,1)
};

// test position
float testPosition = 0;
int8_t testSign = 1;

void setup()
{
	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");

	eepromRegistry.print(Serial); // show eeprom map
	APM_RC.Init();		// APM Radio initialization

	for (int i=0;i<nChannels;i++)
	{
		Serial.printf("ch:\t%d\tscale:\t%f\tcenter:\t%f\tpwmMin:\t%d\tpwmNeutral:\t%d\tpwmMax:\t%d\t",
			rcCh[i].getCh(),rcCh[i].getScale(),rcCh[i].getCenter(),
			rcCh[i].getPwmMin(),rcCh[i].getPwmNeutral(),rcCh[i].getPwmMax());
	}
}

void loop()	
{
	// set channel positions
	for (int i=0;i<nChannels;i++) rcCh[i].setNormalized(testPosition);
	Serial.printf("\ntestPosition (%f)\n\t\t",testPosition);
	for (int i=0;i<nChannels;i++) Serial.printf("%7s\t",rcCh[i].getName());
	Serial.println();
	Serial.printf("pwm      :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7d\t",rcCh[i].getPwm());
	Serial.println();
	Serial.printf("position :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7.2f\t",rcCh[i].getPosition());
	Serial.println();

	// update test value
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

	delay(500);
}
