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
uint8_t nChannels = 8;

// channel configuration
AP_RcChannel rcCh[] = 
{
	AP_RcChannel("ROLL",APM_RC,0,45),
	AP_RcChannel("PITCH",APM_RC,1,45),
	AP_RcChannel("THR",APM_RC,2,100),
	AP_RcChannel("YAW",APM_RC,3,45),
	AP_RcChannel("CH5",APM_RC,4,1),
	AP_RcChannel("CH6",APM_RC,5,1),
	AP_RcChannel("CH7",APM_RC,6,1),
	AP_RcChannel("CH8",APM_RC,7,1)
};

// test position
float testPosition = 2;
int8_t testSign = 1;

void setup()
{
	Serial.begin(115200);
	delay(2000);

	Serial.println("ArduPilot RC Channel test");

	APM_RC.Init();		// APM Radio initialization

	delay(2000);
	eepromRegistry.print(Serial); // show eeprom map
}

void loop()	
{
	// read the radio 
	for (int i=0;i<nChannels;i++) rcCh[i].readRadio();

	// print channel names
	Serial.print("\n\t\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7s\t",rcCh[i].getName());
	Serial.println();

	// print pwm
	Serial.printf("pwm      :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7d\t",rcCh[i].getPwm());
	Serial.println();

	// print position
	Serial.printf("position :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7.2f\t",rcCh[i].getPosition());
	Serial.println();

	delay(500);
}
