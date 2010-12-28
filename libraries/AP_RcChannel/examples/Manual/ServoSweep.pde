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

// test settings
uint8_t nChannels = 8;
bool loadFromEEProm = false;

// channel configuration
Vector< AP_EEPromVar<float> * > scale;
Vector< AP_EEPromVar<uint16_t> * > pwmMin;
Vector< AP_EEPromVar<uint16_t> * > pwmNeutral;
Vector< AP_EEPromVar<uint16_t> * > pwmMax;
Vector< AP_EEPromVar<uint16_t> * > pwmDeadZone;
Vector< AP_Var<bool> * > filter;
Vector< AP_Var<bool> * > reverse;
Vector< AP_RcChannel * > rc ;

// test position
float testPosition = 0;
uint16_t testPwm = 1500;
int8_t testSign = 1;

// serial
FastSerialPort0(Serial);

void setup()
{
	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");
	APM_RC.Init();		// APM Radio initialization

	// add channels
	for (int i=0;i<nChannels;i++)
	{
		char num[2];
		itoa(i,num,10);

		// initialize eeprom settings
		scale.push_back(new AP_EEPromVar<float>(1.0,strcat("SCALE",num)));
		pwmMin.push_back(new AP_EEPromVar<uint16_t>(1200,strcat("PWM_MIN",num)));
		pwmNeutral.push_back(new AP_EEPromVar<uint16_t>(1500,strcat("PWM_NEUTRAL",num)));
		pwmMax.push_back(new AP_EEPromVar<uint16_t>(1800,strcat("PWM_MAX",num)));
		pwmMax.push_back(new AP_EEPromVar<uint16_t>(10,strcat("PWM_DEADZONE",num)));

		// load
		if (loadFromEEProm)
		{
			scale[i]->load();
			pwmMin[i]->load();
			pwmNeutral[i]->load();
			pwmMax[i]->load();
			pwmDeadZone[i]->load();
			filter[i]->load();
			reverse[i]->load();
		}

		// add rc channel
		rc.push_back(new AP_RcChannel(APM_RC,i,scale[i]->get(),
			pwmMin[i]->get(),pwmNeutral[i]->get(),pwmMax[i]->get(),
			pwmDeadZone[i]->get(),filter[i]->get(),reverse[i]->get()));	

		// find neutral position
		rc[i]->readRadio();
		pwmNeutral[i]->set(rc[i]->getPwm());
	}
}

void loop()	
{
	// set channel positions
	for (int i=0;i<7;i++)
	{
		rc[i]->getRadio();
		Serial.printf("\npwm:\t%d\t\tposition:\t%f\n", rc[i]->getPwm(),rc[i]->getPosition());
	}
	delay(100);
}
