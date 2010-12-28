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
bool loadEEProm = false;
bool saveEEProm = false;

// channel configuration
Vector< AP_EEPromVar<float> * > scale;
Vector< AP_EEPromVar<uint16_t> * > pwmMin;
Vector< AP_EEPromVar<uint16_t> * > pwmNeutral;
Vector< AP_EEPromVar<uint16_t> * > pwmMax;
Vector< AP_EEPromVar<uint16_t> * > pwmDeadZone;
Vector< AP_Var<bool> * > filter;
Vector< AP_Var<bool> * > reverse;
Vector< AP_RcChannel * > rc;

void setup()
{
	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");
	APM_RC.Init();		// APM Radio initialization

	// add channels
	for (int i=0;i<nChannels;i++)
	{
		char num[5];
		itoa(i+1,num,10);

		Serial.printf("\nInitializing channel %d", i+1);

		// initialize eeprom settings
		scale.push_back(new AP_EEPromVar<float>(1.0,strcat("SCALE",num)));
		pwmMin.push_back(new AP_EEPromVar<uint16_t>(1200,strcat("PWM_MIN",num)));
		pwmNeutral.push_back(new AP_EEPromVar<uint16_t>(1500,strcat("PWM_NEUTRAL",num)));
		pwmMax.push_back(new AP_EEPromVar<uint16_t>(1800,strcat("PWM_MAX",num)));
		pwmDeadZone.push_back(new AP_EEPromVar<uint16_t>(10,strcat("PWM_DEADZONE",num)));
		filter.push_back(new AP_EEPromVar<bool>(false,strcat("FILTER",num)));
		reverse.push_back(new AP_EEPromVar<bool>(false,strcat("REVERSE",num)));

		// save
		if (saveEEProm)
		{
			scale[i]->save();
			pwmMin[i]->save();
			pwmNeutral[i]->save();
			pwmMax[i]->save();
			pwmDeadZone[i]->save();
			filter[i]->save();
			reverse[i]->save();
		}

		// load
		if (loadEEProm)
		{
			scale[i]->load();
			pwmMin[i]->load();
			pwmNeutral[i]->load();
			pwmMax[i]->load();
			pwmDeadZone[i]->load();
			filter[i]->load();
			reverse[i]->load();
		}

		// find neutral position
		AP_RcChannel * ch = new AP_RcChannel(APM_RC,i,scale[i]->get(),
			pwmMin[i]->get(),pwmNeutral[i]->get(),pwmMax[i]->get(),
			pwmDeadZone[i]->get(),filter[i]->get(),reverse[i]->get());

		ch->readRadio();
		pwmNeutral[i]->set(ch->getPwm());

		// add rc channel
		rc.push_back(ch);	
	}
}

void loop()	
{
	// read radio
	for (int i=0;i<nChannels;i++) rc[i]->readRadio();
	
	// read test positions 
	Serial.printf("\npwm      :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7d\t",rc[i]->getPwm());
	Serial.println();
	Serial.printf("position :\t");
	for (int i=0;i<nChannels;i++) Serial.printf("%7.2f\t",rc[i]->getPosition());
	Serial.println();
	delay(100);
}
