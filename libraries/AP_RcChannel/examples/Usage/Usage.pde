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

FastSerialPort0(Serial); // make sure this proceed variable declarations 

AP_EEPromVar<float> scale(45.0,"RC1_SCALE");
AP_EEPromVar<uint16_t> pwmMin(1000,"RC1_PWMMIN");
AP_EEPromVar<uint16_t> pwmNeutral(1500,"RC1_PWMNEUTRAL");
AP_EEPromVar<uint16_t> pwmMax(2000,"RC1_PWMMAX");
AP_EEPromVar<uint16_t> pwmDeadZone(100,"RC1_PWMDEADZONE");

#define CH_1 0

// configuration
AP_Var<bool> filter(false,"RC1_FILTER");
AP_Var<bool> reverse(false,"RC1_REVERSE");

float testPosition = 0;
uint16_t testPwm = 1500;

AP_RcChannel rc[] = 
{
	AP_RcChannel(APM_RC,CH_1,scale.get(),pwmMin.get(),pwmNeutral.get(),pwmMax.get(),
			pwmDeadZone.get(),filter.get(),reverse.get())	

};

void setup()
{
	Serial.begin(115200);
	Serial.println("ArduPilot RC Channel test");
	APM_RC.Init();		// APM Radio initialization

	delay(1000);

	// configuratoin
	scale.set(100);
	Serial.printf("\nscale.set(100)\n");
	delay(2000);

	// find neutral radio position
	rc[CH_1].readRadio();
	Serial.printf("\nrc[CH_1].readRadio()\n");
	Serial.printf("\npwmNeutral.set(rc[CH_1].getPwm())\n");
	pwmNeutral.set(rc[CH_1].getPwm());
	delay(2000);
}

void loop()	
{
	// get the min pwm
	Serial.printf("\npwmMin.get(): %d\n", pwmMin.get());
	delay(2000);

	// set by pwm 
	rc[CH_1].setPwm(testPwm);
	Serial.printf("\nrc[CH_1].setPwm(1500)\n");
	Serial.printf("pwm: %d position: %f\n",rc[CH_1].getPwm(),
			rc[CH_1].getPosition());
	delay(2000);

	// set by position
	rc[CH_1].setPosition(testPosition);
	Serial.printf("\nrc[CH_1].setPosition(-50))\n");
	Serial.printf("pwm: %d position: %f\n",rc[CH_1].getPwm(),
			rc[CH_1].getPosition());
	delay(2000);

	// update test value
	testPosition += 10;
	if (testPosition > 100) testPosition = -100;
	testPwm += 100;
	if (testPwm > 1800) testPosition = 1200;
}
