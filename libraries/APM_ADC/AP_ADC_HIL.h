#ifndef AP_ADC_HIL_H
#define AP_ADC_HIL_H

/*
	AP_ADC_HIL.cpp - HIL model of ADC ADS7844 for Ardupilot Mega
	Code by James Goppert. DIYDrones.com
*/

#include <inttypes.h>
#include "AP_ADC.h"

class AP_ADC_HIL : public AP_ADC
{
  public:
	AP_ADC_HIL();  // Constructor
	void Init();
	int Ch(unsigned char ch_num);     
	int setHIL(int16_t p, int16_t q, int16_t r, int16_t gyroTemp,
    	int16_t aX, int16_t aY, int16_t aZ, int16_t diffPress);

  private:
 	static const uint16_t adcPerG = 418;
    static const float gyroGainX = 0.4;
    static const float gyroGainY = 0.41;
    static const float gyroGainZ = 0.41;
    static const float deg2rad = 3.14159/180.0;
    static const uint8_t sensors[6];		
    static const int sensorSign[9];
	long adc_value[8];
};

#endif
