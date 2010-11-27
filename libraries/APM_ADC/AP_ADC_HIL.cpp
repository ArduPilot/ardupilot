#include "AP_ADC_HIL.h"

const uint8_t AP_ADC_HIL::sensors[6] = {1,2,0,4,5,6};		
const int AP_ADC_HIL::sensorSign[9]	= { 1, -1, -1,
							-1,  1,  1,
							-1, -1, -1};	
AP_ADC_HIL::AP_ADC_HIL() : adc_value()
{
}

void AP_ADC_HIL::Init(void)
{
}

// Read one channel value
int AP_ADC_HIL::Ch(unsigned char ch_num)         
{
  return adc_value[ch_num];
}

// Set one channel value
int AP_ADC_HIL::setHIL(int16_t p, int16_t q, int16_t r, int16_t gyroTemp,
    int16_t aX, int16_t aY, int16_t aZ, int16_t diffPress)
{
       // TODO: map temp and press to raw
    
    // gyros
    /* 0 */ adc_value[sensors[0]] = sensorSign[0]* p/(gyroGainX*deg2rad*1000) + 1665; // note apm says 1,2,0 gyro order, but 
    /* 1 */ adc_value[sensors[1]] = sensorSign[1]* q/(gyroGainY*deg2rad*1000) + 1665; // this says 0,1,2
    /* 2 */ adc_value[sensors[2]] = sensorSign[2]* r/(gyroGainZ*deg2rad*1000) + 1665;

    // gyro temp
    /* 3 */ adc_value[3] = sensorSign[3]* gyroTemp; //gyroTemp XXX: fix scaling; 

    // accelerometers
    /* 4 */ adc_value[sensors[3]] = sensorSign[4]* (aX*adcPerG)/1.0e3 + 2025;
    /* 5 */ adc_value[sensors[4]] = sensorSign[5]* (aY*adcPerG)/1.0e3 + 2025;
    /* 6 */ adc_value[sensors[5]] = sensorSign[6]* (aZ*adcPerG)/1.0e3 + 2025;

    // differential pressure
    /* 7 */ adc_value[7] = sensorSign[7]* diffPress; //diffPress XXX: fix scaling;
}
