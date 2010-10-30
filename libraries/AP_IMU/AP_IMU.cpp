/*
	APM_IMU.cpp - IMU Sensor Library for Ardupilot Mega
		Code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com

	This library works with the ArduPilot Mega and "Oilpan"
	
	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

        Methods:
                quick_init()	: For air restart
                init() 			: For ground start.  Calibration
				get_gyro()		: Returns gyro vector.  Elements in radians/second
				get_accel()		: Returns acceleration vector.  Elements in meters/seconds squared

*/

#include <AP_IMU.h>

#define A_LED_PIN 37			//37 = A,	35 = C
#define C_LED_PIN 35

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 418
#define GRAVITY 418 //this equivalent to 1G in the raw data coming from the accelerometer 
#define accel_scale(x) (x*9.80665/GRAVITY)//Scaling the raw data of the accel to actual acceleration in meters per second squared

#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

// IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
#define _gyro_gain_x 0.4 //X axis Gyro gain
#define _gyro_gain_y 0.41 //Y axis Gyro gain
#define _gyro_gain_z 0.41 //Z axis Gyro 

#define ADC_CONSTRAINT 900

// Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_IMU::_sensors[6]       = {1,2,0,4,5,6};	// For ArduPilot Mega Sensor Shield Hardware
const int     AP_IMU::_sensor_signs[]	= {	 1, -1, -1,
											 1, -1, -1};	

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
const float   AP_IMU::_gyro_temp_curve[3][3] = {
	{1665,0,0},
	{1665,0,0},
	{1665,0,0}
};	// To Do - make additional constructors to pass this in.



// Constructors ////////////////////////////////////////////////////////////////

AP_IMU::AP_IMU(void)
{
}


/**************************************************/
void
AP_IMU::quick_init(void)
{
	
//  NOTE ***  Need to addd code to retrieve values from EEPROM
}


/**************************************************/
void
AP_IMU::init(void)
{
	
	float temp;
	int flashcount = 0;
	int tc_temp = APM_ADC.Ch(_gyro_temp_ch);
 	delay(500);
 	
	for(int c = 0; c < 200; c++)
	{ 
		digitalWrite(A_LED_PIN, LOW);
		digitalWrite(C_LED_PIN, HIGH);
		delay(20);
		for (int i = 0; i < 6; i++) 
			_adc_in[i] = APM_ADC.Ch(_sensors[i]);
		digitalWrite(C_LED_PIN, LOW);
		digitalWrite(A_LED_PIN, HIGH);
		delay(20);
	}

	for(int i = 0; i < 200; i++){		// We take some readings...
		for (int j = 0; j < 6; j++) {
			_adc_in[j] = APM_ADC.Ch(_sensors[j]);
			if (j < 3) {	
				_adc_in[j] -= _gyro_temp_comp(j, tc_temp);		// Subtract temp compensated typical gyro bias
			} else {
				_adc_in[j] -= 2025;
			}
				
			_adc_offset[j] = _adc_offset[j] * 0.9 + _adc_in[j] * 0.1;

		}

		delay(20);
		if(flashcount == 5) {
		digitalWrite(A_LED_PIN, LOW);
		digitalWrite(C_LED_PIN, HIGH);
		}
		if(flashcount >= 10) {
			flashcount = 0;
		digitalWrite(C_LED_PIN, LOW);
		digitalWrite(A_LED_PIN, HIGH);
		}
		flashcount++;
	}
	
	_adc_offset[5] += GRAVITY * _sensor_signs[5];
	
//  NOTE ***  Need to addd code to save values to EEPROM
}


/**************************************************/
// Returns the temperature compensated raw gyro value
//---------------------------------------------------
float
AP_IMU::_gyro_temp_comp(int i, int temp) const
{
	// We use a 2nd order curve of the form Gtc = A + B * Graw + C * (Graw)**2
	//------------------------------------------------------------------------
	return _gyro_temp_curve[i][0] + _gyro_temp_curve[i][1] * temp + _gyro_temp_curve[i][2] * temp * temp;	
}
/**************************************************/
Vector3f
AP_IMU::get_gyro(void)
{	
	int tc_temp = APM_ADC.Ch(_gyro_temp_ch);
	
	for (int i = 0; i < 3; i++) {
		_adc_in[i] = APM_ADC.Ch(_sensors[i]);
		_adc_in[i] -= _gyro_temp_comp(i,tc_temp);		// Subtract temp compensated typical gyro bias
		if (_sensor_signs[i] < 0)
			_adc_in[i] = (_adc_offset[i] - _adc_in[i]);
		else
			_adc_in[i] = (_adc_in[i] - _adc_offset[i]);
			
		if (abs(_adc_in[i]) > ADC_CONSTRAINT) {
			adc_constraints++; 														// We keep track of the number of times		
			_adc_in[i] = constrain(_adc_in[i], -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
		}
	}

	_gyro_vector.x = ToRad(_gyro_gain_x) * _adc_in[0];
	_gyro_vector.y = ToRad(_gyro_gain_y) * _adc_in[1];
	_gyro_vector.z = ToRad(_gyro_gain_z) * _adc_in[2];
	
	return _gyro_vector;
}

/**************************************************/
Vector3f
AP_IMU::get_accel(void)
{	
	for (int i = 3; i < 6; i++) {
		_adc_in[i] = APM_ADC.Ch(_sensors[i]);
		_adc_in[i] -= 2025;								// Subtract typical accel bias
		if (_sensor_signs[i] < 0)
			_adc_in[i] = (_adc_offset[i] - _adc_in[i]);
		else
			_adc_in[i] = (_adc_in[i] - _adc_offset[i]);		
		if (abs(_adc_in[i]) > ADC_CONSTRAINT) {
			adc_constraints++; 												// We keep track of the number of times		
			_adc_in[i] = constrain(_adc_in[i], -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
		}
	}
	
	_accel_vector.x = accel_scale(_adc_in[3]);
	_accel_vector.y = accel_scale(_adc_in[4]);
	_accel_vector.z = accel_scale(_adc_in[5]);	
	
	return _accel_vector;
}

