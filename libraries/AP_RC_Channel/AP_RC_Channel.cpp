/*
	AP_RC_Channel.cpp - Radio library for Arduino Legacy Hardware
	Code by Jason Short. DIYDrones.com
	Improvements to implement channel curves by Ron Curry, 2012

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include <math.h>
#include <avr/eeprom.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "AP_RC_Channel.h"

#define ANGLE 0
#define RANGE 1

// setup the control preferences
void
AP_RC_Channel::set_range(int low, int high)
{
	_type 	= RANGE;
	_high 	= high;
	_low 	= low;
}

void
AP_RC_Channel::set_angle(int angle)
{
	_type 	= ANGLE;
	_high 	= angle;
}

void
AP_RC_Channel::set_reverse(bool reverse)
{
	if (reverse) _reverse = -1;
	else _reverse = 1;
}

bool
AP_RC_Channel::get_reverse(void)
{
	if (_reverse==-1) return 1;
	else return 0;
}

void
AP_RC_Channel::set_filter(bool filter)
{
	_filter = filter;
}

// call after first read
void
AP_RC_Channel::trim()
{
	radio_trim = radio_in;
}

//-------------------------------------------------------------------------------
// Support for PWM translation (i.e. curves or "expo")
//
//			Translation of the input PWM is done via a pointer "channel_curve" to an array that defines the PWM output value
//			for any given input value. The array is structured with element 0 equal to the number of elements 
//			in the curve. If the length is zero then the array defines no curve. If the "channel_curve" pointer 
//			is NULL that is interpretted as no curve defined and is the default state.
//
//			Elements 1 to n  of the array contain the values for the curve. These are defined in terms of the actual
//			PWM output pulsewidth desired for a given point on the curve with curve element 1 containing the value 
//			for the lowest input value from the RC RX and element "n" containing the value for the highest input value
//			from the RX.
//
//			Input PWM values are expected to be in the range of the radio calibration values "radio_min" to "radio_max". The
//			user must have already completed the radio calibration otherwise output will be inaccurage. Input PWM values
//			generate an index that falls between curve elements will cause the output to be interpolated in a linear fashion
//			between the curve elements. For example: A curve defined as element 0 = 2 (length), element 1 = 900, and 
//			element 2 = 2100 would define a linear straight line output between 900 and 2100 for valid input values. 
//			Additional elements could be inserted between element 1 and element 2 to define more complex
//			curves. - R. Curry 06-14-12



// Sets curve for channel output to user defined curve
// Input: curve - A pointer to a user defined output curve for this channel
void
AP_RC_Channel::set_channel_curve(int  *curve)
{
	_channel_curve = curve;					// Channel_curve points to array containing curve info
}

// Unsets the curve for this channel - i.e. no curve translation
void
AP_RC_Channel::unset_channel_curve()
{
	_channel_curve = NULL;
}


// Apply the current curve to a PWM value
// Input: PWM value in range of radio_min to radio_max
// Output: Translated PWM value
int
AP_RC_Channel::apply_curve(int pwm) 
{
	float scale;
	int	index1, index2;
	
	if (_channel_curve != NULL)
	{
		if (_channel_curve[0] > 0)										// If the length of the curve isn't zero then use it
		{		
			// Calculate the index into the channel curve table
			scale =  ((float)(pwm - radio_min) / 
					  (float)(radio_max - radio_min)) * 
						((float)_channel_curve[0]-1);	
			index1 = (int)scale;									// get the index
			scale -= (float)index1;									// scale now has the remainder for later
			
			if (index1 < 0) {										// If the PWM value below our range then clamp to lowest table entry
				index1 = 0;
				scale = 0.0;
			}
			
			index2 = index1 + 1;									// Point to the next entry beyond our current for interpolation
			if (index2 >= _channel_curve[0]) {							// If we are beyond the end then clamp to highest entry
				index2 = _channel_curve[0] - 1;
				if (index1 >= _channel_curve[0]) {						// Also check index 1 and clamp if necessary
					index1 = _channel_curve[0] -1;
				}
			}
			
			// Do the lookup and interpolation
			index1++;												// curve values start at entry 1
			index2++;
			pwm = ((_channel_curve[index1] * 
					(1 - scale)) + (_channel_curve[index2] * 
									scale));										// Get the pwm value from the curve and interpolate - done
		}
	}
	
	return pwm;																		// 
}


//-------------------------------------------------------------------------------


// read input from APM_RC - create a control_in value
void
AP_RC_Channel::set_pwm(int pwm)
{
	//	Serial.print(pwm,DEC);

	// Apply the curve - if any
	pwm = apply_curve(pwm);
	
	if(_filter){
		if(radio_in == 0)
			radio_in = pwm;
		else
			radio_in = ((pwm + radio_in) >> 1);		// Small filtering
	}else{
		radio_in = pwm;
	}

	if(_type == RANGE){
		//Serial.print("range ");
		control_in = pwm_to_range();
		control_in = (control_in < dead_zone) ? 0 : control_in;

	}else{
		control_in = pwm_to_angle();
		control_in = (abs(control_in) < dead_zone) ? 0 : control_in;

	}
}


int
AP_RC_Channel::control_mix(float value)
{
	return (1 - abs(control_in / _high)) * value + control_in;
}

// are we below a threshold?
bool
AP_RC_Channel::get_failsafe(void)
{
	return (radio_in < (radio_min - 50));
}

// returns just the PWM without the offset from radio_min
void
AP_RC_Channel::calc_pwm(void)
{

	if(_type == RANGE){
		pwm_out 	= range_to_pwm();
		radio_out 	= pwm_out + radio_min;
	}else{
		pwm_out 	= angle_to_pwm();
		radio_out 	= pwm_out + radio_trim;
	}
	//	radio_out = constrain(radio_out, radio_min, radio_max);
}

// ------------------------------------------

void
AP_RC_Channel::load_eeprom(void)
{
	radio_min 	= eeprom_read_word((uint16_t *)	_address);
	radio_max	= eeprom_read_word((uint16_t *)	(_address + 2));
	load_trim();
}

void
AP_RC_Channel::save_eeprom(void)
{
	eeprom_write_word((uint16_t *)	_address, 			radio_min);
	eeprom_write_word((uint16_t *)	(_address + 2), 	radio_max);
	save_trim();
}

// ------------------------------------------
void
AP_RC_Channel::save_trim(void)
{
	eeprom_write_word((uint16_t *)	(_address + 4), 	radio_trim);
	//_ee.write_int((_address + 4), 	radio_trim);
}

void
AP_RC_Channel::load_trim(void)
{
	radio_trim 	= eeprom_read_word((uint16_t *)	(_address + 4));
	//_ee.write_int((_address + 4), 	radio_trim);
}

// ------------------------------------------

void
AP_RC_Channel::zero_min_max()
{
	radio_min = radio_max = radio_in;
}

void
AP_RC_Channel::update_min_max()
{
	radio_min = min(radio_min, radio_in);
	radio_max = max(radio_max, radio_in);
}

// ------------------------------------------

int16_t
AP_RC_Channel::pwm_to_angle()
{
	if(radio_in < radio_trim)
		return _reverse * ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_trim - radio_min);
	else
		return _reverse * ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_max  - radio_trim);

		//return _reverse * _high * ((float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim));
		//return _reverse * _high * ((float)(radio_in - radio_trim) / (float)(radio_trim - radio_min));
}


int16_t
AP_RC_Channel::angle_to_pwm()
{
	if(_reverse == -1)
	{
		if(servo_out < 0)
			return ( -1 * ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high);
		else
			return ( -1 * ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high);
	} else {
		if(servo_out > 0)
			return ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high;
		else
			return ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high;
	}

		//return (((float)servo_out / (float)_high) * (float)(radio_max - radio_trim));
		//return (((float)servo_out / (float)_high) * (float)(radio_trim - radio_min));
}

// ------------------------------------------

int16_t
AP_RC_Channel::pwm_to_range()
{
	//return (_low + ((_high - _low) * ((float)(radio_in - radio_min) / (float)(radio_max - radio_min))));
	return (_low + ((long)(_high - _low) * (long)(radio_in - radio_min)) / (long)(radio_max - radio_min));
}

int16_t
AP_RC_Channel::range_to_pwm()
{
	//return (((float)(servo_out - _low) / (float)(_high - _low)) * (float)(radio_max - radio_min));
	return ((long)(servo_out - _low) * (long)(radio_max - radio_min)) / (long)(_high - _low);
}

// ------------------------------------------

float
AP_RC_Channel::norm_input()
{
	if(radio_in < radio_trim)
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
	else
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
}

float
AP_RC_Channel::norm_output()
{
	if(radio_out < radio_trim)
		return (float)(radio_out - radio_trim) / (float)(radio_trim - radio_min);
	else
		return (float)(radio_out - radio_trim) / (float)(radio_max  - radio_trim);
}
