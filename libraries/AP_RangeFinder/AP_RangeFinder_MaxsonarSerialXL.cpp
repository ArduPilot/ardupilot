// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       AP_RangeFinder_MaxsonarSerialXL.cpp - Arduino Library for MaxSonar XL Serial connection
 *       Code by Charles Werbick. Based on HAL schema code by Randy Mackay. DIYDrones.com
 *
 *		NOTE- this module CANNOT be hooked directly to UART. It's output must be inverted to connect to TTL.
 *			make (or google) logic inverter. All you need is a transistor and two resistors. ('Bout the price of 
 *			a cup of tasty java.)
 *
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Sparkfun URL: http://www.sparkfun.com/products/9491
 *       datasheet: http://www.sparkfun.com/datasheets/Sensors/Proximity/XL-EZ0-Datasheet.pdf
 *
 *       Sensor should be connected to one of the analog ports
 *
 *       Variables:
 *               int raw_value : raw value from the sensor (distance in inches for serial)
 *               int distance : distance in cm
 *               int max_distance : maximum measurable distance (in cm)
 *               int min_distance : minimum measurable distance (in cm)
 *
 *       Methods:
 *               read() : read value from analog port and returns the distance (in cm)
 *
 */

#include "AP_RangeFinder_MaxsonarSerialXL.h"

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_MaxsonarSerialXL::AP_RangeFinder_MaxsonarSerialXL(AP_HAL::UARTDriver *ser, FilterInt16 *filter) :
    RangeFinder(NULL, filter),
	_ser(ser),
    _scaler(AP_RANGEFINDER_MAXSONARSERXL_SCALER)
{
    max_distance = AP_RANGEFINDER_MAXSONARSERXL_MAX_DISTANCE;
    min_distance = AP_RANGEFINDER_MAXSONARSERXL_MIN_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_RangeFinder_MaxsonarSerialXL::init(AP_HAL::UARTDriver *ser)
{
	_ser = ser;
	_ser->flush();
	max_distance = AP_RANGEFINDER_MAXSONARSERXL_MAX_DISTANCE;
    min_distance = AP_RANGEFINDER_MAXSONARSERXL_MIN_DISTANCE;
}

int AP_RangeFinder_MaxsonarSerialXL::read()
{

	    uint8_t _step = 0;
	    uint8_t data;
		int16_t numc;

		
		numc = _ser->available();
		if(numc < 4)
			//return -1; //not enough data
			return -1;
    	char _array_dist[4]; 

		for (int16_t i = 0; i < numc; i++)        // Process bytes received
	    {

			data = _ser->read();
			switch(_step){
			case 0:
				if(AP_RANGEFINDER_MAXSONARSERXL_STARTCHAR == data)
				{
					_step++; //next 3 chars will be distance in inches
				}
				break;
			case 1:
				_array_dist[0] = data;
				_step++;
				break;
			case 2:
				_array_dist[1] = data;
				_step++;
				break;
			case 3:
				_array_dist[2] = data;
				_step++;
				break;
			case 4:
				if(AP_RANGEFINDER_MAXSONARSERXL_STARTCHAR == data)
				{
					//success! got a whole line
					_step = 0; //reset
					_array_dist[3] = 0x0; //terminate the array
					raw_value = atoi(_array_dist);
					//noise seems to create spurious large values 
					_ser->flush();
				}
				break;
			}
//			if(_old + 100 < raw_value)
//			{
				//noise reduction. Ignore single spurious reading
				//if only one outlier, discard. If persists for 2 reads, keep...
//				_old = raw_value;
//			}
		}

	    if(raw_value > 0)
		{
			//int temp_dist = raw_value * _scaler;
			// ensure distance is within min and max
			int temp_dist = constrain_float(raw_value, min_distance, max_distance);

			distance = _mode_filter->apply(temp_dist);
			//distance = temp_dist;
		    //	_ser->flush();
		}

		return distance;






}