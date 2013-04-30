// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
 *       AP_RangeFinder_MaxsonarXL.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
 *       infrared proximity sensor
 *       Code by Jose Julio and Randy Mackay. DIYDrones.com
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
 *               int raw_value : raw value from the sensor
 *               int distance : distance in cm
 *               int max_distance : maximum measurable distance (in cm)
 *               int min_distance : minimum measurable distance (in cm)
 *
 *       Methods:
 *               read() : read value from analog port and returns the distance (in cm)
 *
 */

#include "Collision_MaxsonarI2CXL.h"

// Constructor //////////////////////////////////////////////////////////////

Collision_MaxsonarI2CXL::Collision_MaxsonarI2CXL():Collision_Sensor()
{
	for(uint8_t i = 0; i < CA_NUMBER_OF_SENSORS; i++){
		ModeFilterInt16_Size3* sonar_mode_filter = new ModeFilterInt16_Size3(1);
		filter[i] = sonar_mode_filter;
		sensor[i] = new AP_RangeFinder_MaxsonarI2CXL(sonar_mode_filter);
	}

	sensor[CA_BOTTOM]->init(CA_ADD_BOTTOM);
	sensor[CA_FRONT]->init(CA_ADD_FRONT);
	sensor[CA_RIGHT]->init(CA_ADD_RIGHT);
	sensor[CA_LEFT]->init(CA_ADD_LEFT);
	sensor[CA_BACK]->init(CA_ADD_BACK);
	sensor[CA_TOP]->init(CA_ADD_TOP);

	current_sensor = CA_BOTTOM;
}

void Collision_MaxsonarI2CXL::update(){
	//we need to iterate in a maximum through all available sensors
	for(uint8_t i = 0; i < CA_NUMBER_OF_SENSORS; i++){
		//check if sensor is activated
		if(sensor_activated[current_sensor]){
			AP_RangeFinder_MaxsonarI2CXL* sonar = sensor[current_sensor];
			if(is_reading_requested[current_sensor]){
				int16_t distance = sonar->read();
				if(!sonar->healthy){
					break; //try again
				}else{
					updateDistanceMeasurement(current_sensor, distance);
					is_reading_requested[current_sensor] = false;
				}
			}else{
				sonar->take_reading();
				is_reading_requested[current_sensor] = true;
				break;
			}
		}
		uint8_t tmp_sensor = (uint8_t)current_sensor;
		current_sensor = (CA_SENSOR_POS)((++tmp_sensor)%CA_NUMBER_OF_SENSORS);
	}
}

int16_t Collision_MaxsonarI2CXL::get_max_distance(CA_SENSOR_POS pos){
	return sensor[pos]->max_distance;
}

int16_t Collision_MaxsonarI2CXL::get_min_distance(CA_SENSOR_POS pos){
	return sensor[pos]->min_distance;
}

bool Collision_MaxsonarI2CXL::test(CA_SENSOR_POS pos){
	return sensor[pos]->test();
}
