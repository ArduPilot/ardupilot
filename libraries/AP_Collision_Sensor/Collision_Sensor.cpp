// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	Collision_Sensor.cpp
/// @brief	TODO

#include "Collision_Sensor.h"

//constructor
Collision_Sensor::Collision_Sensor():
	collision_distance {CA_DEFAULT_DISTANCE, CA_DEFAULT_DISTANCE,CA_DEFAULT_DISTANCE,
						CA_DEFAULT_DISTANCE, CA_DEFAULT_DISTANCE, CA_DEFAULT_DISTANCE},
	collision_threshold {CA_DEFAULT_THRESHOLD, CA_DEFAULT_THRESHOLD, CA_DEFAULT_THRESHOLD,
						 CA_DEFAULT_THRESHOLD, CA_DEFAULT_THRESHOLD, CA_DEFAULT_THRESHOLD},
	collision_depth{0,0,0,0,0,0},
	sensor_activated{false, false, false, false, false, false}{

}

//methods

int16_t Collision_Sensor::get_collision_depth(CA_SENSOR_POS pos){
	return collision_depth[pos];
}

int16_t Collision_Sensor::get_collision_distance(CA_SENSOR_POS pos){
	return collision_distance[pos];
}

bool   Collision_Sensor::set_activated(CA_SENSOR_POS pos, bool activated){
	if(activated)
	{
		//check if the sensor is working and attached
		//TODO check
		if(!test(pos)){
			return false;
		}
	}
	else{
		collision_distance[pos] = CA_DEFAULT_DISTANCE;
		collision_depth[pos] = 0;
	}
	sensor_activated[pos] = activated;

	return true;
}

bool   Collision_Sensor::is_activated(CA_SENSOR_POS pos){
	return sensor_activated[pos];
}

int16_t Collision_Sensor::get_max_distance(CA_SENSOR_POS pos){
	return CA_DEFAULT_DISTANCE;
}

int16_t Collision_Sensor::get_min_distance(CA_SENSOR_POS pos){
	return 0;
}

void Collision_Sensor::updateDistanceMeasurement(CA_SENSOR_POS pos, int16_t distance){
	collision_distance[pos] = distance;
	collision_depth[pos] = distance - collision_threshold[pos];
}

void Collision_Sensor::set_threshold(CA_SENSOR_POS pos, int16_t new_threshold){
	collision_threshold[pos] = new_threshold;
}
