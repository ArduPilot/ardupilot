// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	Collision_Sensor.h
/// @brief	TODO.

#ifndef __COLLISION_SENSOR_H__
#define __COLLISION_SENSOR_H__

#include <AP_Common.h>

#define CA_NUMBER_OF_SENSORS 	6
#define CA_DEFAULT_DISTANCE		32767
#define CA_DEFAULT_THRESHOLD	150   //Collision threshold in cm, if we violate this
								      //border it is considered a collision

enum CA_SENSOR_POS{
	CA_BOTTOM = 0,
	CA_FRONT = 1,
	CA_RIGHT = 2,
	CA_LEFT = 3,
	CA_BACK = 4,
	CA_TOP = 5,
};

/// @class	Collision_Sensor
/// @brief	Abstract layer for any kind of collision sensor
class Collision_Sensor {
public:

	Collision_Sensor();

	//update hook for the sensors
	virtual void update() = 0;

	//test if a sensor is working and available, return true if everything is ok
	virtual bool test(CA_SENSOR_POS pos) = 0;

	//return maximum distance the sensor is able to measure
	virtual int16_t get_max_distance(CA_SENSOR_POS pos);

	//return minimum distance the sensor is able to measure
	virtual int16_t get_min_distance(CA_SENSOR_POS pos);

	//get the depth of a collision, a negative value indicates a collision is occurred
	int16_t get_collision_depth(CA_SENSOR_POS pos);

	//get the distance measured by a collision sensor in cm
	int16_t get_collision_distance(CA_SENSOR_POS pos);

	//activate a collision sensor, returns true if the activation was successful
	bool   set_activated(CA_SENSOR_POS pos, bool activated);

	//check if a sensor is activated
	bool   is_activated(CA_SENSOR_POS pos);

	//change sensor threshold
	void   set_threshold(CA_SENSOR_POS pos, int16_t new_threshold);

protected:

	//needs to be called by the child classes, once the sensor measurement is updated
	void updateDistanceMeasurement(CA_SENSOR_POS pos, int16_t distance);

	int16_t collision_distance[CA_NUMBER_OF_SENSORS];

	int16_t collision_threshold[CA_NUMBER_OF_SENSORS];

	int16_t collision_depth[CA_NUMBER_OF_SENSORS];

	bool   sensor_activated[CA_NUMBER_OF_SENSORS];

};

#endif // __COLLISION_SENSOR_H__
