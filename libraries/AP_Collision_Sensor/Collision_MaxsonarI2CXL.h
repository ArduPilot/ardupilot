// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-

#ifndef __COLLISION_MAXSONARI2CXL_H__
#define __COLLISION_MAXSONARI2CXL_H__

#include "AP_RangeFinder.h"
#include "Collision_Sensor.h"

enum CA_SENSOR_ADDRESS{
	CA_ADD_BOTTOM 	= 0x70,
	CA_ADD_FRONT 	= 0x71,
	CA_ADD_RIGHT 	= 0x72,
	CA_ADD_LEFT 	= 0x74,
	CA_ADD_BACK 	= 0x73,
	CA_ADD_TOP 		= 0x75,
};

class Collision_MaxsonarI2CXL : public Collision_Sensor
{

public:

    // constructor
	Collision_MaxsonarI2CXL();

	void update();

	int16_t get_max_distance(CA_SENSOR_POS pos);

	int16_t get_min_distance(CA_SENSOR_POS pos);

	bool test(CA_SENSOR_POS pos);

private:
	CA_SENSOR_POS current_sensor;

	FilterInt16* filter[CA_NUMBER_OF_SENSORS];

	AP_RangeFinder_MaxsonarI2CXL* sensor[CA_NUMBER_OF_SENSORS];

	bool is_reading_requested[CA_NUMBER_OF_SENSORS] =  { false };

};
#endif  // __COLLISION_MAXSONARI2CXL_H__
