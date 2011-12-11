#ifndef RangeFinder_h
#define RangeFinder_h

#include <stdlib.h>
#include <inttypes.h>
#include "../AP_AnalogSource/AP_AnalogSource.h"
#include "../ModeFilter/ModeFilter.h" // ArduPilot Mega RC Library

/*
#define AP_RANGEFINDER_ORIENTATION_FRONT		  0, 10,  0
#define AP_RANGEFINDER_ORIENTATION_RIGHT		-10,  0,  0
#define AP_RANGEFINDER_ORIENTATION_BACK			  0,-10,  0
#define AP_RANGEFINDER_ORIENTATION_LEFT			 10,  0,  0
#define AP_RANGEFINDER_ORIENTATION_UP			  0,  0,-10
#define AP_RANGEFINDER_ORIENTATION_DOWN			  0,  0, 10
#define AP_RANGEFINDER_ORIENTATION_FRONT_RIGHT 	 -5, -5,  0
#define AP_RANGEFINDER_ORIENTATION_BACK_RIGHT 	 -5, -5,  0
#define AP_RANGEFINDER_ORIENTATION_BACK_LEFT 	  5, -5,  0
#define AP_RANGEFINDER_ORIENTATION_FRONT_LEFT 	  5,  5,  0
*/

class RangeFinder
{
  protected:
	RangeFinder(AP_AnalogSource * source, ModeFilter *filter) :
		_analog_source(source),
		_mode_filter(filter)
	{}
  public:

	int raw_value;     // raw value from the sensor
	int distance;      // distance in cm
	int max_distance;  // maximum measurable distance (in cm) - should be set in child's constructor
	int min_distance;  // minimum measurable distance (in cm) - should be set in child's constructor
	int orientation_x, orientation_y, orientation_z;

    virtual void set_orientation(int x, int y, int z);
	virtual int convert_raw_to_distance(int _raw_value) { return _raw_value; }  // function that each child class should override to convert voltage to distance
	virtual int read();   // read value from sensor and return distance in cm

	AP_AnalogSource *_analog_source;
  	ModeFilter  *_mode_filter;
};
#endif
