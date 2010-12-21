#ifndef RangeFinder_h
#define RangeFinder_h

#include <inttypes.h>

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

#define AP_RANGEFINDER_NUM_AVERAGES 4

class RangeFinder
{
  public:
    int _analogPort;  // the port to which the sensor is connected
	int _history_ptr;
	int _history[AP_RANGEFINDER_NUM_AVERAGES];
  public:
	int raw_value;  // raw value from the sensor
	int distance; // distance in cm
	int max_distance;  // maximum measurable distance (in cm)
	int min_distance;  // minimum measurable distance (in cm)
	int orientation_x, orientation_y, orientation_z;

	int filter(int latestValue);  // returns the average of the last AP_RANGEFINDER_NUM_AVERAGES values
	
	virtual void init(int analogPort);
    virtual void set_orientation(int x, int y, int z);
	virtual int read();   // read value from analog port and return distance in cm
};
#endif
