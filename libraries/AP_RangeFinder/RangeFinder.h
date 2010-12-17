#ifndef RangeFinder_h
#define RangeFinder_h

#include <inttypes.h>

//#define AP_RANGEFINDER_FILTER_NONE 0
//#define AP_RANGEFINDER_FILTER_LIMITED_CHANGE 1

class RangeFinder
{
  protected:
    int _analogPort;  // the port to which the sensor is connected
	int _filterType;
  public:
	int raw_value;  // raw value from the sensor
	int distance; // distance in cm
	int max_distance;  // maximum measurable distance (in cm)
	int min_distance;  // minimum measurable distance (in cm)
	
	virtual void init(int analogPort);
	//virtual void set_filter(int filterType) { _filterType = filterType; };
	virtual int read();   // read value from analog port and return distance in cm
};
#endif
