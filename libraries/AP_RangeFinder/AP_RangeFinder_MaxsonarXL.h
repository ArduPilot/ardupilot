#ifndef AP_RangeFinder_MaxsonarXL_H
#define AP_RangeFinder_MaxsonarXL_H

#include "RangeFinder.h"

#define AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE 20
#define AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE 700

class AP_RangeFinder_MaxsonarXL : public RangeFinder
{
 // public:
	//AP_GPS_MTK(Stream *s);
  public:
	AP_RangeFinder_MaxsonarXL(AP_ADC *adc, ModeFilter *filter);

	int convert_raw_to_distance(int _raw_value) { return _raw_value; }   // read value from analog port and return distance in cm
};
#endif
