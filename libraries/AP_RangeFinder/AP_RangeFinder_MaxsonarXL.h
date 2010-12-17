#ifndef AP_RangeFinder_MaxsonarXL_H
#define AP_RangeFinder_MaxsonarXL_H

#include "RangeFinder.h"

#define AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE 20
#define AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE 700

class AP_RangeFinder_MaxsonarXL : public RangeFinder
{	
  public:
	void init(int analogPort);
	int read();   // read value from analog port and return distance in cm
};
#endif
