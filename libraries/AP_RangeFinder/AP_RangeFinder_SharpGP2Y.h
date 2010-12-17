#ifndef AP_RangeFinder_SharpGP2Y_H
#define AP_RangeFinder_SharpGP2Y_H

#include "RangeFinder.h"

#define AP_RANGEFINDER_SHARPEGP2Y_MIN_DISTANCE 20
#define AP_RANGEFINDER_SHARPEGP2Y_MAX_DISTANCE 150

class AP_RangeFinder_SharpGP2Y : public RangeFinder
{	
  public:
	void init(int analogPort);
	int read();   // read value from analog port and return distance in cm
};
#endif
