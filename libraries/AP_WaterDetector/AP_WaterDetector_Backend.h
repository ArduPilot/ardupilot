#ifndef AP_WATERDETECTOR_BACKEND_H
#define AP_WATERDETECTOR_BACKEND_H


#include "AP_WaterDetector.h"

class AP_WaterDetector_Backend
{
public:
	AP_WaterDetector_Backend(AP_WaterDetector &_water_detector, AP_WaterDetector::WaterDetector_State &_state);
	virtual void read(void) = 0;

protected:
	AP_WaterDetector &water_detector;
	AP_WaterDetector::WaterDetector_State &state;

};
#endif
