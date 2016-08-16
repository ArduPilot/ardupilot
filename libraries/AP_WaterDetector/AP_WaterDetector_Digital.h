#include "AP_WaterDetector_Backend.h"

class AP_WaterDetector_Digital : public AP_WaterDetector_Backend
{
public:
	AP_WaterDetector_Digital(AP_WaterDetector &_water_detector, AP_WaterDetector::WaterDetector_State &_state);
	void read(void);

};
