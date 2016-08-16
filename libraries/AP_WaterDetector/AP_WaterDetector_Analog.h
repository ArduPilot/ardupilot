#include "AP_WaterDetector_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_WaterDetector_Analog : public AP_WaterDetector_Backend
{
public:
	AP_WaterDetector_Analog(AP_WaterDetector &_water_detector, AP_WaterDetector::WaterDetector_State &_state);
	void read(void);

private:
	AP_HAL::AnalogSource *source;

};
