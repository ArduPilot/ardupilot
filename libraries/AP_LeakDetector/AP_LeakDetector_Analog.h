#ifndef AP_LEAKDETECTOR_ANALOG_H
#define AP_LEAKDETECTOR_ANALOG_H

#include <AP_HAL/AP_HAL.h>
#include "AP_LeakDetector_Backend.h"

class AP_LeakDetector_Analog : public AP_LeakDetector_Backend
{
public:
	AP_LeakDetector_Analog(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state);
	void read(void);

private:
	AP_HAL::AnalogSource *source;

};

#endif
