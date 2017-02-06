#ifndef AP_LEAKDETECTOR_BACKEND_H
#define AP_LEAKDETECTOR_BACKEND_H

#include "AP_LeakDetector.h"

class AP_LeakDetector_Backend
{
public:
	AP_LeakDetector_Backend(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state);
	virtual void read(void) = 0;

protected:
	AP_LeakDetector &leak_detector;
	AP_LeakDetector::LeakDetector_State &state;

};

#endif
