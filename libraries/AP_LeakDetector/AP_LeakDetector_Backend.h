#pragma once

#include "AP_LeakDetector.h"

class AP_LeakDetector_Backend {
public:
    AP_LeakDetector_Backend(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state);

    // Each backend type must provide an implementation to read the sensor
    virtual void read(void) = 0;

protected:
    AP_LeakDetector &leak_detector;
    AP_LeakDetector::LeakDetector_State &state;
};
