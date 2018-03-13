#pragma once

#include "AP_LeakDetector_Backend.h"

class AP_LeakDetector_Digital : public AP_LeakDetector_Backend {
public:
    AP_LeakDetector_Digital(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state);
    void read(void);
};
