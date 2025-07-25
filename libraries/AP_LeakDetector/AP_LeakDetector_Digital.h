#pragma once

#include "AP_LeakDetector_Backend.h"

#if AP_LEAKDETECTOR_ENABLED
class AP_LeakDetector_Digital : public AP_LeakDetector_Backend {
public:
    AP_LeakDetector_Digital(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state);
    void read(void) override;
};
#endif // AP_LEAKDETECTOR_ENABLED