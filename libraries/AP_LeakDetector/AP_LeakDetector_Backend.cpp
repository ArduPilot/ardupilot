#include "AP_LeakDetector_config.h"

#if AP_LEAKDETECTOR_ENABLED

#include "AP_LeakDetector_Backend.h"

AP_LeakDetector_Backend::AP_LeakDetector_Backend(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state) :
    leak_detector(_leak_detector),
    state(_state)
{}

#endif // AP_LEAKDETECTOR_ENABLED
