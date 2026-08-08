#pragma once

#include "AP_LeakDetector_config.h"

#if AP_LEAKDETECTOR_ANALOG_ENABLED

#include "AP_LeakDetector_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_LeakDetector_Analog : public AP_LeakDetector_Backend {
public:
    AP_LeakDetector_Analog(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state);
    void read(void) override;

private:
    AP_HAL::AnalogSource *source;
};
#endif // AP_LEAKDETECTOR_ANALOG_ENABLED
