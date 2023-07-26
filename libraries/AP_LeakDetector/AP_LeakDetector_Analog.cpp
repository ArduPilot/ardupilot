#include "AP_LeakDetector_Analog.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_LeakDetector_Analog::AP_LeakDetector_Analog(AP_LeakDetector &_leak_detector, AP_LeakDetector::LeakDetector_State &_state) :
    AP_LeakDetector_Backend(_leak_detector, _state)
{
    source = hal.analogin->channel(leak_detector._pin[state.instance]);
}

void AP_LeakDetector_Analog::read()
{
    if (source != NULL && leak_detector._pin[state.instance] >= 0 && source->set_pin(leak_detector._pin[state.instance])) {
        state.status = source->voltage_average() > 2.0f;
        state.status = state.status != leak_detector._default_reading[state.instance];
    } else {
        state.status = false;
    }
}
