#include "AP_RCProtocol_ThrottleFailsafe.h"

#if AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED

// a failsafe is declared after this many consecutive bind-time values,
// and cleared after the same number of consecutive good values:
#define THROTTLE_FAILSAFE_COUNTER_MAX 3

bool AP_RCProtocol_ThrottleFailsafe::update(uint16_t value, uint16_t threshold, bool threshold_is_maximum)
{
    bool bind_value;
    if (threshold_is_maximum) {
        // throttle-reversed case
        bind_value = value > threshold;
    } else {
        bind_value = value < threshold;
    }

    if (bind_value == _active) {
        _counter = 0;
    } else if (++_counter >= THROTTLE_FAILSAFE_COUNTER_MAX) {
        _active = bind_value;
        _counter = 0;
    }

    return bind_value;
}

#endif  // AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED
