#include "AP_RCProtocol_ThrottleFailsafe.h"

#if AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED

bool AP_RCProtocol_ThrottleFailsafe::update_bind_value(bool bind_value)
{
    // the counter accumulates rather than demanding consecutive values,
    // which is the filter the vehicles applied before this moved here:
    // a throttle which flaps about the threshold still declares a
    // failsafe, and recovery still takes the counter back to zero
    if (bind_value) {
        if (_counter < COUNTER_MAX) {
            _counter++;
        }
        if (_counter >= COUNTER_MAX) {
            _active = true;
        }
    } else {
        if (_counter > 0) {
            _counter--;
        }
        if (_counter == 0) {
            _active = false;
        }
    }

    return bind_value;
}

#endif  // AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED
