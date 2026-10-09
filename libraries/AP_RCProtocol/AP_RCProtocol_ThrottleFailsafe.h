#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED

#include <stdint.h>

/*
  debounced detection of "bind time" values on the throttle channel.
  Some receivers indicate loss of contact with the transmitter by
  supplying the values they were bound with rather than by signalling a
  failsafe.
 */
class AP_RCProtocol_ThrottleFailsafe
{
public:

    // supply the throttle value from a frame.  threshold_is_maximum is
    // true in the throttle-reversed case.  Returns true if this value
    // is a bind-time value; active() returns the debounced state.
    bool update(uint16_t value, uint16_t threshold, bool threshold_is_maximum);

    void reset() {
        _active = false;
        _counter = 0;
    }

    // true if a failsafe has been declared:
    bool active() const { return _active; }

private:

    bool _active;
    // number of consecutive values disagreeing with _active:
    uint8_t _counter;
};

#endif  // AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED
