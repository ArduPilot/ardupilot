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

    // supply the throttle value from a frame.  A value at the threshold
    // is a bind-time value, as is anything beyond it;
    // threshold_is_maximum is true in the throttle-reversed case.
    // Returns true if this value is a bind-time value; active() returns
    // the debounced state.
    bool update(uint16_t value, uint16_t threshold, bool threshold_is_maximum) {
        return update_bind_value(is_bind_value(value, threshold, threshold_is_maximum));
    }

    // as update(), for a caller which has determined for itself
    // whether it is looking at a bind-time value:
    bool update_bind_value(bool bind_value);

    static bool is_bind_value(uint16_t value, uint16_t threshold, bool threshold_is_maximum) {
        if (threshold_is_maximum) {
            // the throttle-reversed case
            return value >= threshold;
        }
        return value <= threshold;
    }

    // seed the debounced state.  Used when one consumer takes this test
    // over from another, so that the handover does not look like a
    // change of state:
    void set_active(bool active) {
        _active = active;
        _counter = active ? COUNTER_MAX : 0;
    }

    void reset() {
        set_active(false);
    }

    // true if a failsafe has been declared:
    bool active() const { return _active; }

private:

    // a failsafe is declared after this many bind-time values, and
    // cleared after the same number of good ones:
    static const uint8_t COUNTER_MAX = 3;

    bool _active;
    // counts up towards COUNTER_MAX on bind-time values and back down
    // on good ones:
    uint8_t _counter;
};

#endif  // AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED
