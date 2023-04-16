#include "AP_Camera_Relay.h"

#if AP_CAMERA_RELAY_ENABLED

#include <AP_Relay/AP_Relay.h>

// update - should be called at 50hz
void AP_Camera_Relay::update()
{
    if (trigger_counter > 0) {
        trigger_counter--;
    } else {
        AP_Relay *ap_relay = AP::relay();
        if (ap_relay == nullptr) {
            return;
        }
        if (_params.relay_on) {
            ap_relay->off(0);
        } else {
            ap_relay->on(0);
        }
    }

    // call parent update
    AP_Camera_Backend::update();
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_Relay::trigger_pic()
{
    // fail if have not completed previous picture
    if (trigger_counter > 0) {
        return false;
    }

    // exit immediately if no relay is setup
    AP_Relay *ap_relay = AP::relay();
    if (ap_relay == nullptr) {
        return false;
    }

    if (_params.relay_on) {
        ap_relay->on(0);
    } else {
        ap_relay->off(0);
    }

    // set counter to move servo to off position after this many iterations of update (assumes 50hz update rate)
    trigger_counter = constrain_float(_params.trigger_duration * 50, 0, UINT16_MAX);

    return true;
}

#endif // AP_CAMERA_RELAY_ENABLED
