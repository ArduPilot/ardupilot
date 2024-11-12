#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_FDM_ENABLED

#include "AP_RCProtocol_FDM.h"

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

void AP_RCProtocol_FDM::update()
{
    const auto sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    const auto &fdm = sitl->state;

    // see if there's fresh input.  Until timestamps are worked out,
    // just check for non-zero values:
    if (fdm.rcin_chan_count == 0) {
        return;
    }

    // simulate RC input at 50Hz
    if (AP_HAL::millis() - last_input_ms < 20) {
        return;
    }

    last_input_ms = AP_HAL::millis();

    // scale from FDM 0-1 floats to PWM values
    // these are the values that will be fed into the autopilot.
    uint16_t pwm_input[16];
    const uint8_t count = MIN(ARRAY_SIZE(pwm_input), fdm.rcin_chan_count);
    for (uint8_t i=0; i<count; i++) {
        pwm_input[i] = 1000 + fdm.rcin[i] * 1000;
    }
    add_input(
        count,
        pwm_input,
        false,  // failsafe
        0, // check me
        0  // link quality
        );
}

#endif // AP_RCPROTOCOL_FDM_ENABLED
