#pragma once

#include "AP_RCMapper_config.h"

#if AP_RCMAPPER_ENABLED

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class RCMapper {
public:
    RCMapper();

    /* Do not allow copies */
    CLASS_NO_COPY(RCMapper);

    // get singleton instance
    static RCMapper *get_singleton()
    {
        return _singleton;
    }

    /// roll - return input channel number for roll / aileron input
    uint8_t roll() const { return _ch_roll; }

    /// pitch - return input channel number for pitch / elevator input
    uint8_t pitch() const { return _ch_pitch; }

    /// throttle - return input channel number for throttle input
    uint8_t throttle() const { return _ch_throttle; }

    /// yaw - return input channel number for yaw / rudder input
    uint8_t yaw() const { return _ch_yaw; }

    /// forward - return input channel number for forward input
    uint8_t forward() const { return _ch_forward; }

    /// lateral - return input channel number for lateral input
    uint8_t lateral() const { return _ch_lateral; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // channel mappings
    AP_Int8 _ch_roll;
    AP_Int8 _ch_pitch;
    AP_Int8 _ch_yaw;
    AP_Int8 _ch_throttle;
    AP_Int8 _ch_forward;
    AP_Int8 _ch_lateral;
    static RCMapper *_singleton;
};

namespace AP
{
RCMapper *rcmap();
};

#endif  // AP_RCMAPPER_ENABLED
