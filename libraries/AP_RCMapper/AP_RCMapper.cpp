/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include <AP_RCMapper.h>

const AP_Param::GroupInfo RCMapper::var_info[] PROGMEM = {
    // @Param: ROLL
    // @DisplayName: Roll channel
    // @Description: Roll channel number
    // @Range: 1 8
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ROLL",        0, RCMapper, _ch_roll, 1),

    // @Param: PITCH
    // @DisplayName: Pitch channel
    // @Description: Pitch channel number
    // @Range: 1 8
    // @Increment: 1
    AP_GROUPINFO("PITCH",       1, RCMapper, _ch_pitch, 2),

    // @Param: THROTTLE
    // @DisplayName: Throttle channel
    // @Description: Throttle channel number
    // @Range: 1 8
    // @Increment: 1
    AP_GROUPINFO("THROTTLE",    2, RCMapper, _ch_throttle, 3),

    // @Param: YAW
    // @DisplayName: Yaw channel
    // @Description: Yaw channel number
    // @Range: 1 8
    // @Increment: 1
    AP_GROUPINFO("YAW",         3, RCMapper, _ch_yaw, 4),

    AP_GROUPEND
};

// object constructor.
RCMapper::RCMapper(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
