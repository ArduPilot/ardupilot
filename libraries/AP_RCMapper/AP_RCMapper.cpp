/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include <AP_RCMapper.h>

const AP_Param::GroupInfo RCMapper::var_info[] PROGMEM = {
    // @Param: ROLL
    // @DisplayName: Roll channel
    // @Description: Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter.
    // @Range: 1 8
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ROLL",        0, RCMapper, _ch_roll, 1),

    // @Param: PITCH
    // @DisplayName: Pitch channel
    // @Description: Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter.
    // @Range: 1 8
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PITCH",       1, RCMapper, _ch_pitch, 2),

    // @Param: THROTTLE
    // @DisplayName: Throttle channel
    // @Description: Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended.
    // @Range: 1 8
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("THROTTLE",    2, RCMapper, _ch_throttle, 3),

    // @Param: YAW
    // @DisplayName: Yaw channel
    // @Description: Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter.
    // @Range: 1 8
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("YAW",         3, RCMapper, _ch_yaw, 4),

    AP_GROUPEND
};

// object constructor.
RCMapper::RCMapper(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
