/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include <AP_RCMapper.h>

const AP_Param::GroupInfo RCMapper::var_info[] PROGMEM = {
    // @Param: ROLL
    // @DisplayName: 横滚通道
    // @Description: 横滚通道数。这在你有一个不能改变通道顺序的遥控器时有用。横滚通常在通道1，但是你可以使用这个参数把它移到其他通道。
    // @Range: 1 8
    // @Increment: 1
    // @User: 高级
    AP_GROUPINFO("ROLL",        0, RCMapper, _ch_roll, 1),

    // @Param: PITCH
    // @DisplayName: 俯仰通道
    // @Description: 俯仰通道数。这在你有一个不能改变通道顺序的遥控器时有用。俯仰通常在通道2，但是你可以使用这个参数把它移到其他通道。
    // @Range: 1 8
    // @Increment: 1
    // @User: 高级
    AP_GROUPINFO("PITCH",       1, RCMapper, _ch_pitch, 2),

    // @Param: THROTTLE
    // @DisplayName: 油门通道
    // @Description: 油门通道数。这在你有一个不能改变通道顺序的遥控器时有用。油门通常在通道3，但是你可以使用这个参数把它移到其他通道。警告APM2.x：改变油门通道可能导致意外的失控保护如果接收机和板载PPM编码器的连接丢失。推荐禁用板载PPM编码器。
    // @Range: 1 8
    // @Increment: 1
    // @User: 高级
    AP_GROUPINFO("THROTTLE",    2, RCMapper, _ch_throttle, 3),

    // @Param: YAW
    // @DisplayName: 方向通道
    // @Description: 方向通道数。这在你有一个不能改变通道顺序的遥控器时有用。方向通常在通道4，但是你可以使用这个参数把它移到其他通道。
    // @Range: 1 8
    // @Increment: 1
    // @User: 高级
    AP_GROUPINFO("YAW",         3, RCMapper, _ch_yaw, 4),

    AP_GROUPEND
};

// object constructor.
RCMapper::RCMapper(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
