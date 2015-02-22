/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_Progmem.h>
#include "OpticalFlow.h"

const AP_Param::GroupInfo OpticalFlow::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: 光流启用/禁用
    // @Description: 设为启用（1）会启用光流。设为禁用（0）会禁用光流
    // @Values: 0:禁用, 1:启用
    // @User: Standard
    AP_GROUPINFO("_ENABLE", 0,  OpticalFlow,    _enabled,   0),

    // @Param: FXSCALER
    // @DisplayName: X轴光流比例因数校正
    // @Description: 这个设置了每一千比例因数校正应用在光流传感器X轴上的光学速率。这可以用来修正有效焦点长度的变化。正方向每增加1会增加X轴上的光流比例因数0.1%。 负值会减小比例因数。
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

    // @Param: FYSCALER
    // @DisplayName: Y轴光流比例因数校正
    // @Description: 这个设置了每一千比例因数校正应用在光流传感器Y轴上的光学速率。这可以用来修正有效焦点长度的变化。正方向每增加1会增加Y轴上的光流比例因数0.1%。 负值会减小比例因数。
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow(const AP_AHRS &ahrs) :
    _ahrs(ahrs),
    _device_id(0),
    _surface_quality(0),
    _last_update(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // healthy flag will be overwritten when init is called
    _flags.healthy = false;
};
