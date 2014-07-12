/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_Progmem.h>
#include "OpticalFlow.h"

const AP_Param::GroupInfo OpticalFlow::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Optical flow enable/disable
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_ENABLE", 0,  OpticalFlow,    _enabled,   0),

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
