/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Progmem.h>
#include "OpticalFlow.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo OpticalFlow::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Optical flow enable/disable
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_ENABLE", 0,  OpticalFlow,    _enabled,   0),

    // @Param: FXSCALER
    // @DisplayName: X axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

    // @Param: FYSCALER
    // @DisplayName: Y axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow(void) :
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    backend(new AP_OpticalFlow_PX4(*this))
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    backend(new AP_OpticalFlow_HIL(*this))
#else
    backend(NULL)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_state, 0, sizeof(_state));

    // healthy flag will be overwritten on update
    _flags.healthy = false;
};

void OpticalFlow::init(void)
{
    if (backend != NULL) {
        backend->init();
    } else {
        _enabled = 0;
    }
}

void OpticalFlow::update(void)
{
    if (backend != NULL) {
        backend->update();
    }
    // only healthy if the data is less than 0.5s old
    _flags.healthy = (hal.scheduler->millis() - _last_update_ms < 500);
}

void OpticalFlow::setHIL(const struct OpticalFlow::OpticalFlow_state &state)
{ 
    if (backend) {
        backend->_update_frontend(state); 
    }
}
