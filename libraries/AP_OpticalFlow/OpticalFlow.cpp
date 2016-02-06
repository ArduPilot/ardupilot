/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "OpticalFlow.h"
#include "AP_OpticalFlow_Onboard.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo OpticalFlow::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Optical flow enable/disable
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_ENABLE", 0,  OpticalFlow,    _enabled,   0),

    // @Param: _FXSCALER
    // @DisplayName: X axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

    // @Param: _FYSCALER
    // @DisplayName: Y axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Flow sensor yaw alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
    // @Range: -18000 +18000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ORIENT_YAW", 3,  OpticalFlow,    _yawAngle_cd,   0),

    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow(AP_AHRS_NavEKF& ahrs) :
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    backend(new AP_OpticalFlow_PX4(*this)),
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    backend(new AP_OpticalFlow_HIL(*this)),
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE ||\
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    backend(new AP_OpticalFlow_Onboard(*this, ahrs)),
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    backend(new AP_OpticalFlow_Linux(*this)),
#else
    backend(NULL),
#endif
    _last_update_ms(0)
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
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
}

void OpticalFlow::setHIL(const struct OpticalFlow::OpticalFlow_state &state)
{ 
    if (backend) {
        backend->_update_frontend(state); 
    }
}
