#include <AP_BoardConfig/AP_BoardConfig.h>
#include "OpticalFlow.h"
#include "AP_OpticalFlow_Onboard.h"
#include "AP_OpticalFlow_SITL.h"
#include "AP_OpticalFlow_Pixart.h"
#include "AP_OpticalFlow_PX4Flow.h"

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

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the optical flow sensor focal point in body frame. Positive X is forward of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the optical flow sensor focal point in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the optical flow sensor focal point in body frame. Positive Z is down from the origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 4, OpticalFlow, _pos_offset, 0.0f),

    // @Param: _ADDR
    // @DisplayName: Address on the bus
    // @Description: This is used to select between multiple possible I2C addresses for some sensor types. For PX4Flow you can choose 0 to 7 for the 8 possible addresses on the I2C bus.
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("_ADDR", 5,  OpticalFlow, _address,   0),
    
    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow(AP_AHRS_NavEKF &ahrs)
    : _ahrs(ahrs),
      _last_update_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_state, 0, sizeof(_state));

    // healthy flag will be overwritten on update
    _flags.healthy = false;
}

void OpticalFlow::init(void)
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    if (!backend) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK) {
            // possibly have pixhart on external SPI
            backend = AP_OpticalFlow_Pixart::detect(*this);
        }
        if (backend == nullptr) {
            backend = AP_OpticalFlow_PX4Flow::detect(*this);
        }
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        backend = new AP_OpticalFlow_SITL(*this);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
        backend = new AP_OpticalFlow_Onboard(*this);
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        backend = AP_OpticalFlow_PX4Flow::detect(*this);
#endif
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void OpticalFlow::update(void)
{
    if (backend != nullptr) {
        backend->update();
    }
    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
}

