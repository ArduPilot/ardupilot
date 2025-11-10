#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_OpticalFlow.h"

#if AP_OPTICALFLOW_ENABLED

#include "AP_OpticalFlow_Onboard.h"
#include "AP_OpticalFlow_SITL.h"
#include "AP_OpticalFlow_Pixart.h"
#include "AP_OpticalFlow_PX4Flow.h"
#include "AP_OpticalFlow_CXOF.h"
#include "AP_OpticalFlow_MAV.h"
#include "AP_OpticalFlow_HereFlow.h"
#include "AP_OpticalFlow_MSP.h"
#include "AP_OpticalFlow_UPFLOW.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

#ifndef OPTICAL_FLOW_TYPE_DEFAULT
  #define OPTICAL_FLOW_TYPE_DEFAULT Type::NONE
#endif

const AP_Param::GroupInfo AP_OpticalFlow::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Optical flow sensor type
    // @Description: Optical flow sensor type
    // @SortValues: AlphabeticalZeroAtTop
    // @Values: 0:None, 1:PX4Flow, 2:Pixart, 3:Bebop, 4:CXOF, 5:MAVLink, 6:DroneCAN, 7:MSP, 8:UPFLOW, 10:SITL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 0,  AP_OpticalFlow,    _type,   (float)OPTICAL_FLOW_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: _FXSCALER
    // @DisplayName: X axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -800 +800
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  AP_OpticalFlow,    _flowScalerX,   0),

    // @Param: _FYSCALER
    // @DisplayName: Y axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -800 +800
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  AP_OpticalFlow,    _flowScalerY,   0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Flow sensor yaw alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
    // @Units: cdeg
    // @Range: -17999 +18000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("_ORIENT_YAW", 3,  AP_OpticalFlow,    _yawAngle_cd,   0),

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the optical flow sensor focal point in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the optical flow sensor focal point in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the optical flow sensor focal point in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_POS", 4, AP_OpticalFlow, _pos_offset, 0.0f),

    // @Param: _ADDR
    // @DisplayName: Address on the bus
    // @Description: This is used to select between multiple possible I2C addresses for some sensor types. For PX4Flow you can choose 0 to 7 for the 8 possible addresses on the I2C bus.
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("_ADDR", 5,  AP_OpticalFlow, _address,   0),

    // @Param{Rover}: _HGT_OVR
    // @DisplayName: Height override of sensor above ground
    // @Description: This is used in rover vehicles, where the sensor is a fixed height above the ground
    // @Units: m
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO_FRAME("_HGT_OVR", 6,  AP_OpticalFlow, _height_override,   0.0f, AP_PARAM_FRAME_ROVER),

    // @Param: _OPTIONS
    // @DisplayName: Optical flow options
    // @Description: Optical flow options. Bit 0 should be set if the sensor is stabilised (e.g. mounted on a gimbal)
    // @Bitmask: 0:Roll/Pitch stabilised
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 7,  AP_OpticalFlow, _options,   0),

    AP_GROUPEND
};

// default constructor
AP_OpticalFlow::AP_OpticalFlow()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OpticalFlow::init(uint32_t log_bit)
{
     _log_bit = log_bit;

    // return immediately if not enabled or backend already created
    if ((_type == Type::NONE) || (backend != nullptr)) {
        return;
    }

    switch ((Type)_type) {
    case Type::NONE:
        break;
#if AP_OPTICALFLOW_PX4FLOW_ENABLED
    case Type::PX4FLOW:
        backend = AP_OpticalFlow_PX4Flow::detect(*this);
        break;
#endif  // AP_OPTICALFLOW_PX4FLOW_ENABLED
#if AP_OPTICALFLOW_PIXART_ENABLED
    case Type::PIXART:
        backend = AP_OpticalFlow_Pixart::detect("pixartflow", *this);
        if (backend == nullptr) {
            backend = AP_OpticalFlow_Pixart::detect("pixartPC15", *this);
        }
        break;
#endif  // AP_OPTICALFLOW_PIXART_ENABLED
#if AP_OPTICALFLOW_ONBOARD_ENABLED
    case Type::BEBOP:
        backend = NEW_NOTHROW AP_OpticalFlow_Onboard(*this);
        break;
#endif  // AP_OPTICALFLOW_ONBOARD_ENABLED
#if AP_OPTICALFLOW_CXOF_ENABLED
    case Type::CXOF:
        backend = AP_OpticalFlow_CXOF::detect(*this);
        break;
#endif  // AP_OPTICALFLOW_CXOF_ENABLED
#if AP_OPTICALFLOW_MAV_ENABLED
    case Type::MAVLINK:
        backend = AP_OpticalFlow_MAV::detect(*this);
        break;
#endif  // AP_OPTICALFLOW_MAV_ENABLED
#if AP_OPTICALFLOW_HEREFLOW_ENABLED
    case Type::UAVCAN:
        backend = NEW_NOTHROW AP_OpticalFlow_HereFlow(*this);
        break;
#endif  // AP_OPTICALFLOW_HEREFLOW_ENABLED
#if HAL_MSP_OPTICALFLOW_ENABLED
    case Type::MSP:
        backend = AP_OpticalFlow_MSP::detect(*this);
        break;
#endif  // HAL_MSP_OPTICALFLOW_ENABLED
#if AP_OPTICALFLOW_UPFLOW_ENABLED
    case Type::UPFLOW:
        backend = AP_OpticalFlow_UPFLOW::detect(*this);
        break;
#endif  // AP_OPTICALFLOW_UPFLOW_ENABLED
#if AP_OPTICALFLOW_SITL_ENABLED
    case Type::SITL:
        backend = NEW_NOTHROW AP_OpticalFlow_SITL(*this);
        break;
#endif  // AP_OPTICALFLOW_SITL_ENABLED
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void AP_OpticalFlow::update(void)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }
    if (backend != nullptr) {
        backend->update();
    }

    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);

#if AP_OPTICALFLOW_CALIBRATOR_ENABLED
    // update calibrator and save resulting scaling
    if (_calibrator != nullptr) {
        if (_calibrator->update()) {
            // apply new calibration values
            const Vector2f new_scaling = _calibrator->get_scalars();
            const float flow_scalerx_as_multiplier = (1.0 + (_flowScalerX * 0.001)) * new_scaling.x;
            const float flow_scalery_as_multiplier = (1.0 + (_flowScalerY * 0.001)) * new_scaling.y;
            _flowScalerX.set_and_save_ifchanged((flow_scalerx_as_multiplier - 1.0) * 1000.0);
            _flowScalerY.set_and_save_ifchanged((flow_scalery_as_multiplier - 1.0) * 1000.0);
            _flowScalerX.notify();
            _flowScalerY.notify();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlowCal: FLOW_FXSCALER=%d, FLOW_FYSCALER=%d", (int)_flowScalerX, (int)_flowScalerY);
        }
    }
#endif
}

void AP_OpticalFlow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msg(msg);
    }
}

#if HAL_MSP_OPTICALFLOW_ENABLED
void AP_OpticalFlow::handle_msp(const MSP::msp_opflow_data_message_t &pkt)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msp(pkt);
    }
}
#endif //HAL_MSP_OPTICALFLOW_ENABLED

#if AP_OPTICALFLOW_CALIBRATOR_ENABLED
// start calibration
void AP_OpticalFlow::start_calibration()
{
    if (_calibrator == nullptr) {
        _calibrator = NEW_NOTHROW AP_OpticalFlow_Calibrator();
        if (_calibrator == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "FlowCal: failed to start");
            return;
        }
    }
    if (_calibrator != nullptr) {
        _calibrator->start();
    }
}

// stop calibration
void AP_OpticalFlow::stop_calibration()
{
    if (_calibrator != nullptr) {
        _calibrator->stop();
    }
}
#endif

void AP_OpticalFlow::update_state(const OpticalFlow_state &state)
{
    _state = state;
    _last_update_ms = AP_HAL::millis();

#if AP_AHRS_ENABLED
    // write to log and send to EKF if new data has arrived
    AP::ahrs().writeOptFlowMeas(quality(),
                                _state.flowRate,
                                _state.bodyRate,
                                _last_update_ms,
                                get_pos_offset(),
                                get_height_override());
#endif
#if HAL_LOGGING_ENABLED
    Log_Write_Optflow();
#endif
}

#if HAL_LOGGING_ENABLED
void AP_OpticalFlow::Log_Write_Optflow()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    if (_log_bit != (uint32_t)-1 &&
        !logger->should_log(_log_bit)) {
        return;
    }

    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : _state.surface_quality,
        flow_x          : _state.flowRate.x,
        flow_y          : _state.flowRate.y,
        body_x          : _state.bodyRate.x,
        body_y          : _state.bodyRate.y
    };
    logger->WriteBlock(&pkt, sizeof(pkt));
}
#endif  // HAL_LOGGING_ENABLED


// singleton instance
AP_OpticalFlow *AP_OpticalFlow::_singleton;

namespace AP {

AP_OpticalFlow *opticalflow()
{
    return AP_OpticalFlow::get_singleton();
}

}

#endif // AP_OPTICALFLOW_ENABLED
