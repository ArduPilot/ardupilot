#include "AP_OpticalFlow_HereFlow.h"

#if AP_OPTICALFLOW_HEREFLOW_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

uint8_t AP_OpticalFlow_HereFlow::_node_id = 0;
AP_OpticalFlow_HereFlow* AP_OpticalFlow_HereFlow::_driver = nullptr;
AP_DroneCAN* AP_OpticalFlow_HereFlow::_ap_dronecan = nullptr;
/*
  constructor - registers instance at top Flow driver
 */
AP_OpticalFlow_HereFlow::AP_OpticalFlow_HereFlow(AP_OpticalFlow &flow) :
    OpticalFlow_backend(flow)
{
    if (_driver) {
        AP_HAL::panic("Only one instance of Flow supported!");
    }
    _driver = this;
}

//links the HereFlow messages to the backend
bool AP_OpticalFlow_HereFlow::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_measurement, driver_index) != nullptr);
}

//updates driver states based on received HereFlow messages
void AP_OpticalFlow_HereFlow::handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_hex_equipment_flow_Measurement &msg)
{
    if (_driver == nullptr) {
        return;
    }
    //protect from data coming from duplicate sensors,
    //as we only handle one Here Flow at a time as of now
    if (_ap_dronecan == nullptr) {
        _ap_dronecan = ap_dronecan;
        _node_id = transfer.source_node_id;
    }

    if (_ap_dronecan == ap_dronecan && _node_id == transfer.source_node_id) {
        WITH_SEMAPHORE(_driver->_sem);
        _driver->new_data = true;
        _driver->flow_integral = Vector2f(msg.flow_integral[0], msg.flow_integral[1]);
        _driver->rate_gyro_integral = Vector2f(msg.rate_gyro_integral[0], msg.rate_gyro_integral[1]);
        _driver->integral_time = msg.integration_interval;
        _driver->surface_quality = msg.quality;
    }
}

void AP_OpticalFlow_HereFlow::update()
{
    _push_state();
}

// Read the sensor
void AP_OpticalFlow_HereFlow::_push_state(void)
{
    WITH_SEMAPHORE(_sem);
    if (!new_data) {
        return;
    }
    struct AP_OpticalFlow::OpticalFlow_state state;
    const Vector2f flowScaler = _flowScaler();
    //setup scaling based on parameters
    float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
    float integralToRate = 1.0f / integral_time;
    //Convert to Raw Flow measurement to Flow Rate measurement
    state.flowRate = Vector2f{
        flow_integral.x * flowScaleFactorX,
        flow_integral.y * flowScaleFactorY
    } * integralToRate;
    state.bodyRate = rate_gyro_integral * integralToRate;
    state.surface_quality = surface_quality;
    _applyYaw(state.flowRate);
    _applyYaw(state.bodyRate);
    // hal.console->printf("DRV: %u %f %f\n", state.surface_quality, flowRate.length(), bodyRate.length());
    _update_frontend(state);
    new_data = false;
}

#endif // AP_OPTICALFLOW_HEREFLOW_ENABLED
