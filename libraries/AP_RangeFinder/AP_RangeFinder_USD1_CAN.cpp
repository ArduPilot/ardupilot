#include "AP_RangeFinder_USD1_CAN.h"

#if AP_RANGEFINDER_USD1_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>

/*
  constructor
 */
AP_RangeFinder_USD1_CAN::AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    CANSensor("USD1"),
    AP_RangeFinder_Backend(_state, _params)
{
    register_driver(AP_CANManager::Driver_Type_USD1);
}

// update state
void AP_RangeFinder_USD1_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    if (_distance_count == 0 && now - state.last_reading_ms > 500) {
        // no new data.
        set_status(RangeFinder::Status::NoData);
    } else if (_distance_count != 0) {
        state.distance_m = _distance_sum / _distance_count;
        state.last_reading_ms = AP_HAL::millis();
        _distance_sum = 0;
        _distance_count = 0;
        update_status();
    }
}

// handler for incoming frames. These come in at 100Hz
void AP_RangeFinder_USD1_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    const uint16_t dist_cm = (frame.data[0]<<8) | frame.data[1];
    _distance_sum += dist_cm * 0.01;
    _distance_count++;
}

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
