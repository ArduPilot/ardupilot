#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_USD1_CAN.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

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
    if ((AP_HAL::millis() - _last_reading_ms) > 500) {
        // if data is older than 500ms, report NoData
        set_status(RangeFinder::Status::NoData);
    } else if (new_data) {
        state.distance_cm = _distance_cm;
        state.last_reading_ms = _last_reading_ms;
        update_status();
        new_data = false;
    }
}

// handler for incoming frames
void AP_RangeFinder_USD1_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    _distance_cm = (frame.data[0]<<8) | frame.data[1];
    _last_reading_ms = AP_HAL::millis();
    new_data = true;
}

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
