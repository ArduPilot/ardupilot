#include "AP_RangeFinder_USD1_CAN.h"

#if AP_RANGEFINDER_USD1_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>

RangeFinder_MultiCAN *AP_RangeFinder_USD1_CAN::multican;

/*
  constructor
 */
AP_RangeFinder_USD1_CAN::AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend_CAN(_state, _params)
{
    if (multican == nullptr) {
        multican = new RangeFinder_MultiCAN(AP_CAN::Protocol::USD1, "USD1 MultiCAN");
        if (multican == nullptr) {
            AP_BoardConfig::allocation_error("USD1_CAN");
        }
    }

    {
        // add to linked list of drivers
        WITH_SEMAPHORE(multican->sem);
        auto *prev = multican->drivers;
        next = prev;
        multican->drivers = this;
    }
}

// handler for incoming frames. These come in at 100Hz
bool AP_RangeFinder_USD1_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskStdID;

    if (!is_correct_id(id)) {
        return false;
    }

    const uint16_t dist_cm = (frame.data[0]<<8) | frame.data[1];
    accumulate_distance_m(dist_cm * 0.01);
    return true;
}

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
