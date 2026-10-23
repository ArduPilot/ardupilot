#include "AP_RangeFinder_USD1_CAN.h"

#if AP_RANGEFINDER_USD1_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>

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
