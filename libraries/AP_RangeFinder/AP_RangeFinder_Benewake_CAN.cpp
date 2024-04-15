#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RangeFinder_Benewake_CAN.h"
#include <AP_HAL/utility/sparse-endian.h>

#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED

// handler for incoming frames for H30 radar
bool AP_RangeFinder_Benewake_CAN::handle_frame_H30(AP_HAL::CANFrame &frame)
{
    /*
      The H30 produces 3 targets, each as 16 bit unsigned integers in
      cm. Only look at target1 for now
    */
    const uint16_t target1_cm = be16toh_ptr(&frame.data[0]);
    if (target1_cm == 0) {
        // no target gives 0
        return false;
    }
    //uint16_t target2 = be16toh_ptr(&frame.data[2]);
    //uint16_t target3 = be16toh_ptr(&frame.data[4]);

    accumulate_distance_m(target1_cm * 0.01);

    return true;
}

// handler for incoming frames. These come in at 100Hz
bool AP_RangeFinder_Benewake_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    if (frame.isExtended()) {
        // H30 radar uses extended frames
        const int32_t id = int32_t(frame.id & AP_HAL::CANFrame::MaskExtID);
        if (!is_correct_id(id)) {
            return false;
        }
        return handle_frame_H30(frame);
    }

    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskStdID;
    if (!is_correct_id(id)) {
        return false;
    }

    const uint16_t dist_cm = le16toh_ptr(&frame.data[0]);
    const uint16_t snr = le16toh_ptr(&frame.data[2]);
    if (snr_min != 0 && snr < uint16_t(snr_min.get())) {
        // too low signal strength
        return true;
    }

    accumulate_distance_m(dist_cm * 0.01);
    return true;
}

#endif  // AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
