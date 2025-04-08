#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED

#include "AP_RangeFinder_TOFSenseP_CAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>

// handler for incoming frames. These come in at 10-30Hz
bool AP_RangeFinder_TOFSenseP_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t id = frame.id - 0x200U;

    if (!is_correct_id(id)) {
        return false;
    }

    const int32_t dist_mm = (int32_t)(frame.data[0] << 8U | frame.data[1] << 16U | frame.data[2] << 24U) >> 8;
    const uint8_t status = frame.data[3];
    const uint16_t snr = le16toh_ptr(&frame.data[4]);

    if ((snr_min != 0 && snr < uint16_t(snr_min.get())) || status > 0) {
        // too low signal strength or bad status
        return false;
    }

    accumulate_distance_m(dist_mm * 0.001);
    return true;
}

#endif  // AP_RANGEFINDER_TOFSenseP_CAN_ENABLED
