#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

class AP_RangeFinder_Benewake_CAN : public AP_RangeFinder_Backend_CAN {
public:
    AP_RangeFinder_Benewake_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // handler for incoming frames. Return true if consumed
    bool handle_frame(AP_HAL::CANFrame &frame) override;
    bool handle_frame_H30(AP_HAL::CANFrame &frame);

private:
    static RangeFinder_MultiCAN *multican;
};

#endif  // AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
