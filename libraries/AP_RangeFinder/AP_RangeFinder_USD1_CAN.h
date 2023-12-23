#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_USD1_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

class AP_RangeFinder_USD1_CAN : public AP_RangeFinder_Backend_CAN {
public:

    AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // handler for incoming frames
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    static const struct AP_Param::GroupInfo var_info[];

private:

    static RangeFinder_MultiCAN *multican;

};

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
