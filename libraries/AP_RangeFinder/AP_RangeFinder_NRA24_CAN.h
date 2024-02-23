#pragma once
#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

class AP_RangeFinder_NRA24_CAN : public AP_RangeFinder_Backend_CAN {
public:
    AP_RangeFinder_NRA24_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::NanoRadar, "nra24")
    {
    }

    void update(void) override;

    // handler for incoming frames
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    static const struct AP_Param::GroupInfo var_info[];

private:

    uint32_t get_radar_id(uint32_t id) const { return ((id & 0xF0U) >> 4U); }
    uint32_t last_heartbeat_ms; // last status message received from the sensor
};

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
