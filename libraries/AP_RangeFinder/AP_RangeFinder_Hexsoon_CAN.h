#pragma once
#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_HEXSOON_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"
#include <AP_HAL/utility/sparse-endian.h>

class AP_RangeFinder_Hexsoon_CAN : public AP_RangeFinder_Backend_CAN {
public:
    AP_RangeFinder_Hexsoon_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::Hexsoon, "hexsoon")
    {
    }

    void update(void) override;

    // handler for incoming frames, returns true if packet consumed
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // message ids
    enum class MessageId : uint16_t {
        PARAMETER_CONFIGURATION = 0x200,
        RADAR_STATUS_INFO = 0x201,
        OBJECT_LIST_STATUS = 0x60A,
        OBJECT_GENERAL_INFO = 0x60B,
        VERSION_INFO = 0x700
    };

    // local variables
    bool banner_sent;           // true once lidar version has been sent to the user
    uint32_t last_heartbeat_ms; // last status message received from the sensor
};

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
