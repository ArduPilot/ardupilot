#pragma once
#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

class AP_RangeFinder_NRA24_CAN : public AP_RangeFinder_Backend_CAN {
public:
    AP_RangeFinder_NRA24_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::RadarCAN, "nra24")
    {
        // Nanoradar NRA24: 0.5m to 190m.  HexsoonRadar shares this
        // backend and comes in 20m, 100m and 200m variants
        params.min_distance.set_default(0.5f);
#if AP_RANGEFINDER_NRA24_CAN_ENABLED
        if (type() == RangeFinder::Type::NRA24_CAN) {
            params.max_distance.set_default(190.0f);
        }
#endif  // AP_RANGEFINDER_NRA24_CAN_ENABLED
    }

    void update(void) override;

    // handler for incoming frames
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:

    uint32_t get_radar_id(uint32_t id) const { return ((id & 0xF0U) >> 4U); }
    uint32_t last_heartbeat_ms; // last status message received from the sensor
};

#endif  // AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
