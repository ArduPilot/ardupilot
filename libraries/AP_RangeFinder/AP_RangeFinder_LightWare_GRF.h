#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder_LightWare_GRF_Common.h"
#include <AP_LightWareSerial/AP_LightWareSerial.h>

class AP_RangeFinder_LightWareGRF : public AP_RangeFinder_Backend_Serial, AP_LightWareSerial {
public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_LightWareGRF(_state, _params);
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Returns the MAVLink distance sensor type
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

    // Called periodically to fetch a new range reading
    bool get_reading(float &reading_m) override;

    // Returns read timeout in milliseconds
    uint16_t read_timeout_ms() const override { return 500; }

private:
    // Constructor
    AP_RangeFinder_LightWareGRF(RangeFinder::RangeFinder_State &_state,
                          AP_RangeFinder_Params &_params);

    using MessageID      = AP_RangeFinder_LightWare_GRF_Common::MessageID;
    using ConfigStep     = AP_RangeFinder_LightWare_GRF_Common::ConfigStep;

    // Send configuration messages to the rangefinder
    void configure_rangefinder();

    // Parses config responses and advances setup step
    void check_config(const MessageID &resp_cmd_id, const uint8_t* response_buf, const uint16_t& response_len);

    // Processes the latest message held in the _msg structure
    void process_message(float &sum_m, uint8_t &count);

    AP_RangeFinder_LightWare_GRF_Common _common; // shared params + protocol helpers

    uint32_t last_config_message_ms; // last time we sent a config message
    ConfigStep config_step;          // current configuration step
};

#endif // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED
