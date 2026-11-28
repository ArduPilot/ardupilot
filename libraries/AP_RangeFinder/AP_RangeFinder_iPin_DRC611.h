#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_IPIN_DRC611_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder.h"

class AP_RangeFinder_iPin_DRC611 : public AP_RangeFinder_Backend_Serial
{
public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_iPin_DRC611(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    bool get_reading(float &reading_m) override;
    void resync_frame();

    uint8_t frame[6] {};
    uint8_t frame_len;

    uint32_t last_start_ms;
    uint32_t last_valid_ms;
    bool start_cmd_attempted;
};

#endif // AP_RANGEFINDER_IPIN_DRC611_ENABLED
