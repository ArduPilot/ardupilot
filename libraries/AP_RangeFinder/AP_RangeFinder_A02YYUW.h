#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_A02YYUW_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_A02YYUW : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_A02YYUW(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    int8_t get_signal_quality_pct() const override {
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;

    uint32_t last_request_ms;

    bool no_signal;

    uint8_t buf[4];
    uint8_t buf_len;
};

#endif  // AP_RANGEFINDER_A02YYUW_ENABLED
