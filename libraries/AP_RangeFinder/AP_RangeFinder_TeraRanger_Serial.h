#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_TeraRanger_Serial : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_TeraRanger_Serial(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[10];
    uint8_t linebuf_len;
};
#endif  // AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
