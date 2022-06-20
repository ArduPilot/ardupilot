#pragma once

#include "AP_RangeFinder.h"

#ifndef AP_RANGEFINDER_BENEWAKE_ENABLED
#define AP_RANGEFINDER_BENEWAKE_ENABLED AP_RANGEFINDER_ENABLED
#endif

#if AP_RANGEFINDER_BENEWAKE_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    virtual float model_dist_max_cm() const = 0;
    virtual bool has_signal_byte() const { return false; }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[10];
    uint8_t linebuf_len;
};

#endif  // AP_RANGEFINDER_BENEWAKE_ENABLED
