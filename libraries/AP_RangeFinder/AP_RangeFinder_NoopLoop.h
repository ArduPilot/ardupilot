#pragma once

#include "AP_RangeFinder.h"

#ifndef AP_RANGEFINDER_NOOPLOOP_ENABLED
#define AP_RANGEFINDER_NOOPLOOP_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && BOARD_FLASH_SIZE > 1024
#endif

#if AP_RANGEFINDER_NOOPLOOP_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_NoopLoop : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
            return new AP_RangeFinder_NoopLoop(_state, _params);
     }

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;


    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[16];
    uint8_t linebuf_len;
};

#endif  // AP_RANGEFINDER_NOOPLOOP_ENABLED
