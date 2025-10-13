#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_NRA24_Serial : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_NRA24_Serial(_state, _params);
    }

    void update(void) override;

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    uint8_t linebuf[14];
    uint8_t linebuf_len;

    uint32_t last_heartbeat_ms; // last status message received from the sensor
};
#endif  // AP_RANGEFINDER_NRA24_SERIAL_ENABLED
