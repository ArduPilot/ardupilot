#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_MaxsonarSerialLV : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_MaxsonarSerialLV(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    AP_RangeFinder_MaxsonarSerialLV(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // get a reading
    bool get_reading(float &reading_m) override;

    uint16_t read_timeout_ms() const override { return 500; }

    char linebuf[10];
    uint8_t linebuf_len = 0;
};

#endif  // AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
