#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LANBAO_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_Lanbao : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_Lanbao(_state, _params);
    }

    // Lanbao is always 115200:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;

    uint8_t buf[6];
    uint8_t buf_len = 0;
};

#endif  // AP_RANGEFINDER_LANBAO_ENABLED
