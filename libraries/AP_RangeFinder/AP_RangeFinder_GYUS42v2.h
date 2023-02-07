#pragma once

#include "AP_RangeFinder.h"

#ifndef AP_RANGEFINDER_GYUS42V2_ENABLED
#define AP_RANGEFINDER_GYUS42V2_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#if AP_RANGEFINDER_GYUS42V2_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_GYUS42v2 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_GYUS42v2(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 9600;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;

    // find signature byte in buffer starting at start, moving that
    // byte and following bytes to start of buffer.
    bool find_signature_in_buffer(uint8_t start);

    uint8_t buffer[7];
    uint8_t buffer_used;
};

#endif  // AP_RANGEFINDER_GYUS42V2_ENABLED
