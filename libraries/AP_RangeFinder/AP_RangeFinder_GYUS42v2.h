#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_GYUS42v2 : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 9600;
    }

private:

    // get a reading
    bool get_reading(uint16_t &reading_cm) override;

    // find signature byte in buffer starting at start, moving that
    // byte and following bytes to start of buffer.
    bool find_signature_in_buffer(uint8_t start);

    uint8_t buffer[7];
    uint8_t buffer_used;
};
