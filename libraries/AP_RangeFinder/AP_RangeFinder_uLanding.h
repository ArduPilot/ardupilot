#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_uLanding : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    uint16_t rx_bufsize() const override { return 128; }
    uint16_t tx_bufsize() const override { return 128; }

private:
    // detect uLanding Firmware Version
    bool detect_version(void);

    // get a reading
    bool get_reading(uint16_t &reading_cm) override;

    uint8_t  _linebuf[6];
    uint8_t  _linebuf_len;
    bool     _version_known;
    uint8_t  _header;
    uint8_t  _version;
};
