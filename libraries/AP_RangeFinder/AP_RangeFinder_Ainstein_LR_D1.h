#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED

/*
 questions:
 - bug on page 22 on malfunction codes
 - fixed length seems strange at 28 bytes
 - definition of snr field is missing in documentation
 - roll/pitch limits are in conflict, 3.2 vs 
*/

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_Ainstein_LR_D1 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_Ainstein_LR_D1(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;

    // 0 is no return value, 100 is perfect.  false means signal
    // quality is not available
    bool get_signal_quality_pct(uint8_t &quality_pct) const override;

    // find signature byte in buffer starting at start, moving that
    // byte and following bytes to start of buffer.
    bool move_signature_in_buffer(uint8_t start);

    enum class MalfunctionAlert {
        Temperature = 0x01,
        Voltage = 0x02,
        IFSignalSaturation = 0x04,
        AltitudeReading = 0x08,
    };

    union LRD1Union {
        struct PACKED LRD1Packet {
            uint8_t header_msb;
            uint8_t header_lsb;
            uint8_t device_id;
            uint8_t length;  // "fixed as 28 bytes"
            uint8_t malfunction_alert;
            uint8_t objects_number;  // "fixed as 1"
            uint16_t object1_alt;
            uint8_t object1_snr;
            uint16_t object1_velocity;
            uint8_t unused[20];
            uint8_t checksum;  // (data4+data5+…+data29+data31) bitwise-AND with 0xFF
        } packet;
        uint8_t buffer[64];  // each packet is 30 bytes long

        // return checksum calculated from data in buffer
        uint8_t calculate_checksum() const;
    } u;
    uint8_t buffer_used;

    uint8_t snr = 255;  // stashed SNR value after successful reading; 255 is unknown
};

#endif  // AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
