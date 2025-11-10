#pragma once

#include "AP_RangeFinder_config.h"

/*
 questions:
 - bug on page 22 on malfunction codes
 - fixed length seems strange at 28 bytes
 - definition of snr field is missing in documentation
 - roll/pitch limits are in conflict, 3.2 vs 
*/
#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_Ainstein_LR_D1 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Ainstein_LR_D1(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    // make sure readings go out-of-range when necessary
    float max_distance() const override  {
        return MIN(AP_RangeFinder_Backend::max_distance(), 500);
    }
    float min_distance() const override {
        return MAX(AP_RangeFinder_Backend::min_distance(), 0.7);
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get_reading - read all samples, return last.  The device sends at
    // 40Hz, so there should only ever be one sample available.  Assuming
    // that something's gone wrong with scheduling, we will simply return
    // the last.
    bool get_reading(float &reading_m) override;

    // get a reading
    bool get_one_reading(float &reading_m);

    // 0 is no return value, 100 is perfect.  false means signal
    // quality is not available
    int8_t get_signal_quality_pct() const override { return signal_quality_pct; };

    static void report_malfunction(const uint16_t _malfunction_alert_, const uint16_t _malfunction_alert_prev_);

    enum class MalfunctionAlert : uint8_t {
        Temperature       = (1U << 0),  // 0x01
        Voltage           = (1U << 1),  // 0x02
        IFSignalSaturation= (1U << 6),  // 0x40
        AltitudeReading   = (1U << 7),  // 0x80
    };
    // see page 25
    static void report_malfunction_v19000(const uint16_t malfunction_alert, const uint16_t malfunction_alert_prev);
    enum class MalfunctionAlert_v19000 : uint16_t {
        MCUTemperature         = (1U << 0),   // 0x0001
        MCUVoltage             = (1U << 1),   // 0x0002
        IFTemperature          = (1U << 4),   // 0x0010
        IFSignalSaturation     = (1U << 7),   // 0x0080
        Software               = (1U << 8),   // 0x0100
        AltitudeReadingOverflow= (1U << 10),  // 0x0400
        Voltage                = (1U << 15),  // 0x8000
    };

    // Ainstein LR-D1 firmware emits a single message with the same
    // header bytes, size, device_id and length, but different
    // meanings for some of the fields!  We differentiate between the
    // "original" packet and the new-style packet by looking at "byte
    // 12" (at offset 11).  If it is not 0xff then we consider the
    // packet to be of the "v19.0.0" format.  In this new format there
    // is no object count, there are 16 malfunction alert bits, "byte
    // 12" contains the unit's self-assessed height-valid information
    // and a lot of fields move from "other object readings" to
    // "reserved"
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
        struct PACKED LRD1Packet_v19000 {
            uint8_t header_msb;
            uint8_t header_lsb;
            uint8_t device_id;
            uint8_t length;  // "fixed as 28 bytes"
            uint16_t malfunction_alert;
            uint16_t object1_alt;
            uint8_t object1_snr;
            uint16_t object1_velocity;
            uint8_t altitude_valid;
            uint8_t reserved[19];
            uint8_t checksum;  // (data4+data5+…+data29+data31) bitwise-AND with 0xFF
        } packet_v19000;
        uint8_t buffer[64];  // each packet is 32 bytes long

        // return checksum calculated from data in buffer
        uint8_t calculate_checksum() const;
    } u;
    uint8_t buffer_used;

    uint16_t malfunction_alert_prev;
    uint32_t malfunction_alert_last_send_ms;

    int8_t signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;    

    // ensures that there is a packet starting at offset 0 in the
    // buffer.  If that's not the case this returns false.  Search
    // starts at offset start in the buffer - if a packet header is
    // found at a non-zero offset then the data is moved to the start
    // of the buffer.
    bool move_signature_in_buffer(uint8_t start);
};
#endif  // AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
