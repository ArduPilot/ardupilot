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
    int8_t get_signal_quality_pct() const override { return signal_quality_pct; };

    static void report_malfunction(const uint8_t _malfunction_alert_);

    enum class MalfunctionAlert : uint8_t {
        Temperature       = (1U << 0),  // 0x01
        Voltage           = (1U << 1),  // 0x02
        IFSignalSaturation= (1U << 6),  // 0x40
        AltitudeReading   = (1U << 7),  // 0x80
    };

    static constexpr uint8_t PACKET_SIZE = 32;
    uint8_t malfunction_alert_prev;
    uint32_t malfunction_alert_last_send_ms;
    int8_t signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;    
};
#endif