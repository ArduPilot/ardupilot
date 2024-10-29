#pragma once

// #include "AP_RangeFinder_config.h"

/*
 questions:
 - bug on page 22 on malfunction codes
 - fixed length seems strange at 28 bytes
 - definition of snr field is missing in documentation
 - roll/pitch limits are in conflict, 3.2 vs
*/

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#ifndef AP_RANGEFINDER_AINSTEIN_LRD1_PRO_SERIAL_ENABLED
#define AP_RANGEFINDER_AINSTEIN_LRD1_PRO_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif
#if AP_RANGEFINDER_AINSTEIN_LRD1_PRO_SERIAL_ENABLED

// #include "AP_Math/crc.h"

static constexpr int8_t SIGNAL_QUALITY_MIN = 0;
static constexpr int8_t SIGNAL_QUALITY_MAX = 100;
static constexpr int8_t SIGNAL_QUALITY_UNKNOWN = -1;
static constexpr int8_t CHANGE_HEIGHT_THRESHOLD = 5;

uint16_t crc_sum_of_bytes_16(const uint8_t *data, uint16_t count);
uint8_t crc_sum_of_bytes(const uint8_t *data, uint16_t count);

class AP_RangeFinder_Ainstein_LRD1_Pro : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return new AP_RangeFinder_Ainstein_LRD1_Pro(_state, _params);
    }

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override
    {
        return 115200;
    }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;

    // 0 is no return value, 100 is perfect.  false means signal
    // quality is not available
    // int8_t get_signal_quality_pct() const override { return signal_quality_pct; };
    bool get_signal_quality_pct(uint8_t &quality_pct) const override
    {
        quality_pct = no_signal ? 0 : 100;
        return true;
    }

    static void report_malfunction(const uint16_t _malfunction_alert_);

    enum class MalfunctionAlert : uint16_t
    {
        Temperature = (1U << 0),        // 0x0001
        FPGAVoltage = (1U << 1),        // 0x0002
        Temperature_60G = (1U << 2),    // 0x0004
        Voltage_60G = (1U << 3),        // 0x0008
        Temperature_24G = (1U << 4),    // 0x0010
        TransmitPower_24G = (1U << 5),  // 0x0020
        OverCurrent = (1U << 6),        // 0x0040
        IFSignalSaturation = (1U << 7), // 0x0080
        Software_24G = (1U << 8),       // 0x0100
        Software_60G = (1U << 9),       // 0x0200
        AltitudeReading = (1U << 10),   // 0x0400
        AttitudeAngle = (1U << 11),     // 0x0800
        FrameError_60G = (1U << 12),    // 0x1000
        InvalidAltitude = (1U << 13),   // 0x2000
        InConAlttitude = (1U << 14),    // 0x4000
        BoardVoltage = (1U << 15),      // 0x8000
    };

    static constexpr uint8_t PACKET_SIZE = 32;
    uint8_t malfunction_alert_prev;
    bool no_signal = false;
    int8_t signal_quality_pct = SIGNAL_QUALITY_UNKNOWN;

    // Logging Function
    void Log_LRD1_Pro(uint16_t s_24, uint16_t s_60, uint16_t s_int, uint8_t snr_24, uint8_t snr_60, uint8_t snr_int) const;

    // Validating if the reading is good
    bool check_radar_reading(float &reading_m);
};
#endif