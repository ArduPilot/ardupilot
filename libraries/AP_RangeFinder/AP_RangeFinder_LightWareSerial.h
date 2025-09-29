#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#include <AP_Logger/AP_Logger.h>

#ifndef AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
#define AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED

class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_LightWareSerial(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    bool get_signal_quality_pct(uint8_t &quality_pct) const override {
        quality_pct = no_signal ? 0 : 100;
        return true;
    }

private:
    // get a reading
    bool get_reading(float &reading_m) override;
    bool is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max);
    int8_t get_distance_from_lidar_reply(char *reply, float &distance_m);
    // Logging Function
    void Log_LW20_C(float ldf_reading_cm, float ldl_reading_cm, float integrated_reading_cm);

    char linebuf[50];           // legacy protocol buffer
    uint8_t linebuf_len;        // legacy protocol buffer length
    uint32_t last_init_ms;      // init time used to switch lw20 to serial mode
    uint8_t high_byte;          // binary protocol high byte
    bool high_byte_received;    // true if high byte has been received
    float ldf_val_m;        // lidar first reading in m
    float ldl_val_m;        // lidar last reading in m

    // automatic protocol decision variables
    enum class ProtocolState {
        UNKNOWN,    // the protocol used is not yet known
        LEGACY,     // legacy protocol, distances are sent as strings
        BINARY      // binary protocol, distances are sent using two bytes
    } protocol_state;
    uint8_t legacy_valid_count;
    uint8_t binary_valid_count;

    bool no_signal = false;
};

#endif  // AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
