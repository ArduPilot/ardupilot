#pragma once

#include <AP_HAL/AP_HAL.h>

#if defined(NUM_SERVO_CHANNELS) && NUM_SERVO_CHANNELS == 0
#define HAL_WITH_ESC_TELEM 0
#elif !defined(HAL_WITH_ESC_TELEM)
#define HAL_WITH_ESC_TELEM (HAL_SUPPORT_RCOUT_SERIAL || HAL_MAX_CAN_PROTOCOL_DRIVERS) && !defined(HAL_BUILD_AP_PERIPH)
#endif

#if HAL_WITH_ESC_TELEM

class AP_ESC_Telem;

class AP_ESC_Telem_Backend {
public:

    struct TelemetryData {
        int16_t  temperature_cdeg;  // centi-degrees C, negative values allowed
        float voltage;              // Volt
        float current;              // Ampere
        float consumption_mah;      // milli-Ampere.hours
        uint32_t usage_s;           // usage seconds
        int16_t  motor_temp_cdeg;   // centi-degrees C, negative values allowed
        uint32_t last_update_ms;    // last update time in milliseconds, determines whether active
        uint16_t types;             // telemetry types present
        uint16_t count;             // number of times updated
    };

    struct RpmData {
        float    rpm;               // rpm
        float    prev_rpm;          // previous rpm
        float    error_rate;        // error rate in percent
        uint32_t last_update_us;    // last update time, determines whether active
        float    update_rate_hz;
    };

    enum TelemetryType {
        TEMPERATURE = 1 << 0,
        MOTOR_TEMPERATURE  = 1 << 1,
        VOLTAGE     = 1 << 2,
        CURRENT     = 1 << 3,
        CONSUMPTION = 1 << 4,
        USAGE       = 1 << 5
    };


    AP_ESC_Telem_Backend();

    /* Do not allow copies */
    AP_ESC_Telem_Backend(const AP_ESC_Telem_Backend &other) = delete;
    AP_ESC_Telem_Backend &operator=(const AP_ESC_Telem_Backend&) = delete;

protected:
    // callback to update the rpm in the frontend, should be called by the driver when new data is available
    void update_rpm(const uint8_t esc_index, const uint16_t new_rpm, const float error_rate = 0.0f);

    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(const uint8_t esc_index, const TelemetryData& new_data, const uint16_t data_present_mask);

private:
    AP_ESC_Telem* _frontend;
};

#else

// dummy empty class
class AP_ESC_Telem_Backend {
public:
    AP_ESC_Telem_Backend(){};
};

#endif
