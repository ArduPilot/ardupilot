#pragma once

#include "AP_ESC_Telem_config.h"

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
#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
        uint16_t edt2_status;       // status reported by Extended DShot Telemetry v2
        uint16_t edt2_stress;       // stress reported in dedicated messages by Extended DShot Telemetry v2
#endif
#if AP_EXTENDED_ESC_TELEM_ENABLED
        uint8_t input_duty;         // input duty cycle
        uint8_t output_duty;        // output duty cycle
        uint32_t flags;             // Status flags
        uint8_t power_percentage;   // Percentage of output power
#endif // AP_EXTENDED_ESC_TELEM_ENABLED

        // set to false if no data has been received within the timeout period
        bool any_data_valid;

        // return true if the data is stale
        bool stale() const volatile;

        //  return true if the requested type of data is available and not stale
        bool valid(const uint16_t type_mask) const volatile;
    };

    struct RpmData {
        float    rpm;               // rpm
        float    prev_rpm;          // previous rpm
        float    error_rate;        // error rate in percent
        uint32_t last_update_us;    // last update time, greater then 0 means we've gotten data at some point
        float    update_rate_hz;
        bool     data_valid;        // if this isn't set to true, then the ESC data should be ignored
    };

    enum TelemetryType {
        TEMPERATURE = 1 << 0,
        MOTOR_TEMPERATURE  = 1 << 1,
        VOLTAGE     = 1 << 2,
        CURRENT     = 1 << 3,
        CONSUMPTION = 1 << 4,
        USAGE       = 1 << 5,
        TEMPERATURE_EXTERNAL = 1 << 6,
        MOTOR_TEMPERATURE_EXTERNAL  = 1 << 7,
#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
        EDT2_STATUS = 1 << 8,
        EDT2_STRESS = 1 << 9,
#endif
#if AP_EXTENDED_ESC_TELEM_ENABLED
        INPUT_DUTY  = 1 << 10,
        OUTPUT_DUTY = 1 << 11,
        FLAGS       = 1 << 12,
        POWER_PERCENTAGE = 1 << 13,
#endif // AP_EXTENDED_ESC_TELEM_ENABLED
    };


    AP_ESC_Telem_Backend();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ESC_Telem_Backend);

protected:
    // callback to update the rpm in the frontend, should be called by the driver when new data is available
    void update_rpm(const uint8_t esc_index, const float new_rpm, const float error_rate = 0.0f);

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
