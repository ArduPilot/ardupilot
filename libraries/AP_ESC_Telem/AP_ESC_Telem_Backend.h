#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM 1
#endif

#if HAL_WITH_ESC_TELEM

class AP_ESC_Telem;

class AP_ESC_Telem_Backend {
public:

    struct TelemetryData {
        int16_t  temperature_deg;   // degrees C, negative values allowed
        uint16_t voltage_cv;        // centi-volts
        uint16_t current_ca;        // centi-amps
        uint16_t consumption_mah;   // mAh
        uint32_t usage_s;           // usage seconds
        float    rpm;               // rpm
        int16_t  motor_temp_deg;    // degrees C, negative values allowed
        float    error_rate;        // error rate in percent
        uint16_t count;             // number of times updated
    };

    enum TelemetryType {
        TEMPERATURE = 1 << 0,
        MOTOR_TEMPERATURE  = 1 << 1,
        VOLTAGE     = 1 << 2,
        CURRENT     = 1 << 3,
        CONSUMPTION = 1 << 4,
        USAGE       = 1 << 5,
        RPM         = 1 << 6,
        ERROR_RATE  = 1 << 7
    };


    AP_ESC_Telem_Backend();

    /* Do not allow copies */
    AP_ESC_Telem_Backend(const AP_ESC_Telem_Backend &other) = delete;
    AP_ESC_Telem_Backend &operator=(const AP_ESC_Telem_Backend&) = delete;

protected:
    AP_ESC_Telem* _frontend;

    // callback to update the rpm in the frontend, should be called by the driver when new data is available
    void update_rpm(uint8_t esc_index, uint16_t new_rpm);

    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(uint8_t esc_index, const TelemetryData& new_data, uint8_t data_present_mask);

private:
};

#endif
