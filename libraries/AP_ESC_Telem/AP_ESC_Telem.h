#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_ESC_Telem_Backend.h"

#if HAL_WITH_ESC_TELEM

#ifdef NUM_SERVO_CHANNELS
 #define ESC_TELEM_MAX_ESCS NUM_SERVO_CHANNELS
#elif !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
 #define ESC_TELEM_MAX_ESCS 32
#else
 #define ESC_TELEM_MAX_ESCS 16
#endif

#define ESC_TELEM_DATA_TIMEOUT_MS 5000UL
#define ESC_RPM_DATA_TIMEOUT_US 1000000UL

class AP_ESC_Telem {
public:
    friend class AP_ESC_Telem_Backend;

    AP_ESC_Telem();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ESC_Telem);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_ESC_Telem *get_singleton();

    // get an individual ESC's slewed rpm if available, returns true on success
    bool get_rpm(uint8_t esc_index, float& rpm) const;

    // get an individual ESC's raw rpm if available
    bool get_raw_rpm(uint8_t esc_index, float& rpm) const;

    // return the average motor RPM
    float get_average_motor_rpm(uint32_t servo_channel_mask) const;

    // return the average motor RPM
    float get_average_motor_rpm() const { return get_average_motor_rpm(0xFFFFFFFF); }

    // determine whether all the motors in servo_channel_mask are running
    bool are_motors_running(uint32_t servo_channel_mask, float min_rpm) const;

    // get an individual ESC's temperature in centi-degrees if available, returns true on success
    bool get_temperature(uint8_t esc_index, int16_t& temp) const;

    // get an individual motor's temperature in centi-degrees if available, returns true on success
    bool get_motor_temperature(uint8_t esc_index, int16_t& temp) const;

    // get the highest ESC temperature in centi-degrees if available, returns true if there is valid data for at least one ESC
    bool get_highest_motor_temperature(int16_t& temp) const;

    // get an individual ESC's current in Ampere if available, returns true on success
    bool get_current(uint8_t esc_index, float& amps) const;

    // get an individual ESC's usage time in seconds if available, returns true on success
    bool get_usage_seconds(uint8_t esc_index, uint32_t& usage_sec) const;

    // get an individual ESC's voltage in Volt if available, returns true on success
    bool get_voltage(uint8_t esc_index, float& volts) const;

    // get an individual ESC's consumption in milli-Ampere.hour if available, returns true on success
    bool get_consumption_mah(uint8_t esc_index, float& consumption_mah) const;

    // return the average motor frequency in Hz for dynamic filtering
    float get_average_motor_frequency_hz(uint32_t servo_channel_mask) const { return get_average_motor_rpm(servo_channel_mask) * (1.0f / 60.0f); };

    // return the average motor frequency in Hz for dynamic filtering
    float get_average_motor_frequency_hz() const { return get_average_motor_frequency_hz(0xFFFFFFFF); }

    // return all of the motor frequencies in Hz for dynamic filtering
    uint8_t get_motor_frequencies_hz(uint8_t nfreqs, float* freqs) const;

    // get the number of ESCs that sent valid telemetry data in the last ESC_TELEM_DATA_TIMEOUT_MS
    uint8_t get_num_active_escs() const;

    // get mask of ESCs that sent valid telemetry data in the last
    // ESC_TELEM_DATA_TIMEOUT_MS
    uint32_t get_active_esc_mask() const;

    // return the last time telemetry data was received in ms for the given ESC or 0 if never
    uint32_t get_last_telem_data_ms(uint8_t esc_index) const {
        if (esc_index >= ESC_TELEM_MAX_ESCS) {return 0;}
        return _telem_data[esc_index].last_update_ms;
    }

    // send telemetry data to mavlink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    // udpate at 10Hz to log telemetry
    void update();

    // is rpm telemetry configured for the provided channel mask
    bool is_telemetry_active(uint32_t servo_channel_mask) const;

    // callback to update the rpm in the frontend, should be called by the driver when new data is available
    // can also be called from scripting
    void update_rpm(const uint8_t esc_index, const uint16_t new_rpm, const float error_rate);
    
private:
    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(const uint8_t esc_index, const AP_ESC_Telem_Backend::TelemetryData& new_data, const uint16_t data_mask);

    // rpm data
    volatile AP_ESC_Telem_Backend::RpmData _rpm_data[ESC_TELEM_MAX_ESCS];
    // telemetry data
    volatile AP_ESC_Telem_Backend::TelemetryData _telem_data[ESC_TELEM_MAX_ESCS];

    uint32_t _last_telem_log_ms[ESC_TELEM_MAX_ESCS];
    uint32_t _last_rpm_log_us[ESC_TELEM_MAX_ESCS];
    uint8_t next_idx;

    bool _have_data;

    AP_Int8 mavlink_offset;

    static AP_ESC_Telem *_singleton;
};

namespace AP {
    AP_ESC_Telem &esc_telem();
};

#endif // HAL_WITH_ESC_TELEM

