#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include "AP_ESC_Telem_config.h"
#include "AP_ESC_Telem_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>

#if HAL_WITH_ESC_TELEM

#define ESC_TELEM_MAX_ESCS NUM_SERVO_CHANNELS
static_assert(ESC_TELEM_MAX_ESCS > 0, "Cannot have 0 ESC telemetry instances");

#define ESC_TELEM_DATA_TIMEOUT_MS 5000UL
#define ESC_RPM_DATA_TIMEOUT_US 1000000UL

enum class AP_ESC_Telem_Protocol : uint8_t {
    NONE =  0,
    HOBBYWING_PLATINUM_PRO_V3 = 1,
    HOBBYWING_PLATINUM_V4 = 2,
    HOBBYWING_XROTOR_V4 = 3,
    __LAST__ = 4,
};

class AP_ESC_Telem_MotorGroup {
public:

    AP_ESC_Telem_MotorGroup() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    AP_Enum<AP_ESC_Telem_Protocol> protocol;
    AP_Int8 poles;
    AP_Int32 mask;

    static const struct AP_Param::GroupInfo var_info[];
};

class AP_ESC_Telem {
public:
    friend class AP_ESC_Telem_Backend;

    AP_ESC_Telem();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ESC_Telem);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_ESC_Telem *get_singleton();

    void init();

    // get an individual ESC's slewed rpm if available, returns true on success
    bool get_rpm(uint8_t esc_index, float& rpm) const;

    // get an individual ESC's raw rpm if available
    bool get_raw_rpm(uint8_t esc_index, float& rpm) const;

    // get raw telemetry data, used by IOMCU
    const volatile AP_ESC_Telem_Backend::TelemetryData& get_telem_data(uint8_t esc_index) const {
        return _telem_data[esc_index];
    }

    // return the average motor RPM
    float get_average_motor_rpm(uint32_t servo_channel_mask) const;

    // return the average motor RPM
    float get_average_motor_rpm() const { return get_average_motor_rpm(0xFFFFFFFF); }

    // determine whether all the motors in servo_channel_mask are running
    bool are_motors_running(uint32_t servo_channel_mask, float min_rpm, float max_rpm) const;

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

    // library update call; updates backends and logs data
    void update();

    // is rpm telemetry configured for the provided channel mask
    bool is_telemetry_active(uint32_t servo_channel_mask) const;

    // callback to update the rpm in the frontend, should be called by the driver when new data is available
    // can also be called from scripting
    void update_rpm(const uint8_t esc_index, const float new_rpm, const float error_rate);

    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(const uint8_t esc_index, const AP_ESC_Telem_Backend::TelemetryData& new_data, const uint16_t data_mask);

#if AP_SCRIPTING_ENABLED
    /*
      set RPM scale factor from script
     */
    void set_rpm_scale(const uint8_t esc_index, const float scale_factor);
#endif

private:

    // helper that validates RPM data
    static bool rpm_data_within_timeout (const volatile AP_ESC_Telem_Backend::RpmData &instance, const uint32_t now_us, const uint32_t timeout_us);
    static bool was_rpm_data_ever_reported (const volatile AP_ESC_Telem_Backend::RpmData &instance);

    // rpm data
    volatile AP_ESC_Telem_Backend::RpmData _rpm_data[ESC_TELEM_MAX_ESCS];
    // telemetry data
    volatile AP_ESC_Telem_Backend::TelemetryData _telem_data[ESC_TELEM_MAX_ESCS];

    uint32_t _last_telem_log_ms[ESC_TELEM_MAX_ESCS];
    uint32_t _last_rpm_log_us[ESC_TELEM_MAX_ESCS];
    uint8_t next_idx;

#if AP_SCRIPTING_ENABLED
    // allow for scaling of RPMs via lua scripts
    float rpm_scale_factor[ESC_TELEM_MAX_ESCS];
    uint32_t rpm_scale_mask;
#endif
    
    bool _have_data;

    AP_Int8 mavlink_offset;

    // methods for update() to call
    void update_logging();
    void update_telemetry();


#if AP_ESC_TELEM_MAX_PROTOCOL_GROUPS > 0
    AP_ESC_Telem_MotorGroup motor_group[AP_ESC_TELEM_MAX_PROTOCOL_GROUPS];
    // num_escs contains the number of escs allocated for each supported type:
    uint8_t num_escs_for_protocol[uint8_t(AP_ESC_Telem_Protocol::__LAST__)];
    uint8_t num_escs;  // sum of all the values in num_escs_for_protocol
    class AP_HobbyWing_ESC *escs[ESC_TELEM_MAX_ESCS];
    void thread_main(void);
#endif

#if AP_HOBBYWING_DATALINK_ENABLED
    class AP_HobbyWing_DataLink *datalink;
#endif  // AP_HOBBYWING_DATALINK_ENABLED

    static class AP_HobbyWing_ESC *new_esc_backend_for_type(AP_ESC_Telem_Protocol protocol, AP_HAL::UARTDriver &uart, uint8_t servo_channel, uint8_t poles);
    AP_SerialManager::SerialProtocol serial_protocol_for_esc_telem_protocol(AP_ESC_Telem_Protocol protocol);

    static AP_ESC_Telem *_singleton;
};

namespace AP {
    AP_ESC_Telem &esc_telem();
};

#endif // HAL_WITH_ESC_TELEM
