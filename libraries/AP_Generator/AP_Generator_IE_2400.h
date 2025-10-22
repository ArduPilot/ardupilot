#pragma once

#include "AP_Generator_IE_FuelCell.h"

#if AP_GENERATOR_IE_2400_ENABLED

#include <AP_Logger/AP_Logger_config.h>

class AP_Generator_IE_2400 : public AP_Generator_IE_FuelCell
{
    // Inherit constructor
    using AP_Generator_IE_FuelCell::AP_Generator_IE_FuelCell;

public:

    __INITFUNC__ void init(void) override;

    AP_BattMonitor::Failsafe update_failsafes() const override;

private:

    // Assigns the unit specific measurements once a valid sentence is obtained
    void assign_measurements(const uint32_t now) override;

    // Process characters received and extract terms for IE 2.4kW
    void decode_latest_term(void) override;

    // Decode a data packet
    void decode_data_packet();
    void decode_legacy_data();
    void decode_v2_data();

    // Decode a info packet
    void decode_info_packet();

    // Check if we have received an error code and populate message with error code
    bool check_for_err_code(char* msg_txt, uint8_t msg_len) const override;

    // Check if we have received an warning code and populate message with warning code
    bool check_for_warning_code(char* msg_txt, uint8_t msg_len) const override;

#if HAL_GCS_ENABLED
    // Get the MAV_SEVERITY level of a given error code
    MAV_SEVERITY get_mav_severity(uint32_t err_code) const override;
#endif

    // Check if we should notify on any change of fuel cell state
    void check_status(const uint32_t now) override;

    // Check for error codes that are deemed critical
    bool is_critical_error(const uint32_t err_in) const;

    // Check for error codes that are deemed severe and would be cause to trigger a battery monitor low failsafe action
    bool is_low_error(const uint32_t err_in) const;

#if HAL_LOGGING_ENABLED
    void log_write(void) override;
#endif

    // Return true is fuel cell is in running state suitable for arming
    bool is_running() const override;

    // Print msg to user updating on state change
    void update_state_msg() override;

    // IE 2.4kW failsafes
    enum class ErrorCode {
        MINOR_INTERNAL = 1, // Minor internal error is to be ignored
        REDUCED_POWER = 10,
        SPM_LOST = 11,
        PRESSURE_LOW = 20,
        BATTERY_LOW = 21,
        PRESSURE_ALERT = 30,
        START_DENIED = 31,
        SYSTEM_CRITICAL = 32,
        PRESSURE_CRITICAL = 33,
        BATTERY_CRITICAL = 40,
    };

    // These measurements are only available on this unit
    int16_t _pwr_out;  // Output power (Watts)
    uint16_t _spm_pwr; // Stack Power Module (SPM) power draw (Watts)
    float _fuel_rem; // fuel remaining 0 to 1
    int16_t _battery_pwr; // Battery charging power

    // Extra data in the V2 packet
    struct V2_data {
        float inlet_press;
        uint8_t unit_fault; // Unit number with issue
        char info_str[33];
    };
    V2_data _parsed_V2;
    V2_data _valid_V2;

    // Info packet
    struct {
        char PCM_number[TERM_BUFFER];
        char Software_version[TERM_BUFFER];
        char Protocol_version[TERM_BUFFER];
        char Serial_number[TERM_BUFFER];
    } _info;
    bool _had_info;

    enum class ProtocolVersion {
        DETECTING = 0,
        LEGACY = 1,
        V2 = 2,
        UNKNOWN = 3,
    } _version;

    ProtocolVersion _last_version;
    uint8_t _last_version_packet_count;

    enum class PacketType {
        NONE = 0,
        LEGACY_DATA = 1,
        V2_DATA = 2,
        V2_INFO = 3,
    } _type;

    enum class V2_State {
        FCPM_Off = 0,
        Starting = 1,
        Running = 2,
        Stopping = 3,
        Go_to_Sleep = 4,
    } _v2_state;
    V2_State _last_v2_state;

    // State enum to string lookup
    struct Lookup_State_V2 {
        V2_State option;
        const char *msg_txt;
    };
    static const Lookup_State_V2 lookup_state_V2[];

    uint32_t _last_low_power_warning_ms;

};
#endif  // AP_GENERATOR_IE_2400_ENABLED

