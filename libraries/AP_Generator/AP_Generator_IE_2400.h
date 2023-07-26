#pragma once

#include "AP_Generator_IE_FuelCell.h"

#if AP_GENERATOR_IE2400_ENABLED

class AP_Generator_IE_2400 : public AP_Generator_IE_FuelCell
{
    // Inherit constructor
    using AP_Generator_IE_FuelCell::AP_Generator_IE_FuelCell;

public:

    void init(void) override;

    AP_BattMonitor::Failsafe update_failsafes() const override;

private:

    // Assigns the unit specific measurements once a valid sentence is obtained
    void assign_measurements(const uint32_t now) override;

    // Process characters received and extract terms for IE 2.4kW
    void decode_latest_term(void) override;

    // Check if we have received an error code and populate message with error code
    bool check_for_err_code(char* msg_txt, uint8_t msg_len) const override;

    // Check for error codes that are deemed critical
    bool is_critical_error(const uint32_t err_in) const;

    // Check for error codes that are deemed severe and would be cause to trigger a battery monitor low failsafe action
    bool is_low_error(const uint32_t err_in) const;

    void log_write(void) override;

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

};
#endif  // AP_GENERATOR_IE2400_ENABLED

