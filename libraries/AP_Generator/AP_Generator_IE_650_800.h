#pragma once

#include "AP_Generator_IE_FuelCell.h"

#if AP_GENERATOR_IE_650_800_ENABLED

class AP_Generator_IE_650_800 : public AP_Generator_IE_FuelCell
{
    // Inherit constructor
    using AP_Generator_IE_FuelCell::AP_Generator_IE_FuelCell;

public:

    void init(void) override;

    AP_BattMonitor::Failsafe update_failsafes() const override;

private:

    // Assigns the unit specific measurements once a valid sentence is obtained
    void assign_measurements(const uint32_t now) override;

    // Process characters received and extract terms for IE 650/800 W
    void decode_latest_term(void) override;

    // Convert error code from fuel cell to AP failsafe bitmask
    void set_failsafe_mask(uint32_t err_in);

    // Check if we have received an error code and populate message with error code
    bool check_for_err_code(char* msg_txt, uint8_t msg_len) const override;

    // IE 650/800 W error codes
    // Note: The error codes for this unit are defined as a bitmask
    static const uint32_t ERROR_STACK_OT1_ALRT    = (1U << 31); // (0x80000000) Stack 1 over temperature alert (>58 degC)
    static const uint32_t ERROR_STACK_OT2_ALRT    = (1U << 30); // (0x40000000) Stack 2 over temperature alert (>58 degC)
    static const uint32_t ERROR_BAT_UV_ALRT       = (1U << 29); // (0x20000000) Battery under volt alert (<19 V)
    static const uint32_t ERROR_BAT_OT_ALRT       = (1U << 28); // (0x10000000) Battery over temperature alert (>65 degC)
    static const uint32_t ERROR_NO_FAN            = (1U << 27); // (0x8000000), No fan current detected (<0.01 A)
    static const uint32_t ERROR_FAN_OVERRUN       = (1U << 26); // (0x4000000), Fan over current (>0.25 A)
    static const uint32_t ERROR_STACK_OT1_CRT     = (1U << 25); // (0x2000000), Stack 1 over temperature critical (>57 degC)
    static const uint32_t ERROR_STACK_OT2_CRT     = (1U << 24); // (0x1000000), Stack 2 over temperature critical (>57 degC)
    static const uint32_t ERROR_BAT_UV_CRT        = (1U << 23); // (0x800000), Battery under volt warning (<19.6 V)
    static const uint32_t ERROR_BAT_OT_CRT        = (1U << 22); // (0x400000), Battery over temperature warning (>60 degC)
    static const uint32_t ERROR_START_TIMEOUT     = (1U << 21); // (0x200000), Fuel cell's internal State == start for > 30 s
    static const uint32_t ERROR_STOP_TIMEOUT      = (1U << 20); // (0x100000), Fuel cell's internal State == stop for > 15 s
    static const uint32_t ERROR_START_UNDER_PRESS = (1U << 19); // (0x80000), Tank pressure < 6 barg
    static const uint32_t ERROR_TANK_UNDER_PRESS  = (1U << 18); // (0x40000), Tank pressure < 5 barg
    static const uint32_t ERROR_TANK_LOW_PRESS    = (1U << 17); // (0x20000), Tank pressure < 15 barg
    static const uint32_t ERROR_SAFETY_FLAG       = (1U << 16); // (0x10000), Fuel cell's internal saftey flags not set true
    static const uint32_t ERROR_DENY_START1       = (1U << 15); // (0x8000), Stack 1 denied start
    static const uint32_t ERROR_DENY_START2       = (1U << 14); // (0x4000), Stack 2 denied start
    static const uint32_t ERROR_STACK_UT1         = (1U << 13); // (0x2000), Stack 1 under temperature (<5 degC)
    static const uint32_t ERROR_STACK_UT2         = (1U << 12); // (0x1000), Stack 2 under temperature (<5 degC)
    static const uint32_t ERROR_BAT_UV_WRN        = (1U << 11); // (0x800), Battery under voltage warning (21.6 V)
    static const uint32_t ERROR_BAT_UV_DENY       = (1U << 10); // (0x400), Battery under voltage (21.6 V) and master denied
    static const uint32_t ERROR_FAN_PULSE         = (1U << 9);  // (0x200), Fan pulse aborted
    static const uint32_t ERROR_STACK_UV          = (1U << 8);  // (0x100), Stack under voltage (650W < 17.4 V, 800W < 21.13 V)
    static const uint32_t ERROR_SYS_OVERLOAD      = (1U << 7);  // (0x80), Stack under voltage and battery power below threshold (<-200 W)
    static const uint32_t ERROR_SYS_OV_OC         = (1U << 6);  // (0x40), Over voltage and over current protection
    static const uint32_t ERROR_INVALID_SN        = (1U << 5);  // (0x20), Invalid serial number
    static const uint32_t ERROR_CHARGE_FAULT      = (1U << 4);  // (0x10), Battery charger fault
    static const uint32_t ERROR_BAT_UT            = (1U << 3);  // (0x8), Battery under temperature (<-15 degC)

    // Assign error codes to critical FS mask
    static const uint32_t fs_crit_mask = ERROR_STACK_OT1_ALRT // (0x80000000) Stack 1 over temperature alert (>58 degC)
                                | ERROR_STACK_OT2_ALRT        // (0x40000000) Stack 2 over temperature alert (>58 degC)
                                | ERROR_BAT_UV_ALRT           // (0x20000000) Battery undervolt alert (<19 V)
                                | ERROR_BAT_OT_ALRT           // (0x10000000) Battery over temperature alert (>65 degC)
                                | ERROR_NO_FAN                // (0x8000000), No fan current detected (<0.01 A)
                                | ERROR_STACK_OT1_CRT         // (0x2000000), Stack 1 over temperature critical (>57 degC)
                                | ERROR_STACK_OT2_CRT         // (0x1000000), Stack 2 over temperature critical (>57 degC)
                                | ERROR_BAT_UV_CRT            // (0x800000), Battery undervolt warning (<19.6 V)
                                | ERROR_BAT_OT_CRT            // (0x400000), Battery over temperature warning (>60 degC)
                                | ERROR_START_TIMEOUT         // (0x200000), Fuel cell's internal State == start for > 30 s
                                | ERROR_START_UNDER_PRESS     // (0x80000), Tank pressure < 6 barg
                                | ERROR_TANK_UNDER_PRESS      // (0x40000), Tank pressure < 5 barg
                                | ERROR_SAFETY_FLAG           // (0x10000), Fuel cell's internal saftey flags not set true
                                | ERROR_DENY_START1           // (0x8000), Stack 1 denied start
                                | ERROR_DENY_START2           // (0x4000), Stack 2 denied start
                                | ERROR_BAT_UV_DENY           // (0x400), Battery under voltage (21.6 V) and master denied
                                | ERROR_SYS_OV_OC             // (0x40), Over voltage and over current protection
                                | ERROR_INVALID_SN;           // (0x20), Invalid serial number

    // Assign error codes to low FS mask
    static const uint32_t fs_low_mask = ERROR_FAN_OVERRUN     // (0x4000000), Fan over current (>0.25 A)
                                | ERROR_STOP_TIMEOUT          // (0x100000), Fuel cell's internal State == stop for > 15 s
                                | ERROR_TANK_LOW_PRESS        // (0x20000), Tank pressure < 15 barg
                                | ERROR_STACK_UT1             // (0x2000), Stack 1 under temperature (<5 degC)
                                | ERROR_STACK_UT2             // (0x1000), Stack 2 under temperature (<5 degC)
                                | ERROR_BAT_UV_WRN            // (0x800), Battery under voltage warning (21.6 V)
                                | ERROR_FAN_PULSE             // (0x200), Fan pulse aborted
                                | ERROR_STACK_UV              // (0x100), Stack under voltage (650W < 17.4 V, 800W < 21.13 V)
                                | ERROR_SYS_OVERLOAD          // (0x80), Stack under voltage and battery power below threshold (<-200 W)
                                | ERROR_CHARGE_FAULT          // (0x10), Battery charger fault
                                | ERROR_BAT_UT;               // (0x8), Battery undertemperature (<-15 degC)

};
#endif  // AP_GENERATOR_IE_650_800_ENABLED
