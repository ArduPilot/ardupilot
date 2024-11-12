#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_BattMonitor_config.h"

class AP_BattMonitor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_BattMonitor_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_BattMonitor_Params);

    // low voltage sources (used for BATT_LOW_TYPE parameter)
    enum BattMonitor_LowVoltage_Source {
        BattMonitor_LowVoltageSource_Raw            = 0,
        BattMonitor_LowVoltageSource_SagCompensated = 1
    };
    enum class Options : uint16_t {
        Ignore_UAVCAN_SoC                   = (1U<<0),  // Ignore UAVCAN State-of-Charge (charge %) supplied value from the device and use the internally calculated one
        MPPT_Use_Input_Value                = (1U<<1),  // MPPT reports voltage and current from Input (usually solar panel) instead of the output
        MPPT_Power_Off_At_Disarm            = (1U<<2),  // MPPT Disabled when vehicle is disarmed, if HW supports it
        MPPT_Power_On_At_Arm                = (1U<<3),  // MPPT Enabled when vehicle is armed, if HW supports it
        MPPT_Power_Off_At_Boot              = (1U<<4),  // MPPT Disabled at startup (aka boot), if HW supports it
        MPPT_Power_On_At_Boot               = (1U<<5),  // MPPT Enabled at startup (aka boot), if HW supports it. If Power_Off_at_Boot is also set, the behavior is Power_Off_at_Boot
        GCS_Resting_Voltage                 = (1U<<6),  // send resistance resting voltage to GCS
        AllowSplitAuxInfo                   = (1U<<7),  // allow different node to provide aux info for DroneCAN
        InternalUseOnly                     = (1U<<8),  // for use internally to ArduPilot, not to be (eg.) sent via MAVLink BATTERY_STATUS
        Minimum_Voltage                     = (1U<<9),  // sum monitor measures minimum voltage rather than average
    };

    BattMonitor_LowVoltage_Source failsafe_voltage_source(void) const { return (enum BattMonitor_LowVoltage_Source)_failsafe_voltage_source.get(); }

    AP_Int32 _pack_capacity;            /// battery pack capacity less reserve in mAh
    AP_Int32 _serial_number;            /// battery serial number, automatically filled in on SMBus batteries
    AP_Float _low_voltage;              /// voltage level used to trigger a low battery failsafe
    AP_Float _low_capacity;             /// capacity level used to trigger a low battery failsafe
    AP_Float _critical_voltage;         /// voltage level used to trigger a critical battery failsafe
    AP_Float _critical_capacity;        /// capacity level used to trigger a critical battery failsafe
    AP_Int32 _arming_minimum_capacity;  /// capacity level required to arm
    AP_Float _arming_minimum_voltage;   /// voltage level required to arm
    AP_Int32 _options;                  /// Options
#if AP_BATTERY_WATT_MAX_ENABLED
    AP_Int16 _watt_max;                 /// max battery power allowed. Reduce max throttle to reduce current to satisfy t    his limit
#endif
    AP_Int8  _type;                     /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8  _low_voltage_timeout;      /// timeout in seconds before a low voltage event will be triggered
    AP_Int8  _failsafe_voltage_source;  /// voltage type used for detection of low voltage event
    AP_Int8  _failsafe_low_action;      /// action to preform on a low battery failsafe
    AP_Int8  _failsafe_critical_action; /// action to preform on a critical battery failsafe
#if AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
    AP_Int8  _esc_telem_outbound_index; /// bitmask of ESCs to forward voltage, current, consumption and temperature to.
#endif
};
