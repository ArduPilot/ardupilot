#pragma once

#include <AP_Param/AP_Param.h>

class AP_BattMonitor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_BattMonitor_Params(void);

    /* Do not allow copies */
    AP_BattMonitor_Params(const AP_BattMonitor_Params &other) = delete;
    AP_BattMonitor_Params &operator=(const AP_BattMonitor_Params&) = delete;

    // Battery monitor driver types
    enum BattMonitor_Type {
        BattMonitor_TYPE_NONE                       = 0,
        BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY        = 3,
        BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT = 4,
        BattMonitor_TYPE_SOLO                       = 5,
        BattMonitor_TYPE_BEBOP                      = 6,
        BattMonitor_TYPE_MAXELL                     = 7,
        BattMonitor_TYPE_UAVCAN_BatteryInfo         = 8,
        BattMonitor_TYPE_BLHeliESC                  = 9
    };

    // low voltage sources (used for BATT_LOW_TYPE parameter)
    enum BattMonitor_LowVoltage_Source {
        BattMonitor_LowVoltageSource_Raw            = 0,
        BattMonitor_LowVoltageSource_SagCompensated = 1
    };

    BattMonitor_Type type(void) const { return (enum BattMonitor_Type)_type.get(); }
    BattMonitor_LowVoltage_Source failsafe_voltage_source(void) { return (enum BattMonitor_LowVoltage_Source)_failsafe_voltage_source.get(); }

    AP_Int8  _type;                     /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8  _volt_pin;                 /// board pin used to measure battery voltage
    AP_Int8  _curr_pin;                 /// board pin used to measure battery current
    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _curr_amp_per_volt;        /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float _curr_amp_offset;          /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Int32 _pack_capacity;            /// battery pack capacity less reserve in mAh
    AP_Int16 _watt_max;                 /// max battery power allowed. Reduce max throttle to reduce current to satisfy t    his limit
    AP_Int32 _serial_number;            /// battery serial number, automatically filled in on SMBus batteries
    AP_Int8  _low_voltage_timeout;      /// timeout in seconds before a low voltage event will be triggered
    AP_Int8  _failsafe_voltage_source;  /// voltage type used for detection of low voltage event
    AP_Float _low_voltage;              /// voltage level used to trigger a low battery failsafe
    AP_Float _low_capacity;             /// capacity level used to trigger a low battery failsafe
    AP_Float _critical_voltage;         /// voltage level used to trigger a critical battery failsafe
    AP_Float _critical_capacity;        /// capacity level used to trigger a critical battery failsafe
    AP_Int8  _failsafe_low_action;      /// action to preform on a low battery failsafe
    AP_Int8  _failsafe_critical_action; /// action to preform on a critical battery failsafe

};
