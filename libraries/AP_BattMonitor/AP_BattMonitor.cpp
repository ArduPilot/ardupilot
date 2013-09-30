/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_BattMonitor.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor::var_info[] PROGMEM = {
    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
    AP_GROUPINFO("MONITOR", 0, AP_BattMonitor, _monitoring, AP_BATT_MONITOR_DISABLED),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:Pixhawk, 13:A13, 100:PX4
    // @User: Standard
    AP_GROUPINFO("VOLT_PIN", 1, AP_BattMonitor, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
    // @Values: -1:Disabled, 1:A1, 2:A2, 3:Pixhawk, 12:A12, 101:PX4
    // @User: Standard
    AP_GROUPINFO("CURR_PIN", 2, AP_BattMonitor, _curr_pin, AP_BATT_CURR_PIN),

    // @Param: BATT_VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the PX4 using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 3, AP_BattMonitor, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: BATT_APM_PERVOLT
    // @DisplayName: Apms per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVOLT", 4, AP_BattMonitor, _curr_amp_per_volt, AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: BATT_AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: Volts
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 5, AP_BattMonitor, _curr_amp_offset, 0),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @User: Standard
    AP_GROUPINFO("CAPACITY", 6, AP_BattMonitor, _pack_capacity, AP_BATT_CAPACITY_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_BattMonitor::AP_BattMonitor(void) :
    _voltage(AP_BATT_INITIAL_VOLTAGE)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// init - setup the battery and voltage pins
void
AP_BattMonitor::init()
{
    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
    _curr_pin_analog_source = hal.analogin->channel(_curr_pin);
}

// read - read the voltage and current
void
AP_BattMonitor::read()
{
    if (_monitoring == AP_BATT_MONITOR_DISABLED) {
        _voltage = 0;
        return;
    }

    // read voltage
    if (_monitoring == AP_BATT_MONITOR_VOLTAGE_ONLY || _monitoring == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        // this copes with changing the pin at runtime
        _volt_pin_analog_source->set_pin(_volt_pin);
        _voltage = _volt_pin_analog_source->voltage_average() * _volt_multiplier;
    }

    // read current
    if (_monitoring == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        uint32_t tnow = hal.scheduler->millis();
        float dt = tnow - _last_time_ms;
        // this copes with changing the pin at runtime
        _curr_pin_analog_source->set_pin(_curr_pin);
        _current_amps = (_curr_pin_analog_source->voltage_average()-_curr_amp_offset)*_curr_amp_per_volt;
        if (_last_time_ms != 0 && dt < 2000) {
            // .0002778 is 1/3600 (conversion to hours)
            _current_total_mah += _current_amps * dt * 0.0002778f;
        }
        _last_time_ms = tnow;
    }
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor::capacity_remaining_pct() const
{
    return (100.0 * (_pack_capacity - _current_total_mah) / _pack_capacity);
}
