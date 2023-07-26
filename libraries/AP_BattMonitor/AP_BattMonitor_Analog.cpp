#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ANALOG_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "AP_BattMonitor_Analog.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_Analog::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the analog input pin that should be used for voltage monitoring.
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 1, AP_BattMonitor_Analog, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for current monitoring.
    // @Values: -1:Disabled, 3:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 4:CubeOrange_PM2/Navigator, 14:Pixhawk2_PM2, 15:CubeOrange, 17:Durandal, 101:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 2, AP_BattMonitor_Analog, _curr_pin, AP_BATT_CURR_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 3, AP_BattMonitor_Analog, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 4, AP_BattMonitor_Analog, _curr_amp_per_volt, AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 5, AP_BattMonitor_Analog, _curr_amp_offset, AP_BATT_CURR_AMP_OFFSET_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Voltage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 6, AP_BattMonitor_Analog, _volt_offset, 0),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);

    // no other good way of setting these defaults
#if AP_BATT_MONITOR_MAX_INSTANCES > 1
    if (mon_state.instance == 1) {
#ifdef HAL_BATT2_VOLT_PIN
        _volt_pin.set_default(HAL_BATT2_VOLT_PIN);
#endif
#ifdef HAL_BATT2_CURR_PIN
        _curr_pin.set_default(HAL_BATT2_CURR_PIN);
#endif
#ifdef HAL_BATT2_VOLT_SCALE
        _volt_multiplier.set_default(HAL_BATT2_VOLT_SCALE);
#endif
#ifdef HAL_BATT2_CURR_SCALE
        _curr_amp_per_volt.set_default(HAL_BATT2_CURR_SCALE);
#endif
    }
#endif
    _state.var_info = var_info;
    
    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
    _curr_pin_analog_source = hal.analogin->channel(_curr_pin);

}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    // this copes with changing the pin at runtime
    _state.healthy = _volt_pin_analog_source->set_pin(_volt_pin);

    // get voltage
    _state.voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    // read current
    if (has_current()) {
        // calculate time since last current read
        const uint32_t tnow = AP_HAL::micros();
        const uint32_t dt_us = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _state.healthy &= _curr_pin_analog_source->set_pin(_curr_pin);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average() - _curr_amp_offset) * _curr_amp_per_volt;

        update_consumed(_state, dt_us);

        // record time
        _state.last_time_micros = tnow;
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog::has_current() const
{
    return ((AP_BattMonitor::Type)_params._type.get() == AP_BattMonitor::Type::ANALOG_VOLTAGE_AND_CURRENT);
}

#endif  // AP_BATTERY_ANALOG_ENABLED
