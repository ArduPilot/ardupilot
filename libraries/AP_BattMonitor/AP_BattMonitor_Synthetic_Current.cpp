#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SYNTHETIC_CURRENT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Synthetic_Current.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

/*
  Analog Voltage and Current Monitor for systems with only a voltage sense pin
  Current is calculated from throttle output and modified for voltage droop using
  a square law calculation
 */
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_Synthetic_Current::var_info[] = {

    // @Param: MAX_VOLT
    // @DisplayName: Maximum Battery Voltage
    // @Description: Maximum voltage of battery. Provides scaling of current versus voltage
    // @Range: 7 100
    // @User: Advanced

    AP_GROUPINFO("MAX_VOLT", 50, AP_BattMonitor_Synthetic_Current, _max_voltage, 12.6),
    
    // also inherit analog backend parameters
    AP_SUBGROUPEXTENSION("", 51, AP_BattMonitor_Synthetic_Current, AP_BattMonitor_Analog::var_info),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_Synthetic_Current::AP_BattMonitor_Synthetic_Current(AP_BattMonitor &mon,
                                                 AP_BattMonitor::BattMonitor_State &mon_state,
                                                 AP_BattMonitor_Params &params) :
    AP_BattMonitor_Analog(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
    
    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
}

// read - read the voltage and current
void
AP_BattMonitor_Synthetic_Current::read()
{
    // this copes with changing the pin at runtime
    _state.healthy = _volt_pin_analog_source->set_pin(_volt_pin);

    // get voltage
    _state.voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    // read current
    // calculate time since last current read
    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;

    // read current
    _state.current_amps = ((_state.voltage/_max_voltage)*sq(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)) * 0.0001 * _curr_amp_per_volt) + _curr_amp_offset ;

    update_consumed(_state, dt_us);

    // record time
    _state.last_time_micros = tnow;
 
}

#endif  // AP_BATTERY_SYNTHETIC_CURRENT_ENABLED
