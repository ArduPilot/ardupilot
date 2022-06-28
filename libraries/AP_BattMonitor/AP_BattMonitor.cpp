#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor_SMBus.h"
#include "AP_BattMonitor_SMBus_Solo.h"
#include "AP_BattMonitor_SMBus_Generic.h"
#include "AP_BattMonitor_SMBus_Maxell.h"
#include "AP_BattMonitor_SMBus_Rotoye.h"
#include "AP_BattMonitor_Bebop.h"
#include "AP_BattMonitor_ESC.h"
#include "AP_BattMonitor_SMBus_SUI.h"
#include "AP_BattMonitor_SMBus_NeoDesign.h"
#include "AP_BattMonitor_Sum.h"
#include "AP_BattMonitor_FuelFlow.h"
#include "AP_BattMonitor_FuelLevel_PWM.h"
#include "AP_BattMonitor_Generator.h"
#include "AP_BattMonitor_INA2xx.h"
#include "AP_BattMonitor_LTC2946.h"
#include "AP_BattMonitor_Torqeedo.h"
#include "AP_BattMonitor_FuelLevel_Analog.h"

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_BattMonitor_UAVCAN.h"
#endif

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL& hal;

AP_BattMonitor *AP_BattMonitor::_singleton;

const AP_Param::GroupInfo AP_BattMonitor::var_info[] = {
    // 0 - 18, 20- 22 used by old parameter indexes

    // Monitor 1

    // @Group: _
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "_", 23, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: _
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: _
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: _
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: _
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: _
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[0], "_", 41, AP_BattMonitor, backend_var_info[0]),

#if AP_BATT_MONITOR_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 24, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 2_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 2_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 2_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 2_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 2_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_", 42, AP_BattMonitor, backend_var_info[1]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 25, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 3_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 3_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 3_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 3_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 3_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_", 43, AP_BattMonitor, backend_var_info[2]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 26, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 4_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 4_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 4_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 4_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 4_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_", 44, AP_BattMonitor, backend_var_info[3]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 27, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 5_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 5_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 5_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 5_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 5_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_", 45, AP_BattMonitor, backend_var_info[4]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 28, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 6_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 6_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 6_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 6_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 6_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_", 46, AP_BattMonitor, backend_var_info[5]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 29, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 7_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 7_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 7_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 7_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 7_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_", 47, AP_BattMonitor, backend_var_info[6]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 30, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 8_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 8_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 8_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 8_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 8_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_", 48, AP_BattMonitor, backend_var_info[7]),
#endif

#if AP_BATT_MONITOR_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 31, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 9_
    // @Path: AP_BattMonitor_Analog.cpp
    // @Group: 9_
    // @Path: AP_BattMonitor_SMBus.cpp
    // @Group: 9_
    // @Path: AP_BattMonitor_Sum.cpp
    // @Group: 9_
    // @Path: AP_BattMonitor_UAVCAN.cpp
    // @Group: 9_
    // @Path: AP_BattMonitor_FuelLevel_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_", 49, AP_BattMonitor, backend_var_info[8]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *AP_BattMonitor::backend_var_info[AP_BATT_MONITOR_MAX_INSTANCES];

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_BattMonitor::AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities) :
    _log_battery_bit(log_battery_bit),
    _battery_failsafe_handler_fn(battery_failsafe_handler_fn),
    _failsafe_priorities(failsafe_priorities)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BattMonitor must be singleton");
    }
    _singleton = this;
}

// init - instantiate the battery monitors
void
AP_BattMonitor::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    _highest_failsafe_priority = INT8_MAX;

#ifdef HAL_BATT_MONITOR_DEFAULT
    _params[0]._type.set_default(int8_t(HAL_BATT_MONITOR_DEFAULT));
#endif

    // create each instance
    for (uint8_t instance=0; instance<AP_BATT_MONITOR_MAX_INSTANCES; instance++) {
        // clear out the cell voltages
        memset(&state[instance].cell_voltages, 0xFF, sizeof(cells));

        switch (get_type(instance)) {
            case Type::ANALOG_VOLTAGE_ONLY:
            case Type::ANALOG_VOLTAGE_AND_CURRENT:
                drivers[instance] = new AP_BattMonitor_Analog(*this, state[instance], _params[instance]);
                break;
#if AP_BATTMON_SMBUS_ENABLE
            case Type::SOLO:
                drivers[instance] = new AP_BattMonitor_SMBus_Solo(*this, state[instance], _params[instance]);
                break;
            case Type::SMBus_Generic:
                drivers[instance] = new AP_BattMonitor_SMBus_Generic(*this, state[instance], _params[instance]);
                break;
            case Type::SUI3:
                drivers[instance] = new AP_BattMonitor_SMBus_SUI(*this, state[instance], _params[instance], 3);
                break;
            case Type::SUI6:
                drivers[instance] = new AP_BattMonitor_SMBus_SUI(*this, state[instance], _params[instance], 6);
                break;
            case Type::MAXELL:
                drivers[instance] = new AP_BattMonitor_SMBus_Maxell(*this, state[instance], _params[instance]);
                break;
            case Type::Rotoye:
                drivers[instance] = new AP_BattMonitor_SMBus_Rotoye(*this, state[instance], _params[instance]);
                break;
            case Type::NeoDesign:
                drivers[instance] = new AP_BattMonitor_SMBus_NeoDesign(*this, state[instance], _params[instance]);
                break;
#endif // AP_BATTMON_SMBUS_ENABLE
            case Type::BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
                drivers[instance] = new AP_BattMonitor_Bebop(*this, state[instance], _params[instance]);
#endif
                break;
            case Type::UAVCAN_BatteryInfo:
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
                drivers[instance] = new AP_BattMonitor_UAVCAN(*this, state[instance], AP_BattMonitor_UAVCAN::UAVCAN_BATTERY_INFO, _params[instance]);
#endif
                break;
            case Type::BLHeliESC:
#if HAL_WITH_ESC_TELEM && !defined(HAL_BUILD_AP_PERIPH)
                drivers[instance] = new AP_BattMonitor_ESC(*this, state[instance], _params[instance]);
#endif
                break;
            case Type::Sum:
                drivers[instance] = new AP_BattMonitor_Sum(*this, state[instance], _params[instance], instance);
                break;
#if AP_BATTMON_FUELFLOW_ENABLE
            case Type::FuelFlow:
                drivers[instance] = new AP_BattMonitor_FuelFlow(*this, state[instance], _params[instance]);
                break;
#endif // AP_BATTMON_FUELFLOW_ENABLE
#if AP_BATTMON_FUELLEVEL_PWM_ENABLE
            case Type::FuelLevel_PWM:
                drivers[instance] = new AP_BattMonitor_FuelLevel_PWM(*this, state[instance], _params[instance]);
                break;
#endif // AP_BATTMON_FUELLEVEL_PWM_ENABLE
#if AP_BATTMON_FUELLEVEL_ANALOG_ENABLE
            case Type::FuelLevel_Analog:
                drivers[instance] = new AP_BattMonitor_FuelLevel_Analog(*this, state[instance], _params[instance]);
                break;
#endif // AP_BATTMON_FUELLEVEL_ANALOG_ENABLE
#if HAL_GENERATOR_ENABLED
            case Type::GENERATOR_ELEC:
                drivers[instance] = new AP_BattMonitor_Generator_Elec(*this, state[instance], _params[instance]);
                break;
            case Type::GENERATOR_FUEL:
                drivers[instance] = new AP_BattMonitor_Generator_FuelLevel(*this, state[instance], _params[instance]);
                break;
#endif // HAL_GENERATOR_ENABLED
#if HAL_BATTMON_INA2XX_ENABLED
            case Type::INA2XX:
                drivers[instance] = new AP_BattMonitor_INA2XX(*this, state[instance], _params[instance]);
                break;
#endif
#if HAL_BATTMON_LTC2946_ENABLED
            case Type::LTC2946:
                drivers[instance] = new AP_BattMonitor_LTC2946(*this, state[instance], _params[instance]);
                break;
#endif
#if HAL_TORQEEDO_ENABLED
            case Type::Torqeedo:
                drivers[instance] = new AP_BattMonitor_Torqeedo(*this, state[instance], _params[instance]);
                break;
#endif
            case Type::NONE:
            default:
                break;
        }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            drivers[instance]->init();
            // _num_instances is actually the index for looping over instances
            // the user may have BATT_MONITOR=0 and BATT2_MONITOR=7, in which case
            // there will be a gap, but as we always check for drivers[instances] being nullptr
            // this is safe
            _num_instances = instance + 1;

            // Convert the old analog & Bus parameters to the new dynamic parameter groups
            convert_dynamic_param_groups(instance);
        }
    }
}

void AP_BattMonitor::convert_dynamic_param_groups(uint8_t instance)
{
    AP_Param::ConversionInfo info;
    if (!AP_Param::find_top_level_key_by_pointer(this, info.old_key)) {
        return;
    }

    char param_prefix[6] {};
    char param_name[17] {};
    info.new_name = param_name;

    const uint8_t param_instance = instance + 1;
    // first battmonitor does not have '1' in the param name
    if(param_instance == 1) {
        hal.util->snprintf(param_prefix, sizeof(param_prefix), "BATT");
    } else {
        hal.util->snprintf(param_prefix, sizeof(param_prefix), "BATT%X", param_instance);
    }
    param_prefix[sizeof(param_prefix)-1] = '\0';

    hal.util->snprintf(param_name, sizeof(param_name), "%s_%s", param_prefix, "MONITOR");
    param_name[sizeof(param_name)-1] = '\0';

    // Find the index of the BATTn_MONITOR which is not moving to index the moving parameters off from
    AP_Param::ParamToken token = AP_Param::ParamToken {};
    ap_var_type type;
    AP_Param* param = AP_Param::find_by_name(param_name, &type, &token);
    const uint8_t battmonitor_index = 1;
    if( param == nullptr) {
        // BATTn_MONITOR not found
        return;
    }

    const struct convert_table {
        uint32_t old_group_element;
        ap_var_type type;
        const char* new_name;
    }  conversion_table[] = {
        // PARAMETER_CONVERSION - Added: Aug-2021
            { 2,  AP_PARAM_INT8,  "VOLT_PIN"  },
            { 3,  AP_PARAM_INT8,  "CURR_PIN"  },
            { 4,  AP_PARAM_FLOAT, "VOLT_MULT" },
            { 5,  AP_PARAM_FLOAT, "AMP_PERVLT"},
            { 6,  AP_PARAM_FLOAT, "AMP_OFFSET"},
            { 20, AP_PARAM_INT8,  "I2C_BUS"   },
        };

    for (const auto & elem : conversion_table) {
        info.old_group_element = token.group_element + ((elem.old_group_element - battmonitor_index) * 64);
        info.type = elem.type;

        hal.util->snprintf(param_name, sizeof(param_name), "%s_%s", param_prefix, elem.new_name);
        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }
}

// read - For all active instances read voltage & current; log BAT, BCL, POWR
void AP_BattMonitor::read()
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr && logger->should_log(_log_battery_bit)) {
        logger->Write_Power();
    }
#endif

    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != nullptr && get_type(i) != Type::NONE) {
            drivers[i]->read();
            drivers[i]->update_resistance_estimate();

#if HAL_LOGGING_ENABLED
            if (logger != nullptr && logger->should_log(_log_battery_bit)) {
                const uint64_t time_us = AP_HAL::micros64();
                drivers[i]->Log_Write_BAT(i, time_us);
                drivers[i]->Log_Write_BCL(i, time_us);
            }
#endif
        }
    }

    check_failsafes();
    
    checkPoweringOff();
}

// healthy - returns true if monitor is functioning
bool AP_BattMonitor::healthy(uint8_t instance) const {
    return instance < _num_instances && state[instance].healthy;
}

/// voltage - returns battery voltage in volts
float AP_BattMonitor::voltage(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].voltage;
    } else {
        return 0.0f;
    }
}

/// get voltage with sag removed (based on battery current draw and resistance)
/// this will always be greater than or equal to the raw voltage
float AP_BattMonitor::voltage_resting_estimate(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr) {
        return drivers[instance]->voltage_resting_estimate();
    } else {
        return 0.0f;
    }
}

/// voltage - returns battery voltage in volts for GCS, may be resting voltage if option enabled
float AP_BattMonitor::gcs_voltage(uint8_t instance) const
{
    if ((_params[instance]._options.get() & uint32_t(AP_BattMonitor_Params::Options::GCS_Resting_Voltage)) != 0) {
        return voltage_resting_estimate(instance);
    }
    if (instance < _num_instances) {
        return state[instance].voltage;
    } else {
        return 0.0f;
    }
}

/// current_amps - returns the instantaneous current draw in amperes
bool AP_BattMonitor::current_amps(float &current, uint8_t instance) const {
    if ((instance < _num_instances) && (drivers[instance] != nullptr) && drivers[instance]->has_current()) {
        current = state[instance].current_amps;
        return true;
    } else {
        return false;
    }
}

/// consumed_mah - returns total current drawn since start-up in milliampere.hours
bool AP_BattMonitor::consumed_mah(float &mah, const uint8_t instance) const {
    if ((instance < _num_instances) && (drivers[instance] != nullptr) && drivers[instance]->has_current()) {
        mah = state[instance].consumed_mah;
        return true;
    } else {
        return false;
    }
}

/// consumed_wh - returns energy consumed since start-up in Watt.hours
bool AP_BattMonitor::consumed_wh(float &wh, const uint8_t instance) const {
    if (instance < _num_instances && drivers[instance] != nullptr && drivers[instance]->has_consumed_energy()) {
        wh = state[instance].consumed_wh;
        return true;
    } else {
        return false;
    }
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor::capacity_remaining_pct(uint8_t &percentage, uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr) {
        return drivers[instance]->capacity_remaining_pct(percentage);
    }
    return false;
}

/// time_remaining - returns remaining battery time
bool AP_BattMonitor::time_remaining(uint32_t &seconds, uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr && state[instance].has_time_remaining) {
        seconds = state[instance].time_remaining;
        return true;
    }
    return false;
}

/// pack_capacity_mah - returns the capacity of the battery pack in mAh when the pack is full
int32_t AP_BattMonitor::pack_capacity_mah(uint8_t instance) const
{
    if (instance < AP_BATT_MONITOR_MAX_INSTANCES) {
        return _params[instance]._pack_capacity;
    } else {
        return 0;
    }
}

void AP_BattMonitor::check_failsafes(void)
{
    if (hal.util->get_soft_armed()) {
        for (uint8_t i = 0; i < _num_instances; i++) {
            if (drivers[i] == nullptr) {
                continue;
            }

            const Failsafe type = drivers[i]->update_failsafes();
            if (type <= state[i].failsafe) {
                continue;
            }

            int8_t action = 0;
            const char *type_str = nullptr;
            switch (type) {
                case Failsafe::None:
                    continue; // should not have been called in this case
                case Failsafe::Low:
                    action = _params[i]._failsafe_low_action;
                    type_str = "low";
                    break;
                case Failsafe::Critical:
                    action = _params[i]._failsafe_critical_action;
                    type_str = "critical";
                    break;
            }

            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Battery %d is %s %.2fV used %.0f mAh", i + 1, type_str,
                            (double)voltage(i), (double)state[i].consumed_mah);
            _has_triggered_failsafe = true;
#ifndef HAL_BUILD_AP_PERIPH
            AP_Notify::flags.failsafe_battery = true;
#endif
            state[i].failsafe = type;

            // map the desired failsafe action to a prioritiy level
            int8_t priority = 0;
            if (_failsafe_priorities != nullptr) {
                while (_failsafe_priorities[priority] != -1) {
                    if (_failsafe_priorities[priority] == action) {
                        break;
                    }
                    priority++;
                }

            }

            // trigger failsafe if the action was equal or higher priority
            // It's valid to retrigger the same action if a different battery provoked the event
            if (priority <= _highest_failsafe_priority) {
                _battery_failsafe_handler_fn(type_str, action);
                _highest_failsafe_priority = priority;
            }
        }
    }
}

// return true if any battery is pushing too much power
bool AP_BattMonitor::overpower_detected() const
{
    bool result = false;
    for (uint8_t instance = 0; instance < _num_instances; instance++) {
        result |= overpower_detected(instance);
    }
    return result;
}

bool AP_BattMonitor::overpower_detected(uint8_t instance) const
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (instance < _num_instances && _params[instance]._watt_max > 0) {
        float power = state[instance].current_amps * state[instance].voltage;
        return state[instance].healthy && (power > _params[instance]._watt_max);
    }
    return false;
#else
    return false;
#endif
}

bool AP_BattMonitor::has_cell_voltages(const uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr) {
        return drivers[instance]->has_cell_voltages();
    }

    return false;
}

// return the current cell voltages, returns the first monitor instances cells if the instance is out of range
const AP_BattMonitor::cells & AP_BattMonitor::get_cell_voltages(const uint8_t instance) const
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return state[AP_BATT_PRIMARY_INSTANCE].cell_voltages;
    } else {
        return state[instance].cell_voltages;
    }
}

// returns true if there is a temperature reading
bool AP_BattMonitor::get_temperature(float &temperature, const uint8_t instance) const
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES || drivers[instance] == nullptr) {
        return false;
    } 
    
    temperature = state[instance].temperature;

    return drivers[instance]->has_temperature();
}

// return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor::get_cycle_count(uint8_t instance, uint16_t &cycles) const
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES || (drivers[instance] == nullptr)) {
        return false;
    }
    return drivers[instance]->get_cycle_count(cycles);
}

bool AP_BattMonitor::arming_checks(size_t buflen, char *buffer) const
{
    char temp_buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};

    for (uint8_t i = 0; i < AP_BATT_MONITOR_MAX_INSTANCES; i++) {
        if (drivers[i] != nullptr && !(drivers[i]->arming_checks(temp_buffer, sizeof(temp_buffer)))) {
            hal.util->snprintf(buffer, buflen, "Battery %d %s", i + 1, temp_buffer);
            return false;
        }
    }

    return true;
}

// Check's each smart battery instance for its powering off state and broadcasts notifications
void AP_BattMonitor::checkPoweringOff(void)
{
    for (uint8_t i = 0; i < _num_instances; i++) {
        if (state[i].is_powering_off && !state[i].powerOffNotified) {
#ifndef HAL_BUILD_AP_PERIPH
            // Set the AP_Notify flag, which plays the power off tones
            AP_Notify::flags.powering_off = true;
#endif

            // Send a Mavlink broadcast announcing the shutdown
#if HAL_GCS_ENABLED
            mavlink_command_long_t cmd_msg{};
            cmd_msg.command = MAV_CMD_POWER_OFF_INITIATED;
            cmd_msg.param1 = i+1;
            GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
            gcs().send_text(MAV_SEVERITY_WARNING, "Vehicle %d battery %d is powering off", mavlink_system.sysid, i+1);
#endif

            // only send this once
            state[i].powerOffNotified = true;
        }
    }
}

/*
  reset battery remaining percentage for batteries that integrate to
  calculate percentage remaining
*/
bool AP_BattMonitor::reset_remaining_mask(uint16_t battery_mask, float percentage)
{
    static_assert(AP_BATT_MONITOR_MAX_INSTANCES <= 16, "More batteries are enabled then can be reset");
    bool ret = true;
    Failsafe highest_failsafe = Failsafe::None;
    for (uint8_t i = 0; i < _num_instances; i++) {
        if ((1U<<i) & battery_mask) {
            if (drivers[i] != nullptr) {
                ret &= drivers[i]->reset_remaining(percentage);
            } else {
                ret = false;
            }
        }
        if (state[i].failsafe > highest_failsafe) {
            highest_failsafe = state[i].failsafe;
        }
    }

    // If all backends are not in failsafe then set overall failsafe state
    if (highest_failsafe == Failsafe::None) {
        _highest_failsafe_priority = INT8_MAX;
        _has_triggered_failsafe = false;
        // and reset notify flag
        AP_Notify::flags.failsafe_battery = false;
    }
    return ret;
}

// Returns the mavlink charge state. The following mavlink charge states are not used
// MAV_BATTERY_CHARGE_STATE_EMERGENCY , MAV_BATTERY_CHARGE_STATE_FAILED
// MAV_BATTERY_CHARGE_STATE_UNHEALTHY, MAV_BATTERY_CHARGE_STATE_CHARGING
MAV_BATTERY_CHARGE_STATE AP_BattMonitor::get_mavlink_charge_state(const uint8_t instance) const 
{
    if (instance >= _num_instances) {
        return MAV_BATTERY_CHARGE_STATE_UNDEFINED;
    }

    switch (state[instance].failsafe) {

    case Failsafe::None:
        if (get_mavlink_fault_bitmask(instance) != 0 || !healthy()) {
            return MAV_BATTERY_CHARGE_STATE_UNHEALTHY;
        }
        return MAV_BATTERY_CHARGE_STATE_OK;

    case Failsafe::Low:
        return MAV_BATTERY_CHARGE_STATE_LOW;

    case Failsafe::Critical:
        return MAV_BATTERY_CHARGE_STATE_CRITICAL;
    }

    // Should not reach this
    return MAV_BATTERY_CHARGE_STATE_UNDEFINED;
}

// Returns mavlink fault state
uint32_t AP_BattMonitor::get_mavlink_fault_bitmask(const uint8_t instance) const
{
    if (drivers[instance] == nullptr) {
        return 0;
    }
    return drivers[instance]->get_mavlink_fault_bitmask();
}

/*
  check that all configured battery monitors are healthy
 */
bool AP_BattMonitor::healthy() const
{
    for (uint8_t i=0; i< _num_instances; i++) {
        if (get_type(i) != Type::NONE && !healthy(i)) {
            return false;
        }
    }
    return true;
}

namespace AP {

AP_BattMonitor &battery()
{
    return *AP_BattMonitor::get_singleton();
}

};
