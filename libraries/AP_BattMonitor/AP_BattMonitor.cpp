#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor_SMBus.h"
#include "AP_BattMonitor_SMBus_Solo.h"
#include "AP_BattMonitor_SMBus_Generic.h"
#include "AP_BattMonitor_SMBus_Maxell.h"
#include "AP_BattMonitor_Bebop.h"
#include "AP_BattMonitor_BLHeliESC.h"
#include "AP_BattMonitor_SMBus_SUI.h"
#include "AP_BattMonitor_SMBus_NeoDesign.h"
#include "AP_BattMonitor_Sum.h"
#include "AP_BattMonitor_FuelFlow.h"
#include "AP_BattMonitor_FuelLevel_PWM.h"
#include "AP_BattMonitor_Generator.h"

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN
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

    // @Group: _
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "_", 23, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 2_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 24, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 3_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 25, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 4_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 26, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 5_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 27, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 6_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 28, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 7_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 29, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 8_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 30, AP_BattMonitor, AP_BattMonitor_Params),

    // @Group: 9_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 31, AP_BattMonitor, AP_BattMonitor_Params),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_BattMonitor::AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities) :
    _log_battery_bit(log_battery_bit),
    _num_instances(0),
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

    convert_params();

#ifdef HAL_BATT_MONITOR_DEFAULT
    if (_params[0]._type == 0) {
        // we can't use set_default() as the type is used as a flag for parameter conversion
        _params[0]._type.set(int8_t(HAL_BATT_MONITOR_DEFAULT));
    }
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
#if HAL_BATTMON_SMBUS_ENABLE
            case Type::SOLO:
                _params[instance]._i2c_bus.set_default(AP_BATTMONITOR_SMBUS_BUS_INTERNAL);
                drivers[instance] = new AP_BattMonitor_SMBus_Solo(*this, state[instance], _params[instance],
                                                                  hal.i2c_mgr->get_device(_params[instance]._i2c_bus, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                          100000, true, 20));
                break;
            case Type::SMBus_Generic:
                _params[instance]._i2c_bus.set_default(AP_BATTMONITOR_SMBUS_BUS_EXTERNAL);
                drivers[instance] = new AP_BattMonitor_SMBus_Generic(*this, state[instance], _params[instance],
                                                                     hal.i2c_mgr->get_device(_params[instance]._i2c_bus, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                             100000, true, 20));
                break;
            case Type::SUI3:
                _params[instance]._i2c_bus.set_default(AP_BATTMONITOR_SMBUS_BUS_INTERNAL),
                drivers[instance] = new AP_BattMonitor_SMBus_SUI(*this, state[instance], _params[instance],
                                                                 hal.i2c_mgr->get_device(_params[instance]._i2c_bus, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                          100000, true, 20), 3);
                break;
            case Type::SUI6:
                _params[instance]._i2c_bus.set_default(AP_BATTMONITOR_SMBUS_BUS_INTERNAL),
                drivers[instance] = new AP_BattMonitor_SMBus_SUI(*this, state[instance], _params[instance],
                                                                 hal.i2c_mgr->get_device(_params[instance]._i2c_bus, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                         100000, true, 20), 6);
                break;
            case Type::MAXELL:
                _params[instance]._i2c_bus.set_default(AP_BATTMONITOR_SMBUS_BUS_EXTERNAL);
                drivers[instance] = new AP_BattMonitor_SMBus_Maxell(*this, state[instance], _params[instance],
                                                                    hal.i2c_mgr->get_device(_params[instance]._i2c_bus, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                            100000, true, 20));
                break;
#endif // HAL_BATTMON_SMBUS_ENABLE
            case Type::BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
                drivers[instance] = new AP_BattMonitor_Bebop(*this, state[instance], _params[instance]);
#endif
                break;
            case Type::UAVCAN_BatteryInfo:
#if HAL_WITH_UAVCAN
                drivers[instance] = new AP_BattMonitor_UAVCAN(*this, state[instance], AP_BattMonitor_UAVCAN::UAVCAN_BATTERY_INFO, _params[instance]);
#endif
                break;
            case Type::BLHeliESC:
#ifdef HAVE_AP_BLHELI_SUPPORT
                drivers[instance] = new AP_BattMonitor_BLHeliESC(*this, state[instance], _params[instance]);
#endif
                break;
            case Type::Sum:
                drivers[instance] = new AP_BattMonitor_Sum(*this, state[instance], _params[instance], instance);
                break;
#if HAL_BATTMON_FUEL_ENABLE
            case Type::FuelFlow:
                drivers[instance] = new AP_BattMonitor_FuelFlow(*this, state[instance], _params[instance]);
                break;
            case Type::FuelLevel_PWM:
                drivers[instance] = new AP_BattMonitor_FuelLevel_PWM(*this, state[instance], _params[instance]);
                break;
#endif // HAL_BATTMON_FUEL_ENABLE
            case Type::NeoDesign:
                _params[instance]._i2c_bus.set_default(AP_BATTMONITOR_SMBUS_BUS_INTERNAL),
                drivers[instance] = new AP_BattMonitor_SMBus_NeoDesign(*this, state[instance], _params[instance],
                                                                 hal.i2c_mgr->get_device(_params[instance]._i2c_bus, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                         100000, true, 20));
                break;
            case Type::Generator:
                drivers[instance] = new AP_BattMonitor_Generator(*this, state[instance], _params[instance]);
                break;
            case Type::NONE:
            default:
                break;
        }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            drivers[instance]->init();
            // _num_instances is actually the index for looping over instances
            // the user may have BATT_MONITOR=0 and BATT2_MONITOR=7, in which case
            // there will be a gap, but as we always check for drivers[instances] being nullptr
            // this is safe
            _num_instances = instance + 1;
        }
    }
}

void AP_BattMonitor::convert_params(void) {
    if (_params[0]._type.configured_in_storage()) {
        // _params[0]._type will always be configured in storage after conversion is done the first time
        return;
    }

    #define SECOND_BATT_CONVERT_MASK 0x80
    const struct ConversionTable {
        uint8_t old_element;
        uint8_t new_index; // upper bit used to indicate if its the first or second instance
    }conversionTable[22] = {
        { 0,                             0 }, // _MONITOR
        { 1,                             1 }, // _VOLT_PIN
        { 2,                             2 }, // _CURR_PIN
        { 3,                             3 }, // _VOLT_MULT
        { 4,                             4 }, // _AMP_PERVOLT
        { 5,                             5 }, // _AMP_OFFSET
        { 6,                             6 }, // _CAPACITY
        { 9,                             7 }, // _WATT_MAX
        {10,                             8 }, // _SERIAL_NUM
        {11, (SECOND_BATT_CONVERT_MASK | 0)}, // 2_MONITOR
        {12, (SECOND_BATT_CONVERT_MASK | 1)}, // 2_VOLT_PIN
        {13, (SECOND_BATT_CONVERT_MASK | 2)}, // 2_CURR_PIN
        {14, (SECOND_BATT_CONVERT_MASK | 3)}, // 2_VOLT_MULT
        {15, (SECOND_BATT_CONVERT_MASK | 4)}, // 2_AMP_PERVOLT
        {16, (SECOND_BATT_CONVERT_MASK | 5)}, // 2_AMP_OFFSET
        {17, (SECOND_BATT_CONVERT_MASK | 6)}, // 2_CAPACITY
        {18, (SECOND_BATT_CONVERT_MASK | 7)}, // 2_WATT_MAX
        {20, (SECOND_BATT_CONVERT_MASK | 8)}, // 2_SERIAL_NUM
        {21,                             9 }, // _LOW_TIMER
        {22,                            10 }, // _LOW_TYPE
        {21, (SECOND_BATT_CONVERT_MASK | 9)}, // 2_LOW_TIMER
        {22, (SECOND_BATT_CONVERT_MASK |10)}, // 2_LOW_TYPE
    };


    char param_name[17];
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    info.old_key = 166;
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    info.old_key = 36;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    info.old_key = 33;
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    info.old_key = 145;
#else
    _params[0]._type.save(true);
    return; // no conversion is supported on this platform
#endif

    for (uint8_t i = 0; i < ARRAY_SIZE(conversionTable); i++) {
        uint8_t param_instance = conversionTable[i].new_index >> 7;
        uint8_t destination_index = 0x7F & conversionTable[i].new_index;

        info.old_group_element = conversionTable[i].old_element;
        info.type = (ap_var_type)AP_BattMonitor_Params::var_info[destination_index].type;
        if (param_instance) {
            hal.util->snprintf(param_name, sizeof(param_name), "BATT2_%s", AP_BattMonitor_Params::var_info[destination_index].name);
        } else {
            hal.util->snprintf(param_name, sizeof(param_name), "BATT_%s", AP_BattMonitor_Params::var_info[destination_index].name);
        }

        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }

    // force _params[0]._type into storage to flag that conversion has been done
    _params[0]._type.save(true);
}

// read - read the voltage and current for all instances
void
AP_BattMonitor::read()
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != nullptr && get_type(i) != Type::NONE) {
            drivers[i]->read();
            drivers[i]->update_resistance_estimate();
        }
    }

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger->should_log(_log_battery_bit)) {
        logger->Write_Current();
        logger->Write_Power();
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

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor::capacity_remaining_pct(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr) {
        return drivers[instance]->capacity_remaining_pct();
    } else {
        return 0;
    }
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

            const BatteryFailsafe type = drivers[i]->update_failsafes();
            if (type <= state[i].failsafe) {
                continue;
            }

            int8_t action = 0;
            const char *type_str = nullptr;
            switch (type) {
                case AP_BattMonitor::BatteryFailsafe_None:
                    continue; // should not have been called in this case
                case AP_BattMonitor::BatteryFailsafe_Low:
                    action = _params[i]._failsafe_low_action;
                    type_str = "low";
                    break;
                case AP_BattMonitor::BatteryFailsafe_Critical:
                    action = _params[i]._failsafe_critical_action;
                    type_str = "critical";
                    break;
            }

            gcs().send_text(MAV_SEVERITY_WARNING, "Battery %d is %s %.2fV used %.0f mAh", i + 1, type_str,
                            (double)voltage(i), (double)state[i].consumed_mah);
            _has_triggered_failsafe = true;
            AP_Notify::flags.failsafe_battery = true;
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
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return false;
    } else {
        temperature = state[instance].temperature;
        return (AP_HAL::millis() - state[instance].temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
    }
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
            // Set the AP_Notify flag, which plays the power off tones
            AP_Notify::flags.powering_off = true;

            // Send a Mavlink broadcast announcing the shutdown
            mavlink_command_long_t cmd_msg{};
            cmd_msg.command = MAV_CMD_POWER_OFF_INITIATED;
            cmd_msg.param1 = i+1;
            GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
            gcs().send_text(MAV_SEVERITY_WARNING, "Vehicle %d battery %d is powering off", mavlink_system.sysid, i+1);

            // only send this once
            state[i].powerOffNotified = true;
        }
    }
}

/*
  reset battery remaining percentage for batteries that integrate to
  calculate percentage remaining
*/
bool AP_BattMonitor::reset_remaining(uint16_t battery_mask, float percentage)
{
    bool ret = true;
    BatteryFailsafe highest_failsafe = BatteryFailsafe_None;
    for (uint8_t i = 0; i < _num_instances; i++) {
        if ((1U<<i) & battery_mask) {
            ret &= drivers[i]->reset_remaining(percentage);
        }
        if (state[i].failsafe > highest_failsafe) {
            highest_failsafe = state[i].failsafe;
        }
    }

    // If all backends are not in failsafe then set overall failsafe state
    if (highest_failsafe == BatteryFailsafe_None) {
        _highest_failsafe_priority = INT8_MAX;
        _has_triggered_failsafe = false;
        // and reset notify flag
        AP_Notify::flags.failsafe_battery = false;
    }
    return ret;
}

namespace AP {

AP_BattMonitor &battery()
{
    return *AP_BattMonitor::get_singleton();
}

};
