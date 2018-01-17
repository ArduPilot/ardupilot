#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor_SMBus.h"
#include "AP_BattMonitor_Bebop.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor::var_info[] = {
    // 0 - 18, 20- 22 used by old parameter indexes

    // @Group: _
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO_FLAGS(_params[0], "_", 23, AP_BattMonitor, AP_BattMonitor_Params, AP_PARAM_FLAG_IGNORE_ENABLE),

    // @Group: 2_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 24, AP_BattMonitor, AP_BattMonitor_Params),


    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_BattMonitor::AP_BattMonitor(void) :
    _num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// init - instantiate the battery monitors
void
AP_BattMonitor::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    convert_params();

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
    // force monitor for bebop
    _params[0]._type.set(AP_BattMonitor_Params::BattMonitor_TYPE_BEBOP);
#endif

    // create each instance
    for (uint8_t instance=0; instance<AP_BATT_MONITOR_MAX_INSTANCES; instance++) {
        // clear out the cell voltages
        memset(&state[instance].cell_voltages, 0xFF, sizeof(cells));

        switch (get_type(instance)) {
            case AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY:
            case AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT:
                drivers[instance] = new AP_BattMonitor_Analog(*this, state[instance], _params[instance]);
                _num_instances++;
                break;
            case AP_BattMonitor_Params::BattMonitor_TYPE_SOLO:
                drivers[instance] = new AP_BattMonitor_SMBus_Solo(*this, state[instance], _params[instance],
                                                                  hal.i2c_mgr->get_device(AP_BATTMONITOR_SMBUS_BUS_INTERNAL, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                          100000, true, 20));
                _num_instances++;
                break;
            case AP_BattMonitor_Params::BattMonitor_TYPE_MAXELL:
                drivers[instance] = new AP_BattMonitor_SMBus_Maxell(*this, state[instance], _params[instance],
                                                                    hal.i2c_mgr->get_device(AP_BATTMONITOR_SMBUS_BUS_EXTERNAL, AP_BATTMONITOR_SMBUS_I2C_ADDR,
                                                                                            100000, true, 20));
                _num_instances++;
                break;
            case AP_BattMonitor_Params::BattMonitor_TYPE_BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
                drivers[instance] = new AP_BattMonitor_Bebop(*this, state[instance], _params[instance]);
                _num_instances++;
#endif
                break;
            case AP_BattMonitor_Params::BattMonitor_TYPE_NONE:
            default:
                break;
        }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            drivers[instance]->init();
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
#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
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
            hal.util->snprintf(param_name, 17, "BATT2_%s", AP_BattMonitor_Params::var_info[destination_index].name);
        } else {
            hal.util->snprintf(param_name, 17, "BATT_%s", AP_BattMonitor_Params::var_info[destination_index].name);
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
        if (drivers[i] != nullptr && _params[i].type() != AP_BattMonitor_Params::BattMonitor_TYPE_NONE) {
            drivers[i]->read();
            drivers[i]->update_resistance_estimate();
        }
    }
}

// healthy - returns true if monitor is functioning
bool AP_BattMonitor::healthy(uint8_t instance) const {
    return instance < _num_instances && state[instance].healthy;
}

/// has_current - returns true if battery monitor instance provides current info
bool AP_BattMonitor::has_current(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr && _params[instance].type() != AP_BattMonitor_Params::BattMonitor_TYPE_NONE) {
        return drivers[instance]->has_current();
    }

    // not monitoring current
    return false;
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
    if (instance < _num_instances) {
        // resting voltage should always be greater than or equal to the raw voltage
        return MAX(state[instance].voltage, state[instance].voltage_resting_estimate);
    } else {
        return 0.0f;
    }
}

/// current_amps - returns the instantaneous current draw in amperes
float AP_BattMonitor::current_amps(uint8_t instance) const {
    if (instance < _num_instances) {
        return state[instance].current_amps;
    } else {
        return 0.0f;
    }
}

/// current_total_mah - returns total current drawn since start-up in amp-hours
float AP_BattMonitor::current_total_mah(uint8_t instance) const {
    if (instance < _num_instances) {
        return state[instance].current_total_mah;
    } else {
        return 0.0f;
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

 /// exhausted - returns true if the voltage remains below the low_voltage for 10 seconds or remaining capacity falls below min_capacity_mah
bool AP_BattMonitor::exhausted(uint8_t instance, float low_voltage, float min_capacity_mah)
{
    // exit immediately if no monitors setup
    if (_num_instances == 0 || instance >= _num_instances) {
        return false;
    }

    // use voltage or sag compensated voltage
    float voltage_used;
    switch (_params[instance].failsafe_voltage_source()) {
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_Raw:
        default:
            voltage_used = state[instance].voltage;
            break;
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_SagCompensated:
            voltage_used = voltage_resting_estimate(instance);
            break;
    }

    // check voltage
    if ((voltage_used > 0) && (low_voltage > 0) && (voltage_used < low_voltage)) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (state[instance].low_voltage_start_ms == 0) {
            state[instance].low_voltage_start_ms = AP_HAL::millis();
        } else if (_params[instance]._low_voltage_timeout > 0 &&
                   AP_HAL::millis() - state[instance].low_voltage_start_ms > uint32_t(_params[instance]._low_voltage_timeout)*1000U) {
            return true;
        }
    } else {
        // acceptable voltage so reset timer
        state[instance].low_voltage_start_ms = 0;
    }

    // check capacity if current monitoring is enabled
    if (has_current(instance) && (min_capacity_mah > 0) &&
        (_params[instance]._pack_capacity - state[instance].current_total_mah < min_capacity_mah)) {
        return true;
    }

    // if we've gotten this far then battery is ok
    return false;
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
