/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor_SMBus.h"
#include "AP_BattMonitor_Bebop.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor::var_info[] = {
    // @Param: _MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:SMBus,6:Bebop
    // @User: Standard
    AP_GROUPINFO("_MONITOR", 0, AP_BattMonitor, _monitoring[0], BattMonitor_TYPE_NONE),

    // @Param: _VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:Pixhawk, 13:A13, 100:PX4
    // @User: Standard
    AP_GROUPINFO("_VOLT_PIN", 1, AP_BattMonitor, _volt_pin[0], AP_BATT_VOLT_PIN),

    // @Param: _CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
    // @Values: -1:Disabled, 1:A1, 2:A2, 3:Pixhawk, 12:A12, 101:PX4
    // @User: Standard
    AP_GROUPINFO("_CURR_PIN", 2, AP_BattMonitor, _curr_pin[0], AP_BATT_CURR_PIN),

    // @Param: _VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("_VOLT_MULT", 3, AP_BattMonitor, _volt_multiplier[0], AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: _AMP_PERVOLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
    // @Units: Amps/Volt
    // @User: Standard
    AP_GROUPINFO("_AMP_PERVOLT", 4, AP_BattMonitor, _curr_amp_per_volt[0], AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: _AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: Volts
    // @User: Standard
    AP_GROUPINFO("_AMP_OFFSET", 5, AP_BattMonitor, _curr_amp_offset[0], 0),

    // @Param: _CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("_CAPACITY", 6, AP_BattMonitor, _pack_capacity[0], AP_BATT_CAPACITY_DEFAULT),

    // 7 & 8 were used for VOLT2_PIN and VOLT2_MULT

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: _WATT_MAX
    // @DisplayName: Maximum allowed power (Watts)
    // @Description: If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
    // @Units: Watts
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_WATT_MAX", 9, AP_BattMonitor, _watt_max[0], AP_BATT_MAX_WATT_DEFAULT),
#endif

    // 10 is left for future expansion

#if AP_BATT_MONITOR_MAX_INSTANCES > 1
    // @Param: 2_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:SMBus,6:Bebop
    // @User: Standard
    AP_GROUPINFO("2_MONITOR", 11, AP_BattMonitor, _monitoring[1], BattMonitor_TYPE_NONE),

    // @Param: 2_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:Pixhawk, 13:A13, 100:PX4
    // @User: Standard
    AP_GROUPINFO("2_VOLT_PIN", 12, AP_BattMonitor, _volt_pin[1], AP_BATT_VOLT_PIN),

    // @Param: 2_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
    // @Values: -1:Disabled, 1:A1, 2:A2, 3:Pixhawk, 12:A12, 101:PX4
    // @User: Standard
    AP_GROUPINFO("2_CURR_PIN", 13, AP_BattMonitor, _curr_pin[1], AP_BATT_CURR_PIN),

    // @Param: 2_VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("2_VOLT_MULT", 14, AP_BattMonitor, _volt_multiplier[1], AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: 2_AMP_PERVOL
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
    // @Units: Amps/Volt
    // @User: Standard
    AP_GROUPINFO("2_AMP_PERVOL", 15, AP_BattMonitor, _curr_amp_per_volt[1], AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: 2_AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: Volts
    // @User: Standard
    AP_GROUPINFO("2_AMP_OFFSET", 16, AP_BattMonitor, _curr_amp_offset[1], 0),

    // @Param: 2_CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("2_CAPACITY", 17, AP_BattMonitor, _pack_capacity[1], AP_BATT_CAPACITY_DEFAULT),


#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: 2_WATT_MAX
    // @DisplayName: Maximum allowed current
    // @Description: If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
    // @Units: Amps
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_WATT_MAX", 18, AP_BattMonitor, _watt_max[1], AP_BATT_MAX_WATT_DEFAULT),
#endif

#endif // AP_BATT_MONITOR_MAX_INSTANCES > 1

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

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    // force monitor for bebop
    _monitoring[0] = BattMonitor_TYPE_BEBOP;
#endif

    // create each instance
    for (uint8_t instance=0; instance<AP_BATT_MONITOR_MAX_INSTANCES; instance++) {
        uint8_t monitor_type = _monitoring[instance];
        switch (monitor_type) {
            case BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY:
            case BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT:
                state[instance].instance = instance;
                drivers[instance] = new AP_BattMonitor_Analog(*this, instance, state[instance]);
                _num_instances++;
                break;
            case BattMonitor_TYPE_SMBUS:
                state[instance].instance = instance;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
                drivers[instance] = new AP_BattMonitor_SMBus_PX4(*this, instance, state[instance]);
#else
                drivers[instance] = new AP_BattMonitor_SMBus_I2C(*this, instance, state[instance]);
#endif
                _num_instances++;
                break;
            case BattMonitor_TYPE_BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
                state[instance].instance = instance;
                drivers[instance] = new AP_BattMonitor_Bebop(*this, instance, state[instance]);
                _num_instances++;
#endif
                break;
        }

        // call init function for each backend
        if (drivers[instance] != NULL) {
            drivers[instance]->init();
        }
    }
}

// read - read the voltage and current for all instances
void
AP_BattMonitor::read()
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != NULL && _monitoring[i] != BattMonitor_TYPE_NONE) {
            drivers[i]->read();
        }
    }
}

// healthy - returns true if monitor is functioning
bool AP_BattMonitor::healthy(uint8_t instance) const {
    return instance < _num_instances && _BattMonitor_STATE(instance).healthy;
}

bool AP_BattMonitor::is_powering_off(uint8_t instance) const {
    return instance < _num_instances && _BattMonitor_STATE(instance).is_powering_off;
}

/// has_current - returns true if battery monitor instance provides current info
bool AP_BattMonitor::has_current(uint8_t instance) const
{
    // check for analog voltage and current monitor or smbus monitor
    if (instance < _num_instances && drivers[instance] != NULL) {
        return (_monitoring[instance] == BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT ||
                _monitoring[instance] == BattMonitor_TYPE_SMBUS ||
                _monitoring[instance] == BattMonitor_TYPE_BEBOP);
    }

    // not monitoring current
    return false;
}

/// voltage - returns battery voltage in volts
float AP_BattMonitor::voltage(uint8_t instance) const
{
    if (instance < _num_instances) {
        return _BattMonitor_STATE(instance).voltage;
    } else {
        return 0.0f;
    }
}

/// current_amps - returns the instantaneous current draw in amperes
float AP_BattMonitor::current_amps(uint8_t instance) const {
    if (instance < _num_instances) {
        return _BattMonitor_STATE(instance).current_amps;
    } else {
        return 0.0f;
    }
}

/// current_total_mah - returns total current drawn since start-up in amp-hours
float AP_BattMonitor::current_total_mah(uint8_t instance) const {
    if (instance < _num_instances) {
        return _BattMonitor_STATE(instance).current_total_mah;
    } else {
        return 0.0f;
    }
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor::capacity_remaining_pct(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != NULL) {
        return drivers[instance]->capacity_remaining_pct();
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

    // check voltage
    if ((state[instance].voltage > 0) && (low_voltage > 0) && (state[instance].voltage < low_voltage)) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (state[instance].low_voltage_start_ms == 0) {
            state[instance].low_voltage_start_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - state[instance].low_voltage_start_ms > AP_BATT_LOW_VOLT_TIMEOUT_MS) {
            return true;
        }
    } else {
        // acceptable voltage so reset timer
        state[instance].low_voltage_start_ms = 0;
    }

    // check capacity if current monitoring is enabled
    if (has_current(instance) && (min_capacity_mah > 0) && (_pack_capacity[instance] - state[instance].current_total_mah < min_capacity_mah)) {
        return true;
    }

    // if we've gotten this far then battery is ok
    return false;
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// return true if any battery is pushing too much power
bool AP_BattMonitor::overpower_detected() const
{
    bool result = false;
    for (int instance = 0; instance < _num_instances; instance++) {
        result |= overpower_detected(instance);
    }
    return result;
}

bool AP_BattMonitor::overpower_detected(uint8_t instance) const
{
    if (instance < _num_instances && _watt_max[instance] > 0) {
        float power = _BattMonitor_STATE(instance).current_amps * _BattMonitor_STATE(instance).voltage;
        return _BattMonitor_STATE(instance).healthy && (power > _watt_max[instance]);
    }
    return false;
}
#endif

