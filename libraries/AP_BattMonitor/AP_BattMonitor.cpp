/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor_SMBus.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor::var_info[] PROGMEM = {
    // @Param: MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:SMBus
    // @User: Standard
    AP_GROUPINFO("_MONITOR", 0, AP_BattMonitor, _monitoring[0], BattMonitor_TYPE_NONE),

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:Pixhawk, 13:A13, 100:PX4
    // @User: Standard
    AP_GROUPINFO("_VOLT_PIN", 1, AP_BattMonitor, _volt_pin[0], AP_BATT_VOLT_PIN),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
    // @Values: -1:Disabled, 1:A1, 2:A2, 3:Pixhawk, 12:A12, 101:PX4
    // @User: Standard
    AP_GROUPINFO("_CURR_PIN", 2, AP_BattMonitor, _curr_pin[0], AP_BATT_CURR_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("_VOLT_MULT", 3, AP_BattMonitor, _volt_multiplier[0], AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: AMP_PERVOLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
    // @Units: Amps/Volt
    // @User: Standard
    AP_GROUPINFO("_AMP_PERVOLT", 4, AP_BattMonitor, _curr_amp_per_volt[0], AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: Volts
    // @User: Standard
    AP_GROUPINFO("_AMP_OFFSET", 5, AP_BattMonitor, _curr_amp_offset[0], 0),

    // @Param: CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("_CAPACITY", 6, AP_BattMonitor, _pack_capacity[0], AP_BATT_CAPACITY_DEFAULT),

    // 7 & 8 were used for VOLT2_PIN and VOLT2_MULT
    // 9..10 left for future expansion

#if AP_BATT_MONITOR_MAX_INSTANCES > 1
    // @Param: 2_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:SMBus
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

    // create each instance
    for (uint8_t instance=0; instance<AP_BATT_MONITOR_MAX_INSTANCES; instance++) {
        uint8_t monitor_type = _monitoring[instance];

        // check for analog instance
        if (monitor_type == BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY || monitor_type == BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT) {
            state[instance].instance = instance;
            drivers[instance] = new AP_BattMonitor_Analog(*this, instance, state[instance]);
            _num_instances++;

        // check for SMBus
        } else if (monitor_type == BattMonitor_TYPE_SMBUS) {
            state[instance].instance = instance;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
            drivers[instance] = new AP_BattMonitor_SMBus_PX4(*this, instance, state[instance]);
#else
            drivers[instance] = new AP_BattMonitor_SMBus_I2C(*this, instance, state[instance]);
#endif
            _num_instances++;
        }
    }
}

// read - read the voltage and current for all instances
void
AP_BattMonitor::read()
{
    // exit immediately if no monitors setup
    if (_num_instances == 0) {
        return;
    }

    for (uint8_t i=0; i<AP_BATT_MONITOR_MAX_INSTANCES; i++) {
        if (drivers[i] != NULL && _monitoring[i] != BattMonitor_TYPE_NONE) {
            drivers[i]->read();
        }
    }
}

// healthy - returns true if monitor is functioning
bool AP_BattMonitor::healthy(uint8_t instance) const {
    return instance < AP_BATT_MONITOR_MAX_INSTANCES && _BattMonitor_STATE(instance).healthy;
}

/// has_current - returns true if battery monitor instance provides current info
bool AP_BattMonitor::has_current(uint8_t instance) const
{
    // check for analog voltage and current monitor or smbus monitor
    if (instance < AP_BATT_MONITOR_MAX_INSTANCES) {
        if (drivers[instance] != NULL) {
            return (_monitoring[instance] == BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT || _monitoring[instance] == BattMonitor_TYPE_SMBUS);
        }
    }

    // not monitoring current
    return false;
}

/// voltage - returns battery voltage in volts
float AP_BattMonitor::voltage(uint8_t instance) const
{
    if (instance < AP_BATT_MONITOR_MAX_INSTANCES) {
        return _BattMonitor_STATE(instance).voltage;
    } else {
        return 0.0f;
    }
}

// voltage2 - returns the voltage of the second battery (helper function to send 2nd voltage to GCS)
float AP_BattMonitor::voltage2() const
{
    // exit immediately if one or fewer monitors
    if (_num_instances < 2) {
        return 0.0f;
    }

    // get next battery's voltage
    for (uint8_t i=1; i<AP_BATT_MONITOR_MAX_INSTANCES; i++) {
        if (drivers[i] != NULL) {
            return _BattMonitor_STATE(i).voltage;
        }
    }

    // if we somehow got here, return zero
    return 0.0f;
}

/// current_amps - returns the instantaneous current draw in amperes
float AP_BattMonitor::current_amps(uint8_t instance) const {
    if (instance < AP_BATT_MONITOR_MAX_INSTANCES) {
        return _BattMonitor_STATE(instance).current_amps;
    } else {
        return 0.0f;
    }
}

/// current_total_mah - returns total current drawn since start-up in amp-hours
float AP_BattMonitor::current_total_mah(uint8_t instance) const {
    if (instance < AP_BATT_MONITOR_MAX_INSTANCES) {
        return _BattMonitor_STATE(instance).current_total_mah;
    } else {
        return 0.0f;
    }
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor::capacity_remaining_pct(uint8_t instance) const
{
    if (_num_instances == 0 || instance >= AP_BATT_MONITOR_MAX_INSTANCES || drivers[instance] == NULL) {
        return 0;
    } else {
        return drivers[instance]->capacity_remaining_pct();
    }
}

/// exhausted - returns true if the voltage remains below the low_voltage for 10 seconds or remaining capacity falls below min_capacity_mah
bool AP_BattMonitor::exhausted(uint8_t instance, float low_voltage, float min_capacity_mah)
{
    // exit immediately if no monitors setup
    if (_num_instances == 0 || instance >= AP_BATT_MONITOR_MAX_INSTANCES || drivers[instance] == NULL) {
        return false;
    }

    // get current time
    uint32_t tnow = hal.scheduler->millis();

    // check voltage
    if ((state[instance].voltage > 0.0f) && (low_voltage > 0) && (state[instance].voltage < low_voltage)) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (state[instance].low_voltage_start_ms == 0) {
            state[instance].low_voltage_start_ms = tnow;
        }else if (tnow - state[instance].low_voltage_start_ms > AP_BATT_LOW_VOLT_TIMEOUT_MS) {
            return true;
        }
    }else{
        // acceptable voltage so reset timer
        state[instance].low_voltage_start_ms = 0;
    }

    // check capacity if current monitoring is enabled
    if (has_current(instance) && (min_capacity_mah>0) && (_pack_capacity[instance] - state[instance].current_total_mah < min_capacity_mah)) {
        return true;
    }

    // if we've gotten this far battery is ok
    return false;
}
