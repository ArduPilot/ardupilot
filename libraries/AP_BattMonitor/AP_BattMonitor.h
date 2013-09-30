/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_BATTMONITOR_H
#define AP_BATTMONITOR_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>

// battery monitor types
#define AP_BATT_MONITOR_DISABLED            0
#define AP_BATT_MONITOR_VOLTAGE_ONLY        3
#define AP_BATT_MONITOR_VOLTAGE_AND_CURRENT 4

// setup default mag orientation for each board type
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 # define AP_BATT_VOLT_PIN                  0       // Battery voltage on A0
 # define AP_BATT_CURR_PIN                  1       // Battery current on A1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       3.56    // on-board APM1 voltage divider with a 3.9kOhm resistor
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  0
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define AP_BATT_VOLT_PIN                  13      // APM2.5/2.6 with 3dr power module
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT   0
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
// Flymaple board pin 20 is connected to the external battery supply
// via a 24k/5.1k voltage divider. The schematic claims the divider is 25k/5k, 
// but the actual installed resistors are not so.
// So the divider ratio is 5.70588 = (24000+5100)/5100
 # define AP_BATT_VOLT_PIN                  20
 # define AP_BATT_CURR_PIN                  19
 # define AP_BATT_VOLTDIVIDER_DEFAULT       5.70588
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  0
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 # define AP_BATT_VOLT_PIN                  2
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define AP_BATT_VOLT_PIN                  13
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17
#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  0
#endif

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor


#define AP_BATT_CAPACITY_DEFAULT            1760

#define AP_BATT_INITIAL_VOLTAGE 99              // initial voltage set on start-up to avoid low voltage alarms

class AP_BattMonitor
{
public:

    /// Constructor
    AP_BattMonitor();

    /// Initialize the battery monitor
    void init();

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

    /// monitoring - returns whether we are monitoring voltage only or voltage and current
    int8_t monitoring() const { return _monitoring; }

    /// pack_capacity - battery pack capacity in mAh less reserve
    float pack_capacity() const { return _pack_capacity; }

    /// Battery voltage.  Initialized to 99 to prevent low voltage events at startup
    float voltage() const { return _voltage; }

    /// Battery pack instantaneous currrent draw in amperes
    float current_amps() const { return _current_amps; }

    /// Total current drawn since start-up (Amp-hours)
    float current_total_mah() const { return _current_total_mah; }

    /// time when current was last read
    uint32_t last_time_ms() const { return _last_time_ms; }

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    uint8_t capacity_remaining_pct() const;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_Int8     _monitoring;                 /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8     _volt_pin;
    AP_Int8     _curr_pin;
    AP_Float    _volt_multiplier;
    AP_Float    _curr_amp_per_volt;
    AP_Float    _curr_amp_offset;
    AP_Int32    _pack_capacity;              /// battery pack capacity less reserve

    /// internal variables
    float       _voltage;                   /// last read voltage
    float       _current_amps;              /// last read current drawn
    float       _current_total_mah;         /// total current drawn since startup (Amp-hours)
    uint32_t    _last_time_ms;              /// time when current was last read

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;

};
#endif  // AP_BATTMONITOR_H
