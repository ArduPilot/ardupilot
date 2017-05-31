#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

// maximum number of battery monitors
#define AP_BATT_MONITOR_MAX_INSTANCES       2

// first monitor is always the primary monitor
#define AP_BATT_PRIMARY_INSTANCE            0

#define AP_BATT_CAPACITY_DEFAULT            3300
#define AP_BATT_LOW_VOLT_TIMEOUT_DEFAULT    10   // low voltage of 10 seconds will cause battery_exhausted to return true
#define AP_BATT_MAX_WATT_DEFAULT            0
#define AP_BATT_SERIAL_NUMBER_DEFAULT       -1

#define AP_BATT_MONITOR_TIMEOUT             5000

#define AP_BATT_MONITOR_RES_EST_TC_1        0.5f
#define AP_BATT_MONITOR_RES_EST_TC_2        0.1f

// declare backend class
class AP_BattMonitor_Backend;
class AP_BattMonitor_Analog;
class AP_BattMonitor_SMBus;
class AP_BattMonitor_SMBus_Solo;
class AP_BattMonitor_SMBus_Maxell;

class AP_BattMonitor
{
    friend class AP_BattMonitor_Backend;
    friend class AP_BattMonitor_Analog;
    friend class AP_BattMonitor_SMBus;
    friend class AP_BattMonitor_SMBus_Solo;
    friend class AP_BattMonitor_SMBus_Maxell;

public:

    /// Constructor
    AP_BattMonitor();

    // Battery monitor driver types
    enum BattMonitor_Type {
        BattMonitor_TYPE_NONE                       = 0,
        BattMonitor_TYPE_ANALOG_VOLTAGE_ONLY        = 3,
        BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT = 4,
        BattMonitor_TYPE_SOLO                       = 5,
        BattMonitor_TYPE_BEBOP                      = 6,
        BattMonitor_TYPE_MAXELL                     = 7
    };

    // low voltage sources (used for BATT_LOW_TYPE parameter)
    enum BattMonitor_LowVoltage_Source {
        BattMonitor_LowVoltageSource_Raw            = 0,
        BattMonitor_LowVoltageSource_SagCompensated = 1
    };

    struct cells {
        uint16_t cells[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN];
    };

    // The BattMonitor_State structure is filled in by the backend driver
    struct BattMonitor_State {
        uint8_t     instance;           // the instance number of this monitor
        bool        healthy;            // battery monitor is communicating correctly
        bool        is_powering_off;    // true if the battery is about to power off
        float       voltage;            // voltage in volts
        float       current_amps;       // current in amperes
        float       current_total_mah;  // total current draw since start-up
        uint32_t    last_time_micros;   // time when voltage and current was last read
        uint32_t    low_voltage_start_ms;  // time when voltage dropped below the minimum
        cells       cell_voltages;      // battery cell voltages in millivolts, 10 cells matches the MAVLink spec
        float       temperature;        // battery temperature in celsius
        uint32_t    temperature_time;   // timestamp of the last recieved temperature message
        float       voltage_resting_estimate; // voltage with sag removed based on current and resistance estimate
        float       resistance;         // resistance calculated by comparing resting voltage vs in flight voltage
    };

    // Return the number of battery monitor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz
    void read();

#define _BattMonitor_STATE(instance) state[instance]

    // healthy - returns true if monitor is functioning
    bool healthy(uint8_t instance) const;
    bool healthy() const { return healthy(AP_BATT_PRIMARY_INSTANCE); }

    bool is_powering_off(uint8_t instance) const;
    bool is_powering_off() const { return is_powering_off(AP_BATT_PRIMARY_INSTANCE); }

    /// has_current - returns true if battery monitor instance provides current info
    bool has_current(uint8_t instance) const;
    bool has_current() const { return has_current(AP_BATT_PRIMARY_INSTANCE); }

    /// voltage - returns battery voltage in millivolts
    float voltage(uint8_t instance) const;
    float voltage() const { return voltage(AP_BATT_PRIMARY_INSTANCE); }

    /// get voltage with sag removed (based on battery current draw and resistance)
    /// this will always be greater than or equal to the raw voltage
    float voltage_resting_estimate(uint8_t instance) const;
    float voltage_resting_estimate() const { return voltage_resting_estimate(AP_BATT_PRIMARY_INSTANCE); }

    /// current_amps - returns the instantaneous current draw in amperes
    float current_amps(uint8_t instance) const;
    float current_amps() const { return current_amps(AP_BATT_PRIMARY_INSTANCE); }

    /// current_total_mah - returns total current drawn since start-up in amp-hours
    float current_total_mah(uint8_t instance) const;
    float current_total_mah() const { return current_total_mah(AP_BATT_PRIMARY_INSTANCE); }

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    virtual uint8_t capacity_remaining_pct(uint8_t instance) const;
    uint8_t capacity_remaining_pct() const { return capacity_remaining_pct(AP_BATT_PRIMARY_INSTANCE); }

    /// pack_capacity_mah - returns the capacity of the battery pack in mAh when the pack is full
    int32_t pack_capacity_mah(uint8_t instance) const;
    int32_t pack_capacity_mah() const { return pack_capacity_mah(AP_BATT_PRIMARY_INSTANCE); }
 
    /// exhausted - returns true if the battery's voltage remains below the low_voltage for 10 seconds or remaining capacity falls below min_capacity
    bool exhausted(uint8_t instance, float low_voltage, float min_capacity_mah);
    bool exhausted(float low_voltage, float min_capacity_mah) { return exhausted(AP_BATT_PRIMARY_INSTANCE, low_voltage, min_capacity_mah); }

    /// get_type - returns battery monitor type
    enum BattMonitor_Type get_type() { return get_type(AP_BATT_PRIMARY_INSTANCE); }
    enum BattMonitor_Type get_type(uint8_t instance) { return (enum BattMonitor_Type)_monitoring[instance].get(); }

    /// set_monitoring - sets the monitor type (used for example sketch only)
    void set_monitoring(uint8_t instance, uint8_t mon) { _monitoring[instance].set(mon); }

    bool get_watt_max() { return get_watt_max(AP_BATT_PRIMARY_INSTANCE); }
    bool get_watt_max(uint8_t instance) { return _watt_max[instance]; }

    /// true when (voltage * current) > watt_max
    bool overpower_detected() const;
    bool overpower_detected(uint8_t instance) const;

    // cell voltages
    bool has_cell_voltages() { return has_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    bool has_cell_voltages(const uint8_t instance) const;
    const cells & get_cell_voltages() const { return get_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    const cells & get_cell_voltages(const uint8_t instance) const;

    // temperature
    bool get_temperature(float &temperature) const { return get_temperature(temperature, AP_BATT_PRIMARY_INSTANCE); };
    bool get_temperature(float &temperature, const uint8_t instance) const;

    // get battery resistance estimate in ohms
    float get_resistance() const { return get_resistance(AP_BATT_PRIMARY_INSTANCE); }
    float get_resistance(uint8_t instance) const { return state[instance].resistance; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_Int8     _monitoring[AP_BATT_MONITOR_MAX_INSTANCES];         /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8     _volt_pin[AP_BATT_MONITOR_MAX_INSTANCES];           /// board pin used to measure battery voltage
    AP_Int8     _curr_pin[AP_BATT_MONITOR_MAX_INSTANCES];           /// board pin used to measure battery current
    AP_Float    _volt_multiplier[AP_BATT_MONITOR_MAX_INSTANCES];    /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float    _curr_amp_per_volt[AP_BATT_MONITOR_MAX_INSTANCES];  /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float    _curr_amp_offset[AP_BATT_MONITOR_MAX_INSTANCES];    /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Int32    _pack_capacity[AP_BATT_MONITOR_MAX_INSTANCES];      /// battery pack capacity less reserve in mAh
    AP_Int16    _watt_max[AP_BATT_MONITOR_MAX_INSTANCES];           /// max battery power allowed. Reduce max throttle to reduce current to satisfy this limit
    AP_Int32    _serial_numbers[AP_BATT_MONITOR_MAX_INSTANCES];     /// battery serial number, automatically filled in on SMBus batteries
    AP_Int8     _low_voltage_timeout;                               /// timeout in seconds before a low voltage event will be triggered
    AP_Int8     _low_voltage_source;                                /// voltage type used for detection of low voltage event

private:
    BattMonitor_State state[AP_BATT_MONITOR_MAX_INSTANCES];
    AP_BattMonitor_Backend *drivers[AP_BATT_MONITOR_MAX_INSTANCES];
    uint8_t     _num_instances;                                     /// number of monitors
};
