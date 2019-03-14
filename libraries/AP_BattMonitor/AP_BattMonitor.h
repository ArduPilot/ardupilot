#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_BattMonitor_Params.h"

// maximum number of battery monitors
#define AP_BATT_MONITOR_MAX_INSTANCES       9

// first monitor is always the primary monitor
#define AP_BATT_PRIMARY_INSTANCE            0

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
class AP_BattMonitor_UAVCAN;

class AP_BattMonitor
{
    friend class AP_BattMonitor_Backend;
    friend class AP_BattMonitor_Analog;
    friend class AP_BattMonitor_SMBus;
    friend class AP_BattMonitor_SMBus_Solo;
    friend class AP_BattMonitor_SMBus_Maxell;
    friend class AP_BattMonitor_UAVCAN;
    friend class AP_BattMonitor_Sum;

public:

    // battery failsafes must be defined in levels of severity so that vehicles wont fall backwards
    enum BatteryFailsafe {
        BatteryFailsafe_None = 0,
        BatteryFailsafe_Low,
        BatteryFailsafe_Critical
    };

    FUNCTOR_TYPEDEF(battery_failsafe_handler_fn_t, void, const char *, const int8_t);

    AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities);

    /* Do not allow copies */
    AP_BattMonitor(const AP_BattMonitor &other) = delete;
    AP_BattMonitor &operator=(const AP_BattMonitor&) = delete;

    static AP_BattMonitor *get_singleton() {
        return _singleton;
    }

    struct cells {
        uint16_t cells[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN];
    };

    // The BattMonitor_State structure is filled in by the backend driver
    struct BattMonitor_State {
        cells       cell_voltages;             // battery cell voltages in millivolts, 10 cells matches the MAVLink spec
        float       voltage;                   // voltage in volts
        float       current_amps;              // current in amperes
        float       consumed_mah;              // total current draw in milliamp hours since start-up
        float       consumed_wh;               // total energy consumed in Wh since start-up
        uint32_t    last_time_micros;          // time when voltage and current was last read in microseconds
        uint32_t    low_voltage_start_ms;      // time when voltage dropped below the minimum in milliseconds
        uint32_t    critical_voltage_start_ms; // critical voltage failsafe start timer in milliseconds
        float       temperature;               // battery temperature in degrees Celsius
        uint32_t    temperature_time;          // timestamp of the last received temperature message
        float       voltage_resting_estimate;  // voltage with sag removed based on current and resistance estimate in Volt
        float       resistance;                // resistance, in Ohms, calculated by comparing resting voltage vs in flight voltage
        BatteryFailsafe failsafe;              // stage failsafe the battery is in
        bool        healthy;                   // battery monitor is communicating correctly
        bool        is_powering_off;           // true when power button commands power off
        bool        powerOffNotified;          // only send powering off notification once
    };

    // Return the number of battery monitor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz
    void read();

    // healthy - returns true if monitor is functioning
    bool healthy(uint8_t instance) const;
    bool healthy() const { return healthy(AP_BATT_PRIMARY_INSTANCE); }

    /// has_consumed_energy - returns true if battery monitor instance provides consumed energy info
    bool has_consumed_energy(uint8_t instance) const;
    bool has_consumed_energy() const { return has_consumed_energy(AP_BATT_PRIMARY_INSTANCE); }

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

    /// consumed_mah - returns total current drawn since start-up in milliampere.hours
    float consumed_mah(uint8_t instance) const;
    float consumed_mah() const { return consumed_mah(AP_BATT_PRIMARY_INSTANCE); }

    /// consumed_wh - returns total energy drawn since start-up in watt.hours
    float consumed_wh(uint8_t instance) const;
    float consumed_wh() const { return consumed_wh(AP_BATT_PRIMARY_INSTANCE); }

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    virtual uint8_t capacity_remaining_pct(uint8_t instance) const;
    uint8_t capacity_remaining_pct() const { return capacity_remaining_pct(AP_BATT_PRIMARY_INSTANCE); }

    /// pack_capacity_mah - returns the capacity of the battery pack in mAh when the pack is full
    int32_t pack_capacity_mah(uint8_t instance) const;
    int32_t pack_capacity_mah() const { return pack_capacity_mah(AP_BATT_PRIMARY_INSTANCE); }
 
    /// returns the failsafe state of the battery
    BatteryFailsafe check_failsafe(const uint8_t instance);
    void check_failsafes(void); // checks all batteries failsafes

    /// returns true if a battery failsafe has ever been triggered
    bool has_failsafed(void) const { return _has_triggered_failsafe; };

    /// returns the highest failsafe action that has been triggered
    int8_t get_highest_failsafe_priority(void) const { return _highest_failsafe_priority; };

    /// get_type - returns battery monitor type
    enum AP_BattMonitor_Params::BattMonitor_Type get_type() const { return get_type(AP_BATT_PRIMARY_INSTANCE); }
    enum AP_BattMonitor_Params::BattMonitor_Type get_type(uint8_t instance) const { return _params[instance].type(); }

    /// set_monitoring - sets the monitor type (used for example sketch only)
    void set_monitoring(uint8_t instance, uint8_t mon) { _params[instance]._type.set(mon); }

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

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(size_t buflen, char *buffer) const;

    // sends powering off mavlink broadcasts and sets notify flag
    void checkPoweringOff(void);

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_BattMonitor_Params _params[AP_BATT_MONITOR_MAX_INSTANCES];

private:
    static AP_BattMonitor *_singleton;

    BattMonitor_State state[AP_BATT_MONITOR_MAX_INSTANCES];
    AP_BattMonitor_Backend *drivers[AP_BATT_MONITOR_MAX_INSTANCES];
    uint32_t    _log_battery_bit;
    uint8_t     _num_instances;                                     /// number of monitors

    void convert_params(void);

    battery_failsafe_handler_fn_t _battery_failsafe_handler_fn;
    const int8_t *_failsafe_priorities; // array of failsafe priorities, sorted highest to lowest priority, -1 indicates no more entries

    int8_t      _highest_failsafe_priority; // highest selected failsafe action level (used to restrict what actions we move into)
    bool        _has_triggered_failsafe;  // true after a battery failsafe has been triggered for the first time

};

namespace AP {
    AP_BattMonitor &battery();
};
