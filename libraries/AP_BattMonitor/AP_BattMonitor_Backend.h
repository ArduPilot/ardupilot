/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_BattMonitor.h"

class AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    // we declare a virtual destructor so that BattMonitor driver can
    // override with a custom destructor if need be
    virtual ~AP_BattMonitor_Backend(void) {}

    // initialise
    virtual void init() {};

    // read the latest battery voltage
    virtual void read() = 0;

    /// returns true if battery monitor instance provides time remaining info
    virtual bool has_time_remaining() const { return false; }

    /// returns true if battery monitor instance provides consumed energy info
    virtual bool has_consumed_energy() const { return false; }

    /// returns true if battery monitor instance provides current info
    virtual bool has_current() const = 0;

    // returns true if battery monitor provides individual cell voltages
    virtual bool has_cell_voltages() const { return false; }

    // returns true if battery monitor provides temperature
    virtual bool has_temperature() const { return false; }

    // capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
    // returns false if the battery is unhealthy, does not have current monitoring, or the pack_capacity is too small
    virtual bool capacity_remaining_pct(uint8_t &percentage) const WARN_IF_UNUSED;

    // return true if cycle count can be provided and fills in cycles argument
    virtual bool get_cycle_count(uint16_t &cycles) const { return false; }

    /// get voltage with sag removed (based on battery current draw and resistance)
    /// this will always be greater than or equal to the raw voltage
    float voltage_resting_estimate() const;

    // update battery resistance estimate and voltage_resting_estimate
    void update_resistance_estimate();

    // updates failsafe timers, and returns what failsafes are active
    virtual AP_BattMonitor::Failsafe update_failsafes(void);

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(char * buffer, size_t buflen) const;

    // reset remaining percentage to given value
    virtual bool reset_remaining(float percentage);

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    virtual uint32_t get_mavlink_fault_bitmask() const { return 0; }

    // logging functions 
    void Log_Write_BAT(const uint8_t instance, const uint64_t time_us) const;
    void Log_Write_BCL(const uint8_t instance, const uint64_t time_us) const;

    // set desired MPPT powered state (enabled/disabled)
    virtual void mppt_set_powered_state(bool power_on) {};

    // amps: current (A)
    // dt_us: time between samples (micro-seconds)
    static float calculate_mah(float amps, float dt_us) { return (float) (amps * dt_us * AUS_TO_MAH); }

    // check if a option is set
    bool option_is_set(const AP_BattMonitor_Params::Options option) const {
        return (uint16_t(_params._options.get()) & uint16_t(option)) != 0;
    }

protected:
    AP_BattMonitor                      &_mon;      // reference to front-end
    AP_BattMonitor::BattMonitor_State   &_state;    // reference to this instances state (held in the front-end)
    AP_BattMonitor_Params               &_params;   // reference to this instances parameters (held in the front-end)

    // checks what failsafes could be triggered
    void check_failsafe_types(bool &low_voltage, bool &low_capacity, bool &critical_voltage, bool &critical_capacity) const;
    void update_consumed(AP_BattMonitor::BattMonitor_State &state, uint32_t dt_us);

private:
    // resistance estimate
    uint32_t    _resistance_timer_ms;    // system time of last resistance estimate update
    float       _voltage_filt;           // filtered voltage
    float       _current_max_amps;       // maximum current since start-up
    float       _current_filt_amps;      // filtered current
    float       _resistance_voltage_ref; // voltage used for maximum resistance calculation
    float       _resistance_current_ref; // current used for maximum resistance calculation
};
