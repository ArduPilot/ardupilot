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
#include <AP_HAL/AP_HAL.h>
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
    virtual void init() = 0;

    // read the latest battery voltage
    virtual void read() = 0;

    /// returns true if battery monitor instance provides consumed energy info
    virtual bool has_consumed_energy() const { return false; }

    /// returns true if battery monitor instance provides current info
    virtual bool has_current() const = 0;

    // returns true if battery monitor provides individual cell voltages
    virtual bool has_cell_voltages() const { return false; }

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    uint8_t capacity_remaining_pct() const;

    /// get voltage with sag removed (based on battery current draw and resistance)
    /// this will always be greater than or equal to the raw voltage
    float voltage_resting_estimate() const;

    // update battery resistance estimate and voltage_resting_estimate
    void update_resistance_estimate();

    // updates failsafe timers, and returns what failsafes are active
    AP_BattMonitor::BatteryFailsafe update_failsafes(void);

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(char * buffer, size_t buflen) const;

protected:
    AP_BattMonitor                      &_mon;      // reference to front-end
    AP_BattMonitor::BattMonitor_State   &_state;    // reference to this instances state (held in the front-end)
    AP_BattMonitor_Params               &_params;   // reference to this instances parameters (held in the front-end)

    // checks what failsafes could be triggered
    void check_failsafe_types(bool &low_voltage, bool &low_capacity, bool &critical_voltage, bool &critical_capacity) const;

private:
    // resistance estimate
    uint32_t    _resistance_timer_ms;    // system time of last resistance estimate update
    float       _voltage_filt;           // filtered voltage
    float       _current_max_amps;       // maximum current since start-up
    float       _current_filt_amps;      // filtered current
    float       _resistance_voltage_ref; // voltage used for maximum resistance calculation
    float       _resistance_current_ref; // current used for maximum resistance calculation
};
