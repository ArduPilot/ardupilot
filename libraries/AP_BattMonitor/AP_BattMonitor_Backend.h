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

#if HAL_SMART_BATTERY_INFO_ENABLED
    // returns true if product name can be filled in from either manufacturer name and device name
    virtual bool get_product_name(char *product_name, uint8_t buflen) const { return false; }

    // returns true if design_capacity in mAh (capacity when newly manufactured) can be provided and fills it in
    virtual bool get_design_capacity_mah(float &design_capacity) const { return false; }

    // returns true if the full charge capacity in mAh (accounting for battery degradation) can be provided and fills it in
    virtual bool get_full_charge_capacity_mah(float &full_capacity) const { return false; }

    // returns true if the design voltage in volts (maximum charging voltage) can be provided and fills it in
    virtual bool get_design_voltage(float &design_voltage) const { return false; }

    // returns true if the manufacture date can be provided and fills it in
    virtual bool get_manufacture_date(char *manufacture_date, uint8_t buflen) const { return false; }

    // returns true if the number of cells in series can be provided and fills it in
    virtual bool get_cells_in_series(uint8_t &cells_in_series) const { return false; }
#endif

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
    bool Log_Write_BATI(uint8_t instance) const;

    // Send the mavlink message for the smart_battery_info message
    // returns false only if an instance didn't have enough space available on the link
    bool send_mavlink_smart_battery_info(const uint8_t instance, const mavlink_channel_t chan) const;

    // amps: current (A)
    // dt_us: time between samples (micro-seconds)
    static float calculate_mah(float amps, float dt_us) { return (float) (amps * dt_us * AUS_TO_MAH); }

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

    struct Smart_Batt_Info{
        char        product_name[MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN];             // Static device name. Encode as manufacturer and product names separated using an underscore. First Char \0: field not provided
        char        serial_number[MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN];          // Serial number in ASCII characters, 0 terminated. First Char \0: field not provided
        char        manufacture_date[MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN];    // Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. First Char \0: field not provided
        float       capacity_design_mah;    // [mAh] design capacity when full according to manufacturer
        float       capacity_full_mah;      // [mAh] capacity when full (accounting for battery degradation)
        float       design_voltage;         // [V] maximum charging voltage
        uint16_t    cycles;                 // charge/discharge cycle count
        uint8_t     cells_in_series;        // number of battery cells in series
        bool        got_cells_in_series;
        bool        got_cycle_count;
        bool        got_capacity_design;
        bool        got_capacity_full;
        bool        got_design_voltage;
    };

    // Smart battery info. helper to gather data for logging and mavlink
    // returns false if cells_in_series is not available
    bool get_smart_batt_info(Smart_Batt_Info &smart_batt_info) const;
};
