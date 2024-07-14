/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Charlie Johnson
 */

#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_FUELLEVEL_ANALOG_ENABLED

#include <Filter/LowPassFilter.h>
#include "AP_BattMonitor.h"
#include <Filter/Filter.h>             // Filter library

class AP_BattMonitor_FuelLevel_Analog : public AP_BattMonitor_Backend
{
public:

    // Constructor
    AP_BattMonitor_FuelLevel_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    // Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    // returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    // returns true if battery monitor provides current info (or capacity)
    bool has_current() const override { return true; }

    void init(void) override {}

private:

    AP_Float _fuel_level_empty_voltage;
    AP_Float _fuel_level_voltage_mult;
    AP_Float _fuel_level_filter_frequency;
    AP_Int8  _pin;
    AP_Float _fuel_fit_first_order_coeff;
    AP_Float _fuel_fit_second_order_coeff;
    AP_Float _fuel_fit_third_order_coeff;
    AP_Float _fuel_fit_offset;
    AP_HAL::AnalogSource *_analog_source;

    LowPassFilterFloat _voltage_filter;

};

#endif  // AP_BATTERY_FUELLEVEL_ANALOG_ENABLED
