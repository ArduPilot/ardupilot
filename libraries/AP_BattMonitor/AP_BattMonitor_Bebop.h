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
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_Bebop :public AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_BattMonitor_Bebop(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params),
        _prev_vbat_raw(0.0f),
        _prev_vbat(0.0f),
        _battery_voltage_max(0.0f)
    {};

    virtual ~AP_BattMonitor_Bebop(void) {};

    // initialise
    void init() override;

    // read the latest battery voltage
    void read() override;

    // bebop provides current info
    bool has_current() const override { return true; };

    // don't allow reset of remaining capacity for bebop battery
    bool reset_remaining(float percentage) override { return false; }

private:
    float _prev_vbat_raw;
    float _prev_vbat;
    float _battery_voltage_max;
    float _compute_compensation(const uint16_t *rpm, float vbat_raw);
    float _filter_voltage(float vbat_raw);
    float _compute_battery_percentage(float vbat);
};
