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
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_ESC_ENABLED

class AP_BattMonitor_ESC :public AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_BattMonitor_ESC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    virtual ~AP_BattMonitor_ESC(void) {};

    // initialise
    void init() override;

    // read the latest battery voltage
    void read() override;

    // ESC_Telem provides current info
    bool has_current() const override { return have_current; };

    // ESC_Telem provides temperature info
    bool has_temperature() const override { return have_temperature; };

    // reset remaining percentage to given value
    virtual bool reset_remaining(float percentage) override;

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int32  _mask;

    bool have_current;
    bool have_consumed_mah;
    bool have_temperature;
    float delta_mah;

    uint32_t last_read_us;
};

#endif  // AP_BATTERY_ESC_ENABLED
