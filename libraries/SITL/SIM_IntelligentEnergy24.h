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
/*
  Simulator for the IntelligentEnergy 2.4kW FuelCell

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:ie24 --speedup=1 --console

param set SERIAL5_PROTOCOL 30  # Generator
param set SERIAL5_BAUD 115200
param set GEN_TYPE 2  # IE24
param set BATT3_MONITOR 17  # electrical generator
param set BATT2_MONITOR 18  # fuel-based generator
param set SIM_IE24_ENABLE 1
param fetch

graph BATTERY_STATUS[0].voltages[0]/1000.0
graph BATTERY_STATUS[1].voltages[0]/1000.0

reboot

# TODO: ./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.IntelligentEnergy24

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_IntelligentEnergy.h"

#include <stdio.h>

namespace SITL {

class IntelligentEnergy24 : public IntelligentEnergy {
public:

    IntelligentEnergy24();

    // update state
    void update(const struct sitl_input &input) override;

    static const AP_Param::GroupInfo var_info[];

private:

    void update_send();

    AP_Int8 enabled;  // enable sim
    AP_Int8 set_state;
    AP_Int32 err_code;

    float battery_voltage = 50.4f;
    float bat_capacity_mAh = 3300;
    bool discharge = true; // used to switch between battery charging and discharging
    uint32_t last_sent_ms;

};

}
