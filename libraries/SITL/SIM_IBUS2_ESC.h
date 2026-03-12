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
  Simulator for IBUS2 ESC devices receiving Frame 1 motor commands.

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter \
  -A --serial5=sim:ibus2esc --speedup=1 --console

param set SERIAL5_PROTOCOL 51
param set IBUS2M_ENABLE 1
param set IBUS2M_SEND_MASK 0
param set SIM_IBUS2E_ENA 1
reboot
*/

#pragma once

#include <AP_IBUS2/AP_IBUS2_config.h>
#if AP_IBUS2_ENABLED

#include "SIM_IBUS2_Slave.h"
#include "SITL_Input.h"

namespace SITL {

class IBUS2ESC : public IBUS2Slave {
public:
    IBUS2ESC() = default;

    void update_sitl_input_pwm(struct sitl_input &input);

    static const AP_Param::GroupInfo var_info[];

private:
    // ESCs do not send telemetry responses
    void send_response(const class Aircraft &) override {}
};

} // namespace SITL

#endif  // AP_IBUS2_ENABLED
