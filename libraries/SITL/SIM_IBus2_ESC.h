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

  Each instance has a DEV parameter (SIM_IBUS2E1_DEV, SIM_IBUS2E2_DEV, …) that
  selects the device-name suffix used on the command line.  Instance 0 defaults
  to DEV=0 (device string "ibus2esc0") and instance 1 defaults to DEV=1
  ("ibus2esc1"), giving "works by default" behaviour without configuration.

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter \
  -A --serial5=sim:ibus2esc0 -A --serial6=sim:ibus2esc1 --speedup=1 --console

param set SERIAL5_PROTOCOL 51
param set SERIAL6_PROTOCOL 51
param set IBUS2M_1_ENABLE 1
param set IBUS2M_2_ENABLE 1
param set IBUS2M_1_SMASK 0
param set IBUS2M_2_SMASK 0
param set SIM_IBUS2E1_ENA 1
param set SIM_IBUS2E2_ENA 1
reboot
*/

#pragma once

#include <AP_IBus2/AP_IBus2_config.h>
#if AP_IBUS2_ENABLED

#include "SIM_IBus2_Slave.h"
#include "SITL_Input.h"

namespace SITL {

class IBus2ESC : public IBus2Slave {
public:
    // dev_idx_default sets the default value of the DEV param so that
    // each array element in SITL.h has a distinct default without runtime config.
    explicit IBus2ESC(uint8_t dev_idx_default = 0) : _dev_idx_default(dev_idx_default) {}

    void update_sitl_input_pwm(struct sitl_input &input);

    static const AP_Param::GroupInfo var_info[];

    AP_Int8 _dev_idx;  // device-name suffix; create_serial_sim matches "ibus2escN" where N==_dev_idx

private:
    const float _dev_idx_default;  // default for _dev_idx, set in constructor

    // ESCs do not send telemetry responses
    void send_response(const class Aircraft &) override {}
};

} // namespace SITL

#endif  // AP_IBUS2_ENABLED
