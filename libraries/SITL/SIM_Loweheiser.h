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
  Simulator for the Loweheiser EFI/generator

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduPlane -A --uartF=sim:loweheiser --speedup=1 --console

param set SERIAL5_PROTOCOL 2
param set SERIAL5_OPTIONS 1024  # private
param set GEN_TYPE 4
param set EFI_TYPE 3

param set SIM_EFI_TYPE 2

reboot


# observe RPM
module load generator
long SET_MESSAGE_INTERVAL 373 1000000

./Tools/autotest/autotest.py --gdb --debug build.Copter test.Copter.Loweheiser

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

#include <GCS_MAVLink/GCS_MAVLink.h>

#include <stdio.h>

namespace SITL {

class Loweheiser : public SerialDevice {
public:

    Loweheiser();

    // update state
    void update();

private:

    // TODO: make these parameters:
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    // we share channels with the ArduPilot binary!
    // Beware: the mavlink rangefinder and other stuff shares this channel.
    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);

    class SIM *_sitl;

    uint32_t last_sent_ms;

    void update_send();

    void maybe_send_heartbeat();
    uint32_t last_heartbeat_ms;

    // parameters
    // AP_Int8 _enabled;
};

}
