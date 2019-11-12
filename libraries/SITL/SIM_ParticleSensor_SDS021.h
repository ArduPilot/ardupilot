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
  simple particle sensor simulation

./Tools/autotest/sim_vehicle.py --gdb --debug -v APMrover2 -A --uartF=sim:sds021 --speedup=1

param set SERIAL5_PROTOCOL 24
reboot

*/

#pragma once

#include "SIM_SerialDevice.h"

namespace SITL {

class ParticleSensor_SDS021 : public SerialDevice {
public:

    ParticleSensor_SDS021();

    // update state
    void update(const Location &loc);

private:

    uint32_t last_update_ms;

    uint8_t checksum;

    bool push_byte(const uint8_t byte);
    void push_reading(const float value10, const float value25, const bool should_corrupt);
};

}
