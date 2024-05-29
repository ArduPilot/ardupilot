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
  Simulator for the Ainstein LR-D1 serial rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:ainsteinlrd1 --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 42
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600

# to pass canned data through the simulator:
/Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=file:libraries/AP_RangeFinder/examples/captures/ainstein_lr-d1.bin --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 42
reboot

*/

#pragma once

#include "SIM_SerialRangeFinder.h"

namespace SITL {

class RF_Ainstein_LR_D1 : public SerialRangeFinder {
public:

    uint32_t packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen) override;

    uint16_t reading_interval_ms() const override { return 100; }

private:

};

}
