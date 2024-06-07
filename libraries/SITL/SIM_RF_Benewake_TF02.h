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
  Simulator for the Benewake TF02 RangeFinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:benewake_tf02 --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 19
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600

*/

#pragma once

#include "SIM_RF_Benewake.h"

namespace SITL {

class RF_Benewake_TF02 : public RF_Benewake {
public:

    // see AP_RangeFinder_Benewake.cpp for definitions
    uint8_t byte4() const override { return 1; } // strength low-bits
    uint8_t byte5() const override { return 1; } // strength high-bits
    uint8_t byte6() const override { return 7; } // reliability
    uint8_t byte7() const override { return 0x06; } // exposure time

};

}
