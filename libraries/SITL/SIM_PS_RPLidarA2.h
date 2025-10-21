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
  Simulator for the RPLidarA2 proximity sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:rplidara2 --speedup=1 -l 51.8752066,14.6487840,0,0 --map

param set SERIAL5_PROTOCOL 11
param set PRX1_TYPE 5
reboot

arm throttle
rc 3 1600

# for avoidance:
param set DISARM_DELAY 0
param set AVOID_ENABLE 2 # use proximity sensor
param set AVOID_MARGIN 2.00  # 2m
param set AVOID_BEHAVE 0 # slide
param set OA_DB_OUTPUT 3
param set OA_TYPE 2
reboot
mode loiter
script /tmp/post-locations.scr
arm throttle
rc 3 1600
rc 3 1500
rc 2 1450

*/

#pragma once

#include "SIM_config.h"

#if HAL_SIM_PS_RPLIDARA2_ENABLED

#include "SIM_PS_RPLidar.h"

namespace SITL {

class PS_RPLidarA2 : public PS_RPLidar {
public:
    uint8_t device_info_model() const override { return 0x28; }
    uint8_t max_range() const override { return 16.0f; };
};

}

#endif  // HAL_SIM_PS_RPLIDARA2_ENABLED
