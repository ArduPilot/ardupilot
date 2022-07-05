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
  Simulator for the TeraRangerTower proximity sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:terarangertower --speedup=1 -l 51.8752066,14.6487840,54.15,0

param set SERIAL5_PROTOCOL 11
param set PRX1_TYPE 3  # terarangertower
reboot

arm throttle
rc 3 1600

# for avoidance:
param set DISARM_DELAY 0
param set AVOID_ENABLE 2 # use proximity sensor
param set AVOID_MARGIN 2.00  # 2m
param set AVOID_BEHAVE 0 # slide
reboot
mode loiter
script /tmp/post-locations.scr
arm throttle
rc 3 1600
rc 3 1500
rc 2 1450

*/

#pragma once

#include "SIM_SerialProximitySensor.h"

#ifndef HAL_SIM_PS_TERARANGERTOWER_ENABLED
#define HAL_SIM_PS_TERARANGERTOWER_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#if HAL_SIM_PS_TERARANGERTOWER_ENABLED

#include <stdio.h>

namespace SITL {

class PS_TeraRangerTower : public SerialProximitySensor {
public:

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    void update(const Location &location) override;
    void update_output(const Location &location);
    void update_output_scan(const Location &location);

private:
    const float MAX_RANGE = 4.5;  // metres

    uint32_t last_output_time_ms;
};

};

#endif  // HAL_SIM_PS_TERARANGERTOWER_ENABLED
