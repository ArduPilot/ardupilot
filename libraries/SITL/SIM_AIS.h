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
    Dump logged AIS data to the serial port
    ./Tools/autotest/sim_vehicle.py -v Rover --no-mavproxy -A --uartF=sim:AIS --custom-location 51.58689798356386,-3.9044570193067965,0,0

    param set SERIAL5_PROTOCOL 40
    param set AIS_TYPE 1
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_AIS_ENABLED
#define HAL_SIM_AIS_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_AIS_ENABLED

#include "SIM_SerialDevice.h"
#include <SITL/SITL.h>

namespace SITL {

class AIS : public SerialDevice {
public:

    AIS();

    void update();

private:
    FILE* file;

    uint32_t last_sent_ms;

};

}

#endif // HAL_SIM_AIS_ENABLED
