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
  Simulated CRSF device

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:crsf --speedup=1

param set SERIAL5_PROTOCOL 23
reboot

arm throttle
rc 3 1600

*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SIM_CRSF_ENABLED
#define AP_SIM_CRSF_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_SIM_CRSF_ENABLED

#include "SIM_Aircraft.h"
#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class CRSF : public SerialDevice {
public:

    CRSF() {};

    // update state
    void update();

protected:
    enum DataID {
        VTX_FRAME           = 0x00,
        VTX_TELEM           = 0x01,
        VTX_UNKNOWN         = 0x02,
        MAX_DATA_FRAMES     = 0x03
    };

    const char* dataid_string(DataID id, ssize_t& len);

    char _buffer[64];
    uint16_t _buflen = 0;
    uint8_t _id;
    uint32_t _last_update_ms;
};

}

#endif  // AP_SIM_CRSF_ENABLED
