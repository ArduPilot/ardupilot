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
  Replay serial capture files

# if uart2_000.log is a GPS log:
./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial3=sim:capreplay:uart2_000.log --speedup=1 --console

reboot

module load console

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"
#include "SIM_GeneratorEngine.h"

namespace SITL {

class CapReplay : public SerialDevice {
public:

    CapReplay(const char *arg);

    // update state
    void update();

private:

    char filepath[64];
    int _fd = -2;

    // copy of structure from cpp code:
    struct PACKED Header {
        enum class Flag {
            IS_WRITTEN_DATA = 1,
        };
        uint32_t magic = 0xEAEF0D0F;
        uint32_t time_ms;
        uint16_t length;
        uint8_t flags;
    } header;

    // added to the time_ms in the data structures.  When we replay
    // the log allows timing information to continue to be used:
    uint32_t base_time_ms;
    void rewind();  // restart from the start
};

}
