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
  Simulated Frsky D device

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:frsky-d --speedup=1

param set SERIAL5_PROTOCOL 3
reboot

arm throttle
rc 3 1600

*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include "SIM_Frsky.h"

namespace SITL {

class Frsky_D : public Frsky {
public:

    using Frsky::Frsky;

    // update state
    virtual void update() override;

private:

    enum class State {
        WANT_START_STOP_D = 16,
        WANT_ID = 17,
        WANT_BYTE1 = 18,
        WANT_BYTE2 = 19,
    };
    State _state = State::WANT_START_STOP_D;

    char _buffer[32];
    uint16_t _buflen = 0;

    uint8_t _id;
    uint16_t _data;


    void handle_data(uint8_t id, uint16_t data);

    void shift_start_stop_d();
};

}
