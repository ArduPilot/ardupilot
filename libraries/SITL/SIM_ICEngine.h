/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  simple internal combustion motor simulation class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

class ICEngine {
public:
    const uint8_t throttle_servo;
    const int8_t choke_servo;
    const int8_t ignition_servo;
    const int8_t starter_servo;
    const float slew_rate; // percent-per-second

    ICEngine(uint8_t _throttle, int8_t _choke, int8_t _ignition, int8_t _starter, float _slew_rate) :
        throttle_servo(_throttle),
        choke_servo(_choke),
        ignition_servo(_ignition),
        starter_servo(_starter),
        slew_rate(_slew_rate)
    {}

    // update motor state
    float update(const struct Aircraft::sitl_input &input);

private:
    float last_output;
    uint64_t start_time_us;
    uint64_t last_update_us;
    union state {
        struct {
            bool choke:1;
            bool ignition:1;
            bool starter:1;
        };
        uint8_t value;
    } state, last_state;
    bool overheat:1;
};
}
