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

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:rplidara2 --speedup=1 ./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:rplidara2 --speedup=1 -l 51.8752066,14.6487840,0,0

param set SERIAL5_PROTOCOL 11
param set PRX_TYPE 5
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

#include <stdio.h>

namespace SITL {

class PS_RPLidarA2 : public SerialProximitySensor {
public:

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    void update(const Location &location) override;

private:

    void update_input();
    void update_output(const Location &location);
    void update_output_scan(const Location &location);

    uint32_t last_scan_output_time_ms;

    float last_degrees_bf;

    char _buffer[256]; // from-autopilot
    uint8_t _buflen;

    // TODO: see what happens on real device on partial input
    enum class State {
        IDLE = 17,
        SCANNING = 18,
    };
    State _state = State::IDLE;
    void set_state(State newstate) {
        ::fprintf(stderr, "Moving from state (%u) to (%u)\n", (uint8_t)_state, (uint8_t)newstate);
        _state = newstate;
    }

    enum class InputState {
        WAITING_FOR_PREAMBLE = 45,
        GOT_PREAMBLE = 46,
        RESETTING_START = 47,
        RESETTING_SEND_FIRMWARE_INFO = 48,
    };
    InputState _inputstate = InputState::WAITING_FOR_PREAMBLE;
    void set_inputstate(InputState newstate) {
        ::fprintf(stderr, "Moving from inputstate (%u) to (%u)\n", (uint8_t)_state, (uint8_t)newstate);
        _inputstate = newstate;
    }

    static const uint8_t PREAMBLE = 0xA5;

    enum class Command {
        STOP = 0x25,
        SCAN = 0x20,
        FORCE_SCAN = 0x21,
        RESET = 0x40,
        GET_HEALTH = 0x52,
    };

    void move_preamble_in_buffer();

    enum class DataType {
        Unknown06 = 0x06, // uint8_t ?!
        Unknown81 = 0x81, // uint8_t ?!
    };

    enum class SendMode {
        SRSR = 0x00,
        SRMR = 0x40,  // no idea why this isn't 0x01
    };

    void send_response_descriptor(uint32_t data_response_length,
                                  SendMode sendmode,
                                  DataType datatype);


    // the driver expects to see an "R" followed by 62 bytes more crap.
    static const constexpr char *FIRMWARE_INFO = "R12345678901234567890123456789012345678901234567890123456789012";
    uint8_t _firmware_info_offset;
};

};
