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

   Base class for RPLidar support
 */
/*
  Simulator for the RPLidar proximity sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:rplidara2 --speedup=1 -l 51.8752066,14.6487840,54.15,0 --map

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

#if AP_SIM_PS_RPLIDARA2_ENABLED || AP_SIM_PS_RPLIDARA1_ENABLED || AP_SIM_PS_RPLIDARS2_ENABLED

#include "SIM_SerialProximitySensor.h"

#include <stdio.h>

namespace SITL {

class PS_RPLidar : public SerialProximitySensor {
public:

    using SerialProximitySensor::SerialProximitySensor;

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    void update(const Location &location) override;

private:

    void update_input();
    void update_output(const Location &location);

    // 5-byte scan output
    void update_output_scan(const Location &location);

    // dense express scan output (40 samples per packet)
    void update_output_express_dense(const Location &location);

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
        ::fprintf(stderr, "Moving from inputstate (%u) to (%u)\n", (uint8_t)_inputstate, (uint8_t)newstate);
        _inputstate = newstate;
    }

    static const uint8_t PREAMBLE = 0xA5;

    enum class Command : uint8_t {
        STOP = 0x25,
        SCAN = 0x20,
        FORCE_SCAN = 0x21,
        RESET = 0x40,
        GET_DEVICE_INFO = 0x50,
        GET_HEALTH = 0x52,
        EXPRESS_SCAN = 0x82,
    };

    void move_preamble_in_buffer();

    enum class DataType : uint8_t {
        Unknown04 = 0x04,   // device info
        Unknown06 = 0x06,   // health
        Unknown81 = 0x81,   // scan (5-byte)
        Unknown85 = 0x85    // dense express scan
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

    // scan type
    enum class ScanMode {
        SCAN = 0,
        EXPRESS_SCAN_DENSE = 1,
    };
    ScanMode _scan_mode = ScanMode::SCAN;

    // state for dense express packets (current start angle in degrees)
    float _express_w_i_deg = 0.0f;

    // methods for sub-classes to implement:
    virtual uint8_t device_info_model() const = 0;
    virtual uint8_t max_range() const = 0;
};

};

#endif  // AP_SIM_PS_RPLIDARA2_ENABLED || AP_SIM_PS_RPLIDARA1_ENABLED || AP_SIM_PS_RPLIDARS2_ENABLED
