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
  ESC telemetry from HobbyWing XRotor and T-Motor Alpha ESCs, each
  sending on its own serial port
 */

#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_ESC_TELEM_XROTOR_ENABLED

#include "AP_ESC_Telem_Backend.h"
#include "AP_ESC_Telem_XRotor_Parser.h"

#include <AP_HAL/AP_HAL.h>

class AP_ESC_Telem_XRotor : public AP_ESC_Telem_Backend {
public:
    // pair the serial ports set to XRotor telemetry with the outputs in servo_mask; false if out of memory
    bool init(uint32_t servo_mask);

    // read every port and publish the frames received; poles is the motor pole count
    void update(int8_t poles);

    // the first output in servo_mask left without telemetry, counting from 1, or 0 if none
    uint8_t get_unpaired_output() const { return unpaired_output; }

private:
    // one ESC and the serial port its telemetry arrives on
    struct ESC {
        AP_HAL::UARTDriver *uart;           // port the ESC sends on
        uint8_t esc_index;                  // output number - 1
        AP_ESC_Telem_XRotor_Parser parser;  // frames received from this ESC
    };

    ESC *escs;                  // num_escs entries, allocated in init()
    uint8_t num_escs;           // ESCs paired with a port
    uint8_t unpaired_output;    // see get_unpaired_output()

    // publish the frame just received from esc, for motors with this many poles
    void publish(const ESC &esc, float poles);
};

#endif  // AP_ESC_TELEM_XROTOR_ENABLED
