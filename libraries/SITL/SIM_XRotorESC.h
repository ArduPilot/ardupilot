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
  Simulator for the serial telemetry from a HobbyWing XRotor or T-Motor
  Alpha ESC, one ESC per serial port. The argument is the output the
  ESC drives, starting at 1:

./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --serial5=sim:xrotor:1 --serial6=sim:xrotor:3

param set SERIAL5_PROTOCOL 52
param set SERIAL6_PROTOCOL 52
param set ESC_TLM_SER_MASK 5
param set SIM_ESC_TELEM 0
param set SIM_VIB_MOT_MAX 350
reboot

  SIM_ESC_TELEM 0 stops SITL's own ESC telemetry, and a multicopter
  only simulates motor RPM when SIM_VIB_MOT_MAX is set.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_XROTOR_ESC_ENABLED

#include "SIM_SerialDevice.h"

namespace SITL {

class XRotorESC : public SerialDevice {
public:

    // servo_channel is the output the ESC drives, starting at 1
    explicit XRotorESC(uint8_t _servo_channel) :
        servo_channel{_servo_channel}
    {}

    // send a telemetry frame when one is due
    void update(const class Aircraft &aircraft);

    // the ESC sends at a fixed rate
    uint32_t device_baud() const override { return 19200; }

private:

    uint8_t servo_channel;      // output the ESC drives, starting at 1
    uint16_t frame_count;       // sent in each frame, then incremented
    uint32_t last_frame_ms;     // when the last frame was sent
};

}

#endif  // AP_SIM_XROTOR_ESC_ENABLED
