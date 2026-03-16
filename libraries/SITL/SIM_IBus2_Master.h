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
  Simulator for an IBUS2 master device (receiver/hub) for testing AP_IBUS2_Slave.

  This simulator acts as a FlySky receiver. It periodically:
    1. Sends Frame 1 with simulated RC channel data
    2. Sends Frame 2 commands, cycling through GET_TYPE/GET_VALUE/GET_PARAM/SET_PARAM
    3. Reads and validates Frame 3 responses from ArduPilot

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter \
  -A --serial5=sim:ibus2master --speedup=1 --console

param set SERIAL5_PROTOCOL 52
param set SERIAL5_BAUD 1500000
param set SIM_IBUS2M_ENA 1
reboot
*/

#pragma once

#include "SIM_SerialDevice.h"
#include <AP_Param/AP_Param.h>
#include <AP_IBus2/AP_IBus2.h>

namespace SITL {

class IBus2Master : public SerialDevice {
public:
    IBus2Master();

    void update(const class Aircraft &aircraft);

    static const AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enabled.get(); }

private:
    AP_Int8 _enabled;

    uint32_t _last_send_us;
    uint8_t  _cmd_cycle;  // cycles through GET_TYPE/GET_VALUE/GET_PARAM/SET_PARAM

    // Frame reception from AP (Frame 3)
#define IBUS2_FRAME3_SIZE  21   // fixed: 1 hdr + 19 data + 1 crc

    uint8_t _rx_buf[IBUS2_FRAME3_SIZE];
    uint8_t _rx_len;

    void send_frame1(const class Aircraft &aircraft);
    void send_frame2();
    void read_frame3();
};

}  // namespace SITL
