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

  Protocol flow:
    1. Send Frame 2 GET_TYPE until the slave responds.
    2. On GET_TYPE response, record channels_types and failsafe flags.
       If channels_types=1, the slave needs a decompression key — start sending
       Frame 1 subtype=1 (immediately on the next cycle and every 50 cycles after).
    3. Switch Frame 2 commands to GET_VALUE for telemetry.
    4. Send Frame 1 subtype=0 with SES-encoded RC channel data every cycle.

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
#include <AP_RCProtocol/AP_RCProtocol_UDP.h>

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
    uint8_t  _cmd_cycle;       // GET_VALUE command sub-cycle counter

    // Protocol state, set from the slave's GET_TYPE response
    bool _device_wants_channel_types;  // slave needs Frame 1 subtype=1
    bool _device_wants_failsafe;       // slave needs failsafe data
    bool _send_subtype1_now;           // send subtype=1 on the very next Frame 1

    uint32_t _frame1_cycle;    // counts every Frame 1 sent; drives periodic subtype=1

    // Frame reception from AP (Frame 3)
    uint8_t _rx_buf[IBUS2_FRAME3_SIZE];
    uint8_t _rx_len;

    void send_frame1(const class Aircraft &aircraft);
    void send_frame1_subtype1(const uint16_t *channels, uint8_t n);
    void send_frame1_subtype0(const uint16_t *channels, uint8_t n);
    void send_frame2();
    void read_frame3();
};

}  // namespace SITL
