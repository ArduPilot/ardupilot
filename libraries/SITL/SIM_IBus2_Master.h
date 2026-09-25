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

  Protocol flow (mirrors real receiver behaviour):
    1. Send a boot RESET burst, then Frame 2 GET_TYPE until the slave responds.
    2. Deliver each resource the response still requests (Frame 1 subtype=1
       channel-type key, subtype=2 failsafe values) every cycle; a conforming
       device clears each request bit once the resource is received.
    3. Only once the device reports zero outstanding resources, cycle
       GET_VALUE/GET_PARAM/SET_PARAM commands for telemetry.
    4. Send Frame 1 subtype=0 with SES-encoded RC channel data every cycle.

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter \
  -A --serial5=sim:ibus2master --speedup=1 --console

param set SERIAL5_PROTOCOL 52
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

    // RESET burst sent at startup, mimicking real receiver power-up
    static const uint8_t BOOT_RESET_COUNT = 10;
    uint8_t _boot_resets_sent;

    // Resource-handshake state from the slave's latest GET_TYPE response:
    // bit0=channel types, bit1=failsafe, bit2=receiver internal sensors.
    // The requested resources are delivered every cycle until the device
    // clears the bits; sensor polling starts once it reports zero.
    bool _have_dev_resources;
    uint8_t _dev_resources;

    // Hub-tree scan: when the top node reports a hub type (0xF1..0xF7),
    // probe port nodes at (k,7), descend into nested members (k,0..6),
    // then rotate GET_VALUE over discovered sensor leaves.
    uint8_t _top_type;
    bool _scan_seeded;
    struct Addr { uint8_t l1, l2; };
    Addr _scan_queue[64];
    uint8_t _scan_n;
    uint8_t _scan_head;
    struct Leaf { uint8_t l1, l2, type; bool value_reported; int16_t last_value; };
    Leaf _leaves[49];
    uint8_t _n_leaves;
    uint8_t _leaf_poll_idx;
    // address carried in Frame 1 for the Frame 2 that follows
    uint8_t _addr_l1;
    uint8_t _addr_l2;
    Addr _last_cmd_addr;

    void plan_cycle();

    // Frame reception from AP (Frame 3)
    uint8_t _rx_buf[IBUS2_FRAME3_SIZE];
    uint8_t _rx_len;

    // Bitmask of telemetry sensor types (6-bit) already reported via
    // statustext in the current reporting interval
    uint64_t _telem_types_seen;
    uint32_t _telem_seen_reset_ms;

    void send_frame1(const class Aircraft &aircraft);
    void send_frame1_subtype1(const uint16_t *channels, uint8_t n);
    void send_frame1_subtype2(uint8_t n);
    void send_frame1_subtype0(const uint16_t *channels, uint8_t n);
    void send_frame2();
    void read_frame3();
    void handle_get_value_response(const IBUS2_Resp_GetValue *r);
};

}  // namespace SITL
