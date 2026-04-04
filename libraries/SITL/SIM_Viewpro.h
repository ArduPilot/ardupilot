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
  Simulator for Viewpro gimbal

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:viewpro --speedup=1

param set MNT1_TYPE 11        # viewpro
param set SERIAL5_PROTOCOL 8  # gimbal
reboot

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_VIEWPRO_ENABLED

#include "SIM_Mount.h"
#include "SIM_Gimbal.h"

namespace SITL {

class Viewpro : public Mount {
public:

    void update(const Aircraft &aircraft) override;

private:

    // the physical gimbal:
    Gimbal gimbal;

    // Viewpro header bytes
    static constexpr uint8_t HEADER1 = 0x55;
    static constexpr uint8_t HEADER2 = 0xAA;
    static constexpr uint8_t HEADER3 = 0xDC;
    static constexpr uint8_t PACKETLEN_MAX = 63;

    // frame IDs (matching AP_Mount_Viewpro.h)
    enum class FrameId : uint8_t {
        HANDSHAKE   = 0x00,
        U           = 0x01,
        V           = 0x02,
        HEARTBEAT   = 0x10,
        A1          = 0x1A,
        C1          = 0x1C,
        E1          = 0x1E,
        C2          = 0x2C,
        E2          = 0x2E,
        T1_F1_B1_D1 = 0x40,
        M_AHRS      = 0xB1,
    };

    // U packet communication configuration commands
    enum class CommConfigCmd : uint8_t {
        QUERY_FIRMWARE_VER = 0xD0,
        QUERY_MODEL        = 0xE4,
    };

    // packet buffer.
    // after header bytes are discarded by the state machine:
    //   _buf[0] = length+frame_counter byte
    //   _buf[1] = frame_id
    //   _buf[2+] = data bytes
    //   _buf[last] = CRC
    uint8_t _buf[PACKETLEN_MAX];
    uint8_t _buflen;

    // parser state
    enum class ParseState : uint8_t {
        HEADER1,
        HEADER2,
        HEADER3,
        LENGTH,
        FRAMEID,
        DATA,
        CHECKSUM,
    } _parse_state;

    uint8_t _data_len;              // data bytes expected in current packet
    uint8_t _data_bytes_received;   // data bytes received so far

    uint8_t _frame_counter;         // 2-bit counter for outgoing packets
    uint32_t _last_attitude_ms;     // time of last T1_F1_B1_D1 packet sent

    // last A1 angle target received from driver, stored as raw int16 wire values
    // (same encoding used in T1_F1_B1_D1 replies so the driver round-trips exactly)
    int16_t _target_pitch_raw;
    int16_t _target_yaw_raw;

    // read and parse incoming bytes from autopilot
    void update_input();

    // dispatch a complete, CRC-verified packet in _buf
    void dispatch_packet();

    // send T1_F1_B1_D1 attitude status packet
    void send_t1_f1_b1_d1();

    // send V response to a U communication config command
    void send_v_response(CommConfigCmd cmd);

    // build and write a complete packet; databuff[0] = frame_id
    void send_packet(const uint8_t *databuff, uint8_t databuff_len);

};

}  // namespace SITL

#endif  // AP_SIM_VIEWPRO_ENABLED
