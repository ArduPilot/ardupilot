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
  Simulator for Topotek gimbal

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:topotek --speedup=1

param set MNT1_TYPE 12        # topotek
param set SERIAL5_PROTOCOL 8  # gimbal
reboot

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_TOPOTEK_ENABLED

#include "SIM_Mount.h"
#include "SIM_Gimbal.h"

namespace SITL {

class Topotek : public Mount {
public:

    void update(const Aircraft &aircraft) override;

private:

    // the physical gimbal:
    Gimbal gimbal;

    // input accumulation buffer; also used as working buffer by handle_packet()
    static constexpr uint8_t PACKETLEN_MAX = 36;
    uint8_t _buf[PACKETLEN_MAX];
    uint8_t _buflen;

    uint32_t _last_attitude_ms;     // time of last attitude packet sent

    // last commanded angles from GIP/GIY packets (wire centidegrees, same sign as sent by driver)
    int16_t _commanded_pitch_cd;
    int16_t _commanded_yaw_cd;

    // read and dispatch incoming packets from autopilot
    void update_input();

    // scan forward from search_start_pos for '#' and move it to _buf[0]
    void move_preamble_in_buffer(uint8_t search_start_pos);

    // send gimbal attitude packet to the driver
    void send_attitude();

    // dispatch a complete packet beginning at _buf[0], data_len data bytes
    void handle_packet(uint8_t data_len);

    // build and send a response packet
    void send_packet(char addr2, const char id[3], bool write, const uint8_t *data, uint8_t len);

    // encode a uint16 as 4 uppercase ASCII hex chars
    static void uint16_to_hex4(uint16_t val, uint8_t buf[4]);

    // convert a nibble (0-15) to an uppercase ASCII hex character
    static uint8_t hex2char(uint8_t nibble) {
        return nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10);
    }
};

}  // namespace SITL

#endif  // AP_SIM_TOPOTEK_ENABLED
