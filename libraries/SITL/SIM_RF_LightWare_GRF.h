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
   Simulator for the LightWare GRF rangefinder

   ./Tools/autotest/sim_vehicle.py -v ArduCopter \
       -A --serial5=sim:lightware_grf \
       --console

   Then in ArduPilot:
     param set SERIAL5_PROTOCOL 9   # Rangefinder
     param set RNGFND1_TYPE   45
     reboot
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_RF_LIGHTWAR_GRF250_ENABLED

#include "SIM_SerialRangeFinder.h"
#include <AP_Math/crc.h>
#include <stdint.h>

namespace SITL {

class RF_LightWareGRF : public SerialRangeFinder {
public:
    static SerialRangeFinder *create() { return NEW_NOTHROW RF_LightWareGRF(); }

    uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override;

    uint16_t reading_interval_ms() const override {
        return update_period_ms;
    }

private:
    /*
        GRF binary message format:

           [0]   PREAMBLE 0xAA
           [1-2] FLAGS  (payload length encoded in upper bits)
           [3]   MSGID
           [4..] PAYLOAD
           [end-2..end-1] CRC-XMODEM
    */

    enum class MsgID : uint8_t {
        PRODUCT_NAME     = 0,
        UPDATE_RATE      = 74,
        DISTANCE_OUTPUT  = 27,
        STREAM           = 30,
        DISTANCE_DATA_CM = 44
    };

    uint32_t update_period_ms;
    bool stream_enabled;

    // internal parser buffer
    uint8_t rxbuf[128];
    uint8_t rxlen;

    void process_input();
    void try_parse_message();

    uint16_t compute_crc(const uint8_t *buf, uint8_t len);

    void send_product_name();
    void send_ack_u32(MsgID id, uint32_t value);
    void send_ack_u8(MsgID id, uint8_t value);

    void build_distance_packet(float alt_m, uint8_t *out, uint32_t &outlen);
};

}

#endif // AP_SIM_RF_LIGHTWAR_GRF250_ENABLED
