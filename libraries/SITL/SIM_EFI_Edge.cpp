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
  simulate Edge Autonomy EFI system
*/

#include "SIM_Aircraft.h"
#include <SITL/SITL.h>
#include <AP_Math/crc.h>
#include <stdio.h>
#include "SIM_EFI_Edge.h"

using namespace SITL;

// assume SERVO3 is throttle
#define EDGE_RPM_INDEX 2

void EFI_Edge::init()
{
    if (is_zero(AP::baro().get_temperature())) {
        return;
    }

    engine.engine_temperature = AP::baro().get_temperature();

    init_done = true;
}

void EFI_Edge::update_engine_model()
{
    auto sitl = AP::sitl();

    const float ambient = AP::baro().get_temperature();
    const uint32_t now_ms = AP_HAL::millis();
    const float delta_t = (now_ms - engine.last_update_ms) * 1e-6;
    engine.last_update_ms = now_ms;

    // lose heat to environment
    const float ENV_LOSS_FACTOR = 25;
    engine.engine_temperature -= (engine.engine_temperature - ambient) * delta_t * ENV_LOSS_FACTOR;

    // gain heat from RPM
    const float rpm = sitl->state.rpm[EDGE_RPM_INDEX];
    const float RPM_GAIN_FACTOR = 10;
    engine.engine_temperature += rpm * delta_t * RPM_GAIN_FACTOR;
}

void EFI_Edge::update_receive()
{
    const ssize_t num_bytes_read = read_from_autopilot((char*)&receive_buf[receive_buf_ofs], ARRAY_SIZE(receive_buf) - receive_buf_ofs);
    if (num_bytes_read < 0) {
        return;
    }
    receive_buf_ofs += num_bytes_read;

    // EDGE packet: [0xA0][0x05][type][size][payload...][crc_lo][crc_hi]
    // minimum packet is 6 bytes (header=4 + crc=2, size=0)
    while (receive_buf_ofs >= 6) {
        if (receive_buf[0] != 0xA0 || receive_buf[1] != 0x05) {
            // scan for sync
            memmove(&receive_buf[0], &receive_buf[1], receive_buf_ofs - 1);
            receive_buf_ofs--;
            continue;
        }

        const uint8_t payload_size = receive_buf[3];
        const uint8_t packet_len = payload_size + 6;  // header(4) + payload + crc(2)

        if (receive_buf_ofs < packet_len) {
            return;  // incomplete packet
        }

        // verify CRC over header+payload (size+4 bytes)
        const uint16_t computed_crc = crc_crc16_arc(0, receive_buf, payload_size + 4);
        const uint16_t received_crc = receive_buf[payload_size + 4] | (receive_buf[payload_size + 5] << 8);

        if (computed_crc == received_crc) {
            const uint8_t ptype = receive_buf[2];
            if (ptype == 11) {
                // TELEMETRY_PERIOD: 2-byte payload (period in 50ms units)
                if (payload_size >= 1) {
                    telem_period = receive_buf[4];
                    if (telem_period < 1) {
                        telem_period = 1;
                    }
                }
            } else if (ptype == 14) {
                // SW_KILL_SWITCH: 1-byte payload (0=off, 1=on)
                if (payload_size >= 1) {
                    sw_kill = receive_buf[4] != 0;
                }
            }
        }

        memmove(&receive_buf[0], &receive_buf[packet_len], receive_buf_ofs - packet_len);
        receive_buf_ofs -= packet_len;
    }
}

void EFI_Edge::send_packet(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t pkt[70];
    pkt[0] = 0xA0;
    pkt[1] = 0x05;
    pkt[2] = type;
    pkt[3] = len;
    memcpy(&pkt[4], payload, len);

    const uint16_t crc = crc_crc16_arc(0, pkt, len + 4);
    pkt[len + 4] = crc & 0xFF;
    pkt[len + 5] = (crc >> 8) & 0xFF;

    write_to_autopilot((const char*)pkt, len + 6);
}

void EFI_Edge::send_telem1()
{
    // telem1: type 6, 10-byte payload
    // fuel and engine working time data - we just send zeros for now
    Telem1 t1 {};
    send_packet(6, (const uint8_t *)&t1, sizeof(t1));
}

void EFI_Edge::send_telem2()
{
    auto sitl = AP::sitl();
    const float ambient_C = AP::baro().get_temperature();

    Telem2 t2 {};

    t2.time_raw = AP_HAL::millis() * 10;
    t2.ecu_t = (int8_t)engine.engine_temperature;
    t2.fuel_flow = (uint16_t)(MAX(sitl->state.rpm[EDGE_RPM_INDEX] - 1500.0, 0) / 2200.0 * 2000);
    t2.sys_status = sw_kill ? 0x04 : 0x00;
    t2.rpm = (uint16_t)MAX(sitl->state.rpm[EDGE_RPM_INDEX], 0);
    t2.baro = (int16_t)(AP::baro().get_pressure() / 100.0);
    t2.mat = (int16_t)((ambient_C * 9.0 / 5.0 + 32.0) * 10.0);
    t2.coolant = (int16_t)((engine.engine_temperature * 9.0 / 5.0 + 32.0) * 10.0);

    const float throttle_pct = MAX(sitl->state.rpm[EDGE_RPM_INDEX], 0) / 7000.0 * 100.0;
    t2.tps = (int16_t)(throttle_pct * 10.0);
    t2.ms_volt = 126;  // 12.6V

    send_packet(7, (const uint8_t *)&t2, sizeof(t2));
}

void EFI_Edge::update_send()
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t period_ms = telem_period * 50;

    if (now_ms - last_telem_send_ms < period_ms) {
        return;
    }
    last_telem_send_ms = now_ms;

    send_telem1();
    send_telem2();
}

void EFI_Edge::update()
{
    const auto *sitl = AP::sitl();
    if (!sitl || sitl->efi_type != SIM::EFI_TYPE_EDGE) {
        return;
    }

    if (!init_done) {
        init();
        if (!init_done) {
            return;
        }
    }

    update_engine_model();
    update_receive();
    update_send();
}
