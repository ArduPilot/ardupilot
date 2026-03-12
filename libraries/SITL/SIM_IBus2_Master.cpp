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
  Simulator for an IBUS2 master (receiver/hub) — used to test AP_IBUS2_Slave.
*/

#include "SIM_IBus2_Master.h"

#if AP_IBUS2_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

using namespace SITL;

// ~7 ms between master cycles (per IBUS2 spec)
#define IBUS2_MASTER_CYCLE_US 7000

// Simulated RC channels (1000-2000 µs range, 1500 = centre)
static const uint16_t sim_channels[14] = {
    1500, 1500, 1000, 1500,   // roll, pitch, throttle, yaw
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500,
};

// ---------------------------------------------------------------------------
// Local CRC8 helpers (polynomial 0x25).
// Duplicated from AP_IBUS2.cpp so the SITL library is independent of the
// vehicle library and does not require AP_IBUS2.cpp to be linked.
// ---------------------------------------------------------------------------
static uint8_t sitl_ibus2_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x25) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static bool sitl_ibus2_crc8_ok(const uint8_t *buf, uint16_t len)
{
    return len >= 2 && sitl_ibus2_crc8(buf, len - 1) == buf[len - 1];
}

static void sitl_ibus2_crc8_write(uint8_t *buf, uint16_t len)
{
    if (len >= 2) {
        buf[len - 1] = sitl_ibus2_crc8(buf, len - 1);
    }
}
// ---------------------------------------------------------------------------

const AP_Param::GroupInfo IBus2Master::var_info[] = {
    // @Param: ENA
    // @DisplayName: IBUS2 Master simulator enable
    // @Description: Enable IBUS2 master simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, IBus2Master, _enabled, 0),

    AP_GROUPEND
};

IBus2Master::IBus2Master() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void IBus2Master::update(const Aircraft &aircraft)
{
    if (!_enabled.get()) {
        return;
    }
    if (!init_sitl_pointer()) {
        return;
    }

    const uint32_t now_us = AP_HAL::micros();
    if ((now_us - _last_send_us) < IBUS2_MASTER_CYCLE_US) {
        // Read any pending Frame 3 responses between cycles
        read_frame3();
        return;
    }
    _last_send_us = now_us;

    send_frame1(aircraft);
    send_frame2();
}

void IBus2Master::send_frame1(const Aircraft &aircraft)
{
    const uint8_t n_ch = ARRAY_SIZE(sim_channels);
    const uint8_t total_len = 3 + n_ch * 2 + 1;

    uint8_t buf[IBUS2_FRAME1_MAX] {};
    buf[0] = IBUS2_PKT_CHANNELS;  // PacketType=0, subtype=0, sync_lost=0, failsafe=0
    buf[1] = total_len;
    buf[2] = 0;  // address bytes

    for (uint8_t i = 0; i < n_ch; i++) {
        buf[3 + i * 2]     = sim_channels[i] & 0xFF;
        buf[3 + i * 2 + 1] = (sim_channels[i] >> 8) & 0xFF;
    }

    sitl_ibus2_crc8_write(buf, total_len);
    write_to_autopilot((const char *)buf, total_len);
}

void IBus2Master::send_frame2()
{
    // Cycle through all four command codes to exercise every AP_IBUS2_Slave path.
    static const IBUS2Cmd cmds[] = {
        IBUS2Cmd::GET_TYPE,
        IBUS2Cmd::GET_VALUE,
        IBUS2Cmd::GET_PARAM,
        IBUS2Cmd::SET_PARAM,
    };
    const IBUS2Cmd cmd = cmds[_cmd_cycle % ARRAY_SIZE(cmds)];
    _cmd_cycle++;

    // Frame 2 layout: buf[0] = pkt_type:2|cmd_code:6, buf[1..19] = payload, buf[20] = CRC
    uint8_t buf[IBUS2_FRAME2_SIZE] {};
    buf[0] = (uint8_t)(IBUS2_PKT_COMMAND | ((uint8_t)cmd << 2));

    if (cmd == IBUS2Cmd::SET_PARAM) {
        // Send ReceiverInternalSensors payload (spec Appendix 1, ParamType=0xC000)
        auto *sp = reinterpret_cast<IBUS2_Cmd_SetParam *>(buf + 1);
        sp->param_type   = IBUS2_PARAM_RECEIVER_SENSORS;
        sp->param_length = sizeof(IBUS2_PA_ReceiverInternalSensors);
    }

    sitl_ibus2_crc8_write(buf, sizeof(buf));
    write_to_autopilot((const char *)buf, sizeof(buf));
}

void IBus2Master::read_frame3()
{
    char buf[IBUS2_FRAME3_SIZE];
    const ssize_t n = read_from_autopilot(buf, sizeof(buf) - _rx_len);
    if (n <= 0) {
        return;
    }

    for (ssize_t i = 0; i < n; i++) {
        const uint8_t b = (uint8_t)buf[i];

        if (_rx_len == 0) {
            // Wait for Frame 3 header (PacketType == 2)
            if ((b & 0x3) != IBUS2_PKT_RESPONSE) {
                continue;
            }
        }
        _rx_buf[_rx_len++] = b;

        if (_rx_len == IBUS2_FRAME3_SIZE) {
            if (sitl_ibus2_crc8_ok(_rx_buf, IBUS2_FRAME3_SIZE)) {
                const uint8_t cmd_code = _rx_buf[0] >> 2;
                const uint8_t *msg = _rx_buf + 1;
                switch ((IBUS2Cmd)cmd_code) {
                case IBUS2Cmd::GET_TYPE: {
                    const auto *r = reinterpret_cast<const IBUS2_Resp_GetType *>(msg);
                    ::fprintf(stderr, "IBus2Master: GET_TYPE type=0x%02x vlen=%u\n",
                              (unsigned)r->type, (unsigned)r->value_length);
                    break;
                }
                case IBUS2Cmd::GET_VALUE: {
                    const auto *r = reinterpret_cast<const IBUS2_Resp_GetValue *>(msg);
                    ::fprintf(stderr, "IBus2Master: GET_VALUE VID=%u PID=%u\n",
                              (unsigned)r->vid, (unsigned)r->pid);
                    break;
                }
                case IBUS2Cmd::GET_PARAM: {
                    const auto *r = reinterpret_cast<const IBUS2_Resp_GetParam *>(msg);
                    ::fprintf(stderr, "IBus2Master: GET_PARAM type=0x%04x len=%u\n",
                              (unsigned)r->param_type, (unsigned)r->param_length);
                    break;
                }
                case IBUS2Cmd::SET_PARAM: {
                    const auto *r = reinterpret_cast<const IBUS2_Resp_SetParam *>(msg);
                    ::fprintf(stderr, "IBus2Master: SET_PARAM ack type=0x%04x len=%u\n",
                              (unsigned)r->param_type, (unsigned)r->param_length);
                    break;
                }
                default:
                    break;
                }
            }
            _rx_len = 0;
        }
    }
}

#endif  // AP_IBUS2_ENABLED
