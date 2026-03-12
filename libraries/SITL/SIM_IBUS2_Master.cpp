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

#include "SIM_IBUS2_Master.h"

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

const AP_Param::GroupInfo IBUS2Master::var_info[] = {
    // @Param: ENA
    // @DisplayName: IBUS2 Master simulator enable
    // @Description: Enable IBUS2 master simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, IBUS2Master, _enabled, 0),

    AP_GROUPEND
};

IBUS2Master::IBUS2Master() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void IBUS2Master::update(const Aircraft &aircraft)
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

void IBUS2Master::send_frame1(const Aircraft &aircraft)
{
    const uint8_t n_ch = ARRAY_SIZE(sim_channels);
    const uint8_t data_len = n_ch * 2;
    const uint8_t total_len = 3 + data_len + 1;

    uint8_t buf[IBUS2_FRAME1_MAX];
    memset(buf, 0, sizeof(buf));

    buf[0] = IBUS2_PKT_CHANNELS;  // PacketType=0, subtype=0, sync_lost=0, failsafe=0
    buf[1] = total_len;
    buf[2] = 0;  // address bytes

    for (uint8_t i = 0; i < n_ch; i++) {
        buf[3 + i * 2]     = sim_channels[i] & 0xFF;
        buf[3 + i * 2 + 1] = (sim_channels[i] >> 8) & 0xFF;
    }

    ibus2_crc8_write(buf, total_len);
    write_to_autopilot((const char *)buf, total_len);
}

void IBUS2Master::send_frame2()
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

    if (cmd == IBUS2Cmd::SET_PARAM) {
        // Send ReceiverInternalSensors payload (spec Appendix 1, ParamType=0xC000)
        IBUS2_Cmd_SetParam sp{};
        sp.param_type   = IBUS2_PARAM_RECEIVER_SENSORS;
        sp.param_length = sizeof(IBUS2_PA_ReceiverInternalSensors);
        const IBUS2_Pkt<IBUS2_Cmd_SetParam> f2{IBUS2_PKT_COMMAND, (uint8_t)cmd, sp};
        write_to_autopilot((const char *)&f2, sizeof(f2));
        return;
    }

    const IBUS2_Pkt<IBUS2_Frame2> f2{IBUS2_PKT_COMMAND, (uint8_t)cmd, {}};
    write_to_autopilot((const char *)&f2, sizeof(f2));
}

void IBUS2Master::read_frame3()
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
            const auto *pkt = IBUS2_Pkt<IBUS2_Frame3>::cast_validated(_rx_buf);
            if (pkt != nullptr) {
                switch ((IBUS2Cmd)pkt->cmd_code) {
                case IBUS2Cmd::GET_TYPE: {
                    const IBUS2_Resp_GetType *r = (const IBUS2_Resp_GetType *)&pkt->msg;
                    ::fprintf(stderr, "IBUS2Master: GET_TYPE type=0x%02x vlen=%u\n",
                              (unsigned)r->type, (unsigned)r->value_length);
                    break;
                }
                case IBUS2Cmd::GET_VALUE: {
                    const IBUS2_Resp_GetValue *r = (const IBUS2_Resp_GetValue *)&pkt->msg;
                    ::fprintf(stderr, "IBUS2Master: GET_VALUE VID=%u PID=%u\n",
                              (unsigned)r->vid, (unsigned)r->pid);
                    break;
                }
                case IBUS2Cmd::GET_PARAM: {
                    const IBUS2_Resp_GetParam *r = (const IBUS2_Resp_GetParam *)&pkt->msg;
                    ::fprintf(stderr, "IBUS2Master: GET_PARAM type=0x%04x len=%u\n",
                              (unsigned)r->param_type, (unsigned)r->param_length);
                    break;
                }
                case IBUS2Cmd::SET_PARAM: {
                    const IBUS2_Resp_SetParam *r = (const IBUS2_Resp_SetParam *)&pkt->msg;
                    ::fprintf(stderr, "IBUS2Master: SET_PARAM ack type=0x%04x len=%u\n",
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
