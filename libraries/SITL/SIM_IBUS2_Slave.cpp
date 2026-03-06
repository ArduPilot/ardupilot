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
  Simulator for an IBUS2 slave device — used to test AP_IBUS2_Master.
*/

#include "SIM_IBUS2_Slave.h"

#if AP_IBUS2_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

using namespace SITL;

// Response delay after Frame 2 (µs)
#define IBUS2_SLAVE_RESPONSE_DELAY_US 160

const AP_Param::GroupInfo IBUS2Slave::var_info[] = {
    // @Param: ENA
    // @DisplayName: IBUS2 Slave simulator enable
    // @Description: Enable IBUS2 slave device simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, IBUS2Slave, _enabled, 0),

    AP_GROUPEND
};

IBUS2Slave::IBUS2Slave() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_channels, 0, sizeof(_channels));
}

void IBUS2Slave::update(const Aircraft &aircraft)
{
    if (!_enabled.get()) {
        return;
    }
    if (!init_sitl_pointer()) {
        return;
    }

    process_rx(aircraft);

    if (_respond_pending) {
        const uint32_t now_us = AP_HAL::micros();
        if ((now_us - _frame2_end_us) >= IBUS2_SLAVE_RESPONSE_DELAY_US) {
            send_response(aircraft);
            _respond_pending = false;
        }
    }
}

void IBUS2Slave::process_rx(const Aircraft &aircraft)
{
    char raw[64];
    const ssize_t n = read_from_autopilot(raw, sizeof(raw));
    if (n <= 0) {
        return;
    }

    for (ssize_t i = 0; i < n; i++) {
        const uint8_t b = (uint8_t)raw[i];

        switch (_rx_state) {
        case RxState::WAIT_HEADER: {
            const uint8_t pkt_type = b & 0x3;
            if (pkt_type == IBUS2_PKT_CHANNELS) {
                _rx1_buf[0] = b;
                _rx1_len = 1;
                _rx1_expected_len = 0;
                _rx_state = RxState::IN_FRAME1;
            } else if (pkt_type == IBUS2_PKT_COMMAND) {
                _rx2_buf[0] = b;
                _rx2_len = 1;
                _rx_state = RxState::IN_FRAME2;
            }
            break;
        }

        case RxState::IN_FRAME1:
            _rx1_buf[_rx1_len++] = b;
            if (_rx1_len == 2) {
                _rx1_expected_len = b;
                if (_rx1_expected_len < IBUS2_FRAME1_MIN ||
                    _rx1_expected_len > IBUS2_FRAME1_MAX) {
                    _rx_state = RxState::WAIT_HEADER;
                }
            } else if (_rx1_len == _rx1_expected_len) {
                if (ibus2_crc8_ok(_rx1_buf, _rx1_expected_len)) {
                    // Decode channels (simple 2-byte LE per channel)
                    const uint8_t data_len = _rx1_expected_len - 4;  // header(3) + crc(1)
                    const uint8_t n_ch = MIN(data_len / 2, (uint8_t)ARRAY_SIZE(_channels));
                    for (uint8_t c = 0; c < n_ch; c++) {
                        _channels[c] = (uint16_t)_rx1_buf[3 + c * 2] |
                                       ((uint16_t)_rx1_buf[3 + c * 2 + 1] << 8);
                    }
                }
                _rx_state = RxState::WAIT_HEADER;
            }
            break;

        case RxState::IN_FRAME2:
            _rx2_buf[_rx2_len++] = b;
            if (_rx2_len == IBUS2_FRAME2_SIZE) {
                if (!_respond_pending && ibus2_crc8_ok(_rx2_buf, IBUS2_FRAME2_SIZE)) {
                    memcpy(&_pending_cmd, _rx2_buf, sizeof(_pending_cmd));
                    _frame2_end_us = AP_HAL::micros();
                    _respond_pending = true;
                }
                _rx_state = RxState::WAIT_HEADER;
            }
            break;
        }
    }
}

void IBUS2Slave::send_response(const Aircraft &aircraft)
{
    const IBUS2Cmd cmd = (IBUS2Cmd)_pending_cmd.cmd_code;

    switch (cmd) {
    case IBUS2Cmd::GET_TYPE: {
        IBUS2_Resp_GetType r;
        memset(&r, 0, sizeof(r));
        r.pkt_type        = IBUS2_PKT_RESPONSE;
        r.cmd_code        = (uint8_t)IBUS2Cmd::GET_TYPE;
        r.type            = (uint8_t)IBUS2DeviceType::DIGITAL_SERVO;
        r.value_length    = 16;
        r.channels_types  = 1;
        r.failsafe        = 1;
        ibus2_crc8_write((uint8_t *)&r, sizeof(r));
        write_to_autopilot((const char *)&r, sizeof(r));
        break;
    }
    case IBUS2Cmd::GET_VALUE: {
        IBUS2_Resp_GetValue r;
        memset(&r, 0, sizeof(r));
        r.pkt_type = IBUS2_PKT_RESPONSE;
        r.cmd_code = (uint8_t)IBUS2Cmd::GET_VALUE;
        r.vid      = 0x01;
        r.pid      = 0x03;

        // Pack one voltage data point (scaled from aircraft battery voltage)
        // PackLength=1, PackCurIndex=0
        r.value[1] = (1 << 2);
        const int16_t volts_raw = (int16_t)(aircraft.get_battery_voltage() * 100);
        r.value[2] = (uint8_t)IBUS2SensorType::VOLTAGE & 0x3F;
        r.value[3] = 0;
        r.value[4] = volts_raw & 0xFF;
        r.value[5] = (volts_raw >> 8) & 0xFF;

        ibus2_crc8_write((uint8_t *)&r, sizeof(r));
        write_to_autopilot((const char *)&r, sizeof(r));
        break;
    }
    case IBUS2Cmd::GET_PARAM: {
        IBUS2_Resp_GetParam r;
        memset(&r, 0, sizeof(r));
        r.pkt_type    = IBUS2_PKT_RESPONSE;
        r.cmd_code    = (uint8_t)IBUS2Cmd::GET_PARAM;
        r.param_type  = ((const IBUS2_Cmd_GetParam *)&_pending_cmd)->param_type;
        r.param_length = 0;
        ibus2_crc8_write((uint8_t *)&r, sizeof(r));
        write_to_autopilot((const char *)&r, sizeof(r));
        break;
    }
    case IBUS2Cmd::SET_PARAM: {
        IBUS2_Resp_SetParam r;
        memset(&r, 0, sizeof(r));
        r.pkt_type    = IBUS2_PKT_RESPONSE;
        r.cmd_code    = (uint8_t)IBUS2Cmd::SET_PARAM;
        r.param_type  = ((const IBUS2_Cmd_SetParam *)&_pending_cmd)->param_type;
        r.param_length = 0;
        ibus2_crc8_write((uint8_t *)&r, sizeof(r));
        write_to_autopilot((const char *)&r, sizeof(r));
        break;
    }
    default:
        break;
    }
}

#endif  // AP_IBUS2_ENABLED
