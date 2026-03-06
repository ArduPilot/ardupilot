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

#include "AP_IBUS2_Master.h"

#if AP_IBUS2_MASTER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL &hal;

// IBUS2 baud rate
#define IBUS2_BAUD 1500000

// Minimum time (µs) to wait for Frame 3 after sending Frame 2
#define IBUS2_RESPONSE_TIMEOUT_US 500

// Interval (µs) between IBUS2 master cycles (~7 ms)
#define IBUS2_CYCLE_US 7000

const AP_Param::GroupInfo AP_IBUS2_Master::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: IBUS2 Master enable
    // @Description: Enable IBUS2 master driver
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_IBUS2_Master, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SEND_MASK
    // @DisplayName: IBUS2 Master Frame2 send mask
    // @Description: Bitmask of Frame 2 command types to send each cycle. bit0=GET_TYPE, bit1=GET_VALUE, bit2=GET_PARAM, bit3=SET_PARAM
    // @Bitmask: 0:GET_TYPE,1:GET_VALUE,2:GET_PARAM,3:SET_PARAM
    // @User: Advanced
    AP_GROUPINFO("SEND_MASK", 2, AP_IBUS2_Master, _send_mask, 3),  // default: GET_TYPE + GET_VALUE

    AP_GROUPEND
};

AP_IBUS2_Master::AP_IBUS2_Master()
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_devices, 0, sizeof(_devices));
    memset(_device_known, 0, sizeof(_device_known));
}

void AP_IBUS2_Master::init()
{
    if (!_enable) {
        return;
    }
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IBUS2_Master, 0);
    if (_port == nullptr) {
        return;
    }
    _port->set_options(_port->OPTION_HDPLEX);
    _port->begin(IBUS2_BAUD);
    _initialized = true;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_IBUS2_Master::update, void));
}

void AP_IBUS2_Master::update()
{
    if (!_initialized) {
        return;
    }

    // Receive any pending Frame 3 first
    process_rx();

    const uint32_t now_us = AP_HAL::micros();

    // If we are waiting for a response and it has timed out, move on
    if (_waiting_response && (now_us - _frame2_sent_us) > IBUS2_RESPONSE_TIMEOUT_US) {
        _waiting_response = false;
        _current_addr = (_current_addr + 1) & 0x7;
    }

    if (_waiting_response) {
        return;
    }

    // Only start a new cycle every IBUS2_CYCLE_US
    static uint32_t last_cycle_us;
    if ((now_us - last_cycle_us) < IBUS2_CYCLE_US) {
        return;
    }
    last_cycle_us = now_us;

    // Send Frame 1 (channel values)
    send_frame1();

    // Send Frame 2 if any command bits are set
    if (_send_mask != 0) {
        send_frame2(_current_addr);
        _frame2_sent_us = AP_HAL::micros();
        _waiting_response = true;
    } else {
        _current_addr = (_current_addr + 1) & 0x7;
    }
}

void AP_IBUS2_Master::send_frame1()
{
    // Build a minimal Frame 1 with channel values from SRV_Channel outputs.
    // For simplicity we use uncompressed 2-byte channel values (subtype=0 would
    // normally be compressed; a future revision should implement SES compression).
    // We send up to 14 channels (14 × 2 bytes = 28 bytes data, total frame = 32 bytes).
    const uint8_t max_channels = 14;
    const uint8_t data_len = max_channels * 2;
    const uint8_t total_len = 3 + data_len + 1;  // header(3) + data + CRC

    uint8_t buf[IBUS2_FRAME1_MAX];
    memset(buf, 0, sizeof(buf));

    buf[0] = (IBUS2_PKT_CHANNELS & 0x3);  // PacketType=0, subtype=0, sync/failsafe=0
    buf[1] = total_len;
    buf[2] = 0;  // AddressLevel1=0, AddressLevel2=0, reserved=0

    for (uint8_t i = 0; i < max_channels; i++) {
        uint16_t pwm = SRV_Channels::srv_channel(i) ? SRV_Channels::srv_channel(i)->get_output_pwm() : 1500;
        buf[3 + i * 2]     = pwm & 0xFF;
        buf[3 + i * 2 + 1] = (pwm >> 8) & 0xFF;
    }

    ibus2_crc8_write(buf, total_len);
    _port->write(buf, total_len);
}

void AP_IBUS2_Master::send_frame2(uint8_t addr)
{
    const uint8_t mask = (uint8_t)_send_mask.get();

    // Choose which command to send based on SEND_MASK and whether device is known
    IBUS2Cmd cmd;
    if (!_device_known[addr] && (mask & SEND_GET_TYPE)) {
        cmd = IBUS2Cmd::GET_TYPE;
    } else if (mask & SEND_GET_VALUE) {
        cmd = IBUS2Cmd::GET_VALUE;
    } else if (mask & SEND_GET_PARAM) {
        cmd = IBUS2Cmd::GET_PARAM;
    } else if (mask & SEND_SET_PARAM) {
        cmd = IBUS2Cmd::SET_PARAM;
    } else {
        return;
    }

    IBUS2_Frame2 f2;
    memset(&f2, 0, sizeof(f2));
    f2.pkt_type = IBUS2_PKT_COMMAND;
    f2.cmd_code = (uint8_t)cmd;
    // data[0..1] encode the address (AddressLevel1/Level2) in the addressing scheme;
    // for a simple single-device setup leave at zero.
    ibus2_crc8_write((uint8_t *)&f2, sizeof(f2));
    _port->write((const uint8_t *)&f2, sizeof(f2));
}

void AP_IBUS2_Master::process_rx()
{
    const uint16_t avail = MIN(_port->available(), 256U);
    for (uint16_t i = 0; i < avail; i++) {
        uint8_t b;
        if (!_port->read(b)) {
            break;
        }

        switch (_rx_state) {
        case RxState::WAIT_HEADER:
            // Look for a byte whose top 2 bits == IBUS2_PKT_RESPONSE (2)
            if ((b & 0x3) == IBUS2_PKT_RESPONSE) {
                _rx_buf[0] = b;
                _rx_len = 1;
                _rx_state = RxState::IN_FRAME3;
            }
            break;

        case RxState::IN_FRAME3:
            _rx_buf[_rx_len++] = b;
            if (_rx_len == IBUS2_FRAME3_SIZE) {
                if (ibus2_crc8_ok(_rx_buf, IBUS2_FRAME3_SIZE)) {
                    handle_frame3((const IBUS2_Frame3 *)_rx_buf);
                }
                _rx_state = RxState::WAIT_HEADER;
                _waiting_response = false;
                _current_addr = (_current_addr + 1) & 0x7;
            }
            break;
        }
    }
}

void AP_IBUS2_Master::handle_frame3(const IBUS2_Frame3 *f3)
{
    const uint8_t addr = _current_addr & 0x7;
    const IBUS2Cmd cmd = (IBUS2Cmd)f3->cmd_code;

    switch (cmd) {
    case IBUS2Cmd::GET_TYPE: {
        const IBUS2_Resp_GetType *r = (const IBUS2_Resp_GetType *)f3;
        _devices[addr].device_type = r->type;
        _device_known[addr] = true;
        break;
    }
    case IBUS2Cmd::GET_VALUE: {
        const IBUS2_Resp_GetValue *r = (const IBUS2_Resp_GetValue *)f3;
        memcpy(_devices[addr].value, r->value, sizeof(_devices[addr].value));
        _devices[addr].vid = r->vid;
        _devices[addr].pid = r->pid;
        _devices[addr].valid = true;
        _devices[addr].last_update_ms = AP_HAL::millis();
        break;
    }
    case IBUS2Cmd::GET_PARAM: {
        const IBUS2_Resp_GetParam *r = (const IBUS2_Resp_GetParam *)f3;
        // Store raw param value; consumers can read via get_device_data()
        if (r->param_length > 0 && r->param_length <= 16) {
            memcpy(_devices[addr].value, r->param_value,
                   MIN(r->param_length, (uint8_t)sizeof(_devices[addr].value)));
        }
        break;
    }
    case IBUS2Cmd::SET_PARAM:
        // Acknowledge only; nothing stored
        break;
    default:
        break;
    }
}

#endif  // AP_IBUS2_MASTER_ENABLED
