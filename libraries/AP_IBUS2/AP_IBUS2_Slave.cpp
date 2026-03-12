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

#include "AP_IBUS2_Slave.h"

#if AP_IBUS2_SLAVE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL &hal;

// IBUS2 baud rate
#define IBUS2_BAUD 1500000

// Telemetry Adapter VID/PID (Telemetry_Adapter_Protocol___20240923_en_.pdf §4)
#define IBUS2_TELEM_VID 0x01
#define IBUS2_TELEM_PID 0x03

AP_IBUS2_Slave::AP_IBUS2_Slave()
{
}

void AP_IBUS2_Slave::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IBUS2_Slave, 0);
    if (_port == nullptr) {
        return;
    }
    _port->set_options(_port->OPTION_HDPLEX);
    _port->begin(IBUS2_BAUD);
    _initialized = true;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_IBUS2_Slave::update, void));
}

void AP_IBUS2_Slave::update()
{
    if (!_initialized) {
        return;
    }

    // Parse incoming bytes (Frame 1 and Frame 2)
    process_rx();

    // Send Frame 3 response after the required delay
    if (_response_pending) {
        const uint32_t now_us = AP_HAL::micros();
        if ((now_us - _frame2_end_us) >= RESPONSE_DELAY_US) {
            send_frame3();
            _response_pending = false;
        }
    }
}

bool AP_IBUS2_Slave::get_rc_channels(uint16_t *channels, uint8_t &count) const
{
    if (_rc_channel_count == 0) {
        count = 0;
        return false;
    }
    count = _rc_channel_count;
    memcpy(channels, _rc_channels, count * sizeof(uint16_t));
    return true;
}

void AP_IBUS2_Slave::process_rx()
{
    const uint16_t avail = MIN(_port->available(), 512U);
    for (uint16_t i = 0; i < avail; i++) {
        uint8_t b;
        if (!_port->read(b)) {
            break;
        }

        switch (_rx_state) {
        case RxState::WAIT_HEADER: {
            const uint8_t pkt_type = b & 0x3;
            if (pkt_type == IBUS2_PKT_CHANNELS) {
                // Start of Frame 1
                _rx_buf[0] = b;
                _rx_len = 1;
                _rx_state = RxState::IN_FRAME1;
                _frame1_expected_len = 0;  // will be set when we get byte 1 (Length)
            } else if (pkt_type == IBUS2_PKT_COMMAND) {
                // Start of Frame 2
                _rx_buf[0] = b;
                _rx_len = 1;
                _rx_state = RxState::IN_FRAME2;
            }
            break;
        }

        case RxState::IN_FRAME1:
            _rx_buf[_rx_len++] = b;
            if (_rx_len == 2) {
                // Byte 1 is the Length field
                _frame1_expected_len = b;
                if (_frame1_expected_len < IBUS2_FRAME1_MIN ||
                    _frame1_expected_len > IBUS2_FRAME1_MAX) {
                    // Bad length, abort
                    _rx_state = RxState::WAIT_HEADER;
                }
            } else if (_rx_len == _frame1_expected_len) {
                if (ibus2_crc8_ok(_rx_buf, _frame1_expected_len)) {
                    handle_frame1(_rx_buf, _frame1_expected_len);
                }
                _rx_state = RxState::WAIT_HEADER;
            }
            break;

        case RxState::IN_FRAME2:
            _rx_buf[_rx_len++] = b;
            if (_rx_len == IBUS2_FRAME2_SIZE) {
                const auto *pkt = IBUS2_Pkt<IBUS2_Frame2>::cast_validated(_rx_buf);
                if (pkt != nullptr) {
                    handle_frame2(pkt);
                }
                _rx_state = RxState::WAIT_HEADER;
            }
            break;
        }
    }
}

void AP_IBUS2_Slave::handle_frame1(const uint8_t *buf, uint8_t len)
{
    // buf[0] = header (PacketType, subtype, sync_lost, failsafe)
    // buf[1] = Length
    // buf[2] = address byte
    // buf[3..len-2] = compressed channel data
    // buf[len-1] = CRC8

    // For subtype=0 (compressed) we decode 2-byte little-endian values.
    // A real implementation would use the SES_UnpackChannels decompression;
    // for now treat each pair of bytes as a raw 16-bit channel value.
    const uint8_t data_start = 3;
    const uint8_t data_end   = len - 1;  // exclude CRC
    const uint8_t data_len   = data_end - data_start;
    const uint8_t n_channels = data_len / 2;

    for (uint8_t i = 0; i < n_channels && i < 32; i++) {
        _rc_channels[i] = (uint16_t)buf[data_start + i * 2] |
                          ((uint16_t)buf[data_start + i * 2 + 1] << 8);
    }
    const bool was_empty = (_rc_channel_count == 0);
    _rc_channel_count = n_channels;
    _rc_last_update_ms = AP_HAL::millis();
    if (was_empty && n_channels > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IBUS2: %u RC chans from receiver", (unsigned)n_channels);
    }
}

void AP_IBUS2_Slave::handle_frame2(const IBUS2_Pkt<IBUS2_Frame2> *f2)
{
    // Only respond if not already waiting to respond
    if (_response_pending) {
        return;
    }
    memcpy(&_pending_cmd, f2, sizeof(_pending_cmd));
    _frame2_end_us = AP_HAL::micros();
    _response_pending = true;
}

void AP_IBUS2_Slave::send_frame3()
{
    const IBUS2Cmd cmd = (IBUS2Cmd)_pending_cmd.cmd_code;

    // Log first occurrence of each command type (except GET_VALUE which is routine)
    const uint8_t cmd_bit = 1U << (uint8_t)cmd;
    if (!(_logged_cmds & cmd_bit)) {
        _logged_cmds |= cmd_bit;
        switch (cmd) {
        case IBUS2Cmd::GET_TYPE:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IBUS2: GET_TYPE cmd from master");
            break;
        case IBUS2Cmd::GET_PARAM:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IBUS2: GET_PARAM cmd from master");
            break;
        case IBUS2Cmd::SET_PARAM:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IBUS2: SET_PARAM cmd from master");
            break;
        default:
            break;
        }
    }

    switch (cmd) {
    case IBUS2Cmd::GET_TYPE:
        send_resp_get_type();
        break;
    case IBUS2Cmd::GET_VALUE:
        send_resp_get_value();
        break;
    case IBUS2Cmd::GET_PARAM:
        send_resp_get_param((const IBUS2_Cmd_GetParam *)&_pending_cmd.msg);
        break;
    case IBUS2Cmd::SET_PARAM:
        send_resp_set_param((const IBUS2_Cmd_SetParam *)&_pending_cmd.msg);
        break;
    default:
        break;
    }
}

void AP_IBUS2_Slave::send_resp_get_type()
{
    const IBUS2_Pkt<IBUS2_Resp_GetType> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::GET_TYPE,
        IBUS2_Resp_GetType{
            (uint8_t)IBUS2DeviceType::DIGITAL_SERVO,
            16,  // value_length: use max as recommended
            1,   // channels_types
            1,   // failsafe
        },
    };
    _port->write((const uint8_t *)&r, sizeof(r));
}

void AP_IBUS2_Slave::send_resp_get_value()
{
    IBUS2_Pkt<IBUS2_Resp_GetValue> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::GET_VALUE,
        IBUS2_Resp_GetValue{{}, IBUS2_TELEM_VID, IBUS2_TELEM_PID},
    };
    populate_sensor_data(r.msg.value);
    r.update_crc();
    _port->write((const uint8_t *)&r, sizeof(r));
}

void AP_IBUS2_Slave::send_resp_get_param(const IBUS2_Cmd_GetParam *cmd)
{
    const IBUS2_Pkt<IBUS2_Resp_GetParam> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::GET_PARAM,
        IBUS2_Resp_GetParam{cmd->param_type, 0},  // param_length: not supported
    };
    _port->write((const uint8_t *)&r, sizeof(r));
}

void AP_IBUS2_Slave::send_resp_set_param(const IBUS2_Cmd_SetParam *cmd)
{
    const IBUS2_Pkt<IBUS2_Resp_SetParam> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::SET_PARAM,
        IBUS2_Resp_SetParam{cmd->param_type, 0},  // param_length: not supported
    };
    _port->write((const uint8_t *)&r, sizeof(r));
}

uint8_t AP_IBUS2_Slave::populate_sensor_data(uint8_t *value14)
{
    // Pack sensor data into the 14-byte value field.
    // Format from spec §3 (sIB2_ResponseGetValue):
    //   PackType:4, PackLength:6, PackCurIndex:6  (= 2 bytes)
    //   [Data1Type:6, Data1Format:5, Data1Unit:5, Data1:16] × N data points
    //
    // For simplicity we pack one data point at a time (PackLength=1).
    // Each data point uses: type(6b)+format(5b)+unit(5b)+value(16b) = 4 bytes.
    // With PackType(4b)+PackLength(6b)+PackCurIndex(6b) = 2 bytes header.
    // Total per call: 6 bytes, leaving 8 bytes unused (zeroed).

    memset(value14, 0, 14);

    // value14[0..1] = PackType(4)|PackLength(6)|PackCurIndex(6)
    // PackType=0 (normal), PackLength=1 (1 data point), PackCurIndex=0
    value14[0] = 0x00;  // PackType=0, PackLength hi bits
    value14[1] = (1 << 2) | 0;  // PackLength=1, PackCurIndex=0

    // value14[2..5] = first data point
    // Report voltage (sensor type 1, format 0, unit 0)
    float voltage = 0.0f;
#if AP_BATTERY_ENABLED
    voltage = AP::battery().voltage();
#endif
    const int16_t voltage_raw = (int16_t)(voltage * 100);  // in 10mV units (0.01V per count)

    value14[2] = ((uint8_t)IBUS2SensorType::VOLTAGE & 0x3F);  // Data1Type (6 bits)
    value14[3] = 0;  // Data1Format(5) | Data1Unit(5) hi
    value14[4] = (uint8_t)(voltage_raw & 0xFF);
    value14[5] = (uint8_t)((voltage_raw >> 8) & 0xFF);

    return 1;
}

#endif  // AP_IBUS2_SLAVE_ENABLED
