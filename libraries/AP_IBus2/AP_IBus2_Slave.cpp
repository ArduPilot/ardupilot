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

#include "AP_IBus2_Slave.h"

#if AP_IBUS2_SLAVE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
static_assert(AP_IBUS2_MAX_CHANNELS <= MAX_RCIN_CHANNELS,
              "AP_IBUS2_MAX_CHANNELS must not exceed MAX_RCIN_CHANNELS");
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// IBUS2 baud rate
#define IBUS2_BAUD 1500000

// Telemetry Adapter VID/PID (Telemetry_Adapter_Protocol___20240923_en_.pdf §4)
#define IBUS2_TELEM_VID 0x01
#define IBUS2_TELEM_PID 0x03

// SES factor table — indexed by 5-bit ChannelType value.
// Lower 4 bits (ct & 0xF) = NbBits per channel in the subtype=0 payload.
// Bit 4 doubles the scale, giving higher resolution for standard RC channels.
static const uint32_t ses_factors[32] = {
    0,          0,          0x40000000, 0,          0,          0,          0x028F5C29, 0x0147AE15,
    0x0083126F, 0x00418938, 0x0020C49C, 0x0010624E, 0x00083127, 0x00041894, 0,          0,
    0,          0,          0,          0x20000000, 0x10000000, 0x06666667, 0x03333334, 0,
    0x0147AE15, 0x00A3D70B, 0x00418938, 0x0020C49C, 0x0010624E, 0x00083127, 0,          0,
};

// Read nb_bits (LSB-first) from payload at bit_pos.
// Reads at most 4 bytes, stopping at payload_len boundary (zero-pads the rest).
static uint32_t ses_read_bits(const uint8_t *payload, uint8_t payload_len,
                               uint16_t bit_pos, uint8_t nb_bits)
{
    const uint16_t byte_idx = bit_pos / 8;
    const uint8_t  bit_off  = bit_pos % 8;
    uint32_t chunk = 0;
    for (uint8_t b = 0; b < 4 && (byte_idx + b) < payload_len; b++) {
        chunk |= (uint32_t)payload[byte_idx + b] << (b * 8);
    }
    return (chunk >> bit_off) & ((1U << nb_bits) - 1U);
}

#if AP_IBUS2_LOG_RAW_FRAMES && HAL_LOGGING_ENABLED
void AP_IBus2_Slave::log_raw_frame(const uint8_t *buf, uint8_t len) const
{
    uint32_t u32[10] {};
    const uint8_t capped = MIN(len, (uint8_t)sizeof(u32));
    memcpy(u32, buf, capped);
    // Prot value 22 = AP_RCProtocol::IBUS2 (not included here to avoid circular dependency)
    AP::logger().WriteStreaming("RCDA", "TimeUS,TS,Prot,Len,U0,U1,U2,U3,U4,U5,U6,U7,U8,U9", "QIBBIIIIIIIIII",
                               AP_HAL::micros64(),
                               AP_HAL::micros(),
                               (uint8_t)22,
                               capped,
                               u32[0], u32[1], u32[2], u32[3], u32[4],
                               u32[5], u32[6], u32[7], u32[8], u32[9]);
}
#endif  // AP_IBUS2_LOG_RAW_FRAMES && HAL_LOGGING_ENABLED

AP_IBus2_Slave *AP_IBus2_Slave::_singleton;

AP_IBus2_Slave::AP_IBus2_Slave()
{
    _singleton = this;
}

void AP_IBus2_Slave::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IBUS2_Slave, 0);
    if (_port == nullptr) {
        return;
    }

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_IBus2_Slave::update, void));
}

void AP_IBus2_Slave::update()
{
    if (!_initialized) {
        _port->set_options(_port->OPTION_HDPLEX);
        _port->begin(IBUS2_BAUD);  // sets ownership
        _initialized = true;
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

uint8_t AP_IBus2_Slave::get_rc_channel_count() const
{
    WITH_SEMAPHORE(_rc_state.sem);
    return _rc_state.channel_count;
}

bool AP_IBus2_Slave::get_rc_channels(uint16_t *channels, uint8_t &count) const
{
    WITH_SEMAPHORE(_rc_state.sem);
    if (_rc_state.channel_count == 0) {
        count = 0;
        return false;
    }
    count = _rc_state.channel_count;
    memcpy(channels, _rc_state.channels, count * sizeof(uint16_t));
    return true;
}

uint32_t AP_IBus2_Slave::get_rc_last_update_ms() const
{
    WITH_SEMAPHORE(_rc_state.sem);
    return _rc_state.last_update_ms;
}

bool AP_IBus2_Slave::get_failsafe() const
{
    WITH_SEMAPHORE(_rc_state.sem);
    return _rc_state.failsafe;
}

void AP_IBus2_Slave::process_rx()
{
    // In half-duplex mode our own TX bytes are echoed back; discard them first.
    _tx_pending_echo = _port->discard_bytes(_tx_pending_echo);
    if (_tx_pending_echo > 0) {
        return;
    }

    const uint16_t to_read = MIN(_port->available(), 512U);

    for (uint16_t i = 0; i < to_read; i++) {
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
#if AP_IBUS2_LOG_RAW_FRAMES && HAL_LOGGING_ENABLED
                    log_raw_frame(_rx_buf, _frame1_expected_len);
#endif
                    handle_frame1(_rx_buf, _frame1_expected_len);
                }
                _rx_state = RxState::WAIT_HEADER;
            }
            break;

        case RxState::IN_FRAME2:
            _rx_buf[_rx_len++] = b;
            if (_rx_len == IBUS2_FRAME2_SIZE) {
#if AP_IBUS2_LOG_RAW_FRAMES && HAL_LOGGING_ENABLED
                log_raw_frame(_rx_buf, IBUS2_FRAME2_SIZE);
#endif
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

void AP_IBus2_Slave::handle_frame1(const uint8_t *buf, uint8_t len)
{
    const IBUS2_Frame1_Header *hdr = reinterpret_cast<const IBUS2_Frame1_Header *>(buf);
    const bool failsafe = hdr->failsafe || hdr->sync_lost;

    const uint8_t *payload     = buf + 3;
    const uint8_t  payload_len = len - 4;  // exclude 3-byte header and 1-byte CRC

    if (hdr->subtype == 1) {
        // Decompression key: extract packed 5-bit channel types (LSB-first).
        uint8_t count = 0;
        uint16_t bit_pos = 0;
        const uint16_t total_bits = payload_len * 8;
        while (bit_pos + 5 <= total_bits &&
               count < ARRAY_SIZE(frame1_handling.channel_types)) {
            const uint8_t ct = (uint8_t)ses_read_bits(payload, payload_len, bit_pos, 5);
            if ((ct & 0xF) < 2) {
                break;  // NbBits<2 marks end of active channels
            }
            frame1_handling.channel_types[count++] = ct;
            bit_pos += 5;
        }
        frame1_handling.channel_count = count;
        frame1_handling.have_decompression_key = true;
        return;
    }

    if (hdr->subtype != 0 || !frame1_handling.have_decompression_key) {
        return;
    }

    // subtype=0: SES_UnpackChannels decode (sign-magnitude, LSB-first bit packing).
    // Decode into a local buffer first to minimise time holding the semaphore.
    uint16_t new_channels[ARRAY_SIZE(_rc_state.channels)];
    const uint16_t total_bits = payload_len * 8;
    uint16_t bit_pos = 0;
    uint8_t n_decoded = 0;

    for (uint8_t i = 0;
         i < frame1_handling.channel_count && i < ARRAY_SIZE(_rc_state.channels);
         i++) {
        const uint8_t ct      = frame1_handling.channel_types[i];
        const uint8_t nb_bits = ct & 0xF;

        if (bit_pos + nb_bits > total_bits) {
            break;
        }

        uint32_t raw = ses_read_bits(payload, payload_len, bit_pos, nb_bits);
        bit_pos += nb_bits;

        const uint32_t sign_bit = 1U << (nb_bits - 1);
        const uint32_t mag_mask = sign_bit - 1U;

        // Failsafe sentinels: keep-failsafe or stop-failsafe
        if (raw == sign_bit || (nb_bits >= 6 && raw == sign_bit + 1U)) {
            n_decoded++;
            continue;
        }

        const bool negative = (raw & sign_bit) != 0;
        if (negative) {
            raw = (uint32_t)(-(int32_t)raw) & mag_mask;
        }

        const uint32_t factor  = (ct < ARRAY_SIZE(ses_factors)) ? ses_factors[ct] : 0U;
        const uint64_t ses     = ((uint64_t)raw * factor + (1U << 15)) >> 16;
        const uint64_t max_ses = ((uint64_t)mag_mask * factor + (1U << 15)) >> 16;
        const uint32_t ms      = MAX(1U, (uint32_t)max_ses);
        const uint32_t sv      = (uint32_t)MIN(ses, (uint64_t)ms);
        const uint32_t offset  = sv * 512U / ms;

        new_channels[i] = (uint16_t)(1500 + (negative ? -(int32_t)offset : (int32_t)offset));
        n_decoded++;
    }

    {
        WITH_SEMAPHORE(_rc_state.sem);
        const bool was_empty = (_rc_state.channel_count == 0);
        memcpy(_rc_state.channels, new_channels, n_decoded * sizeof(uint16_t));
        _rc_state.channel_count = n_decoded;
        _rc_state.last_update_ms = AP_HAL::millis();
        _rc_state.failsafe = failsafe;
        if (was_empty && n_decoded > 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IBUS2: %u RC chans from receiver", (unsigned)n_decoded);
        }
    }

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("IB2C", "TimeUS,Ch0,Ch1,Ch2,Ch3,Ch4,Ch5,Ch6,Ch7",
                                "QHHHHHHHH",
                                AP_HAL::micros64(),
                                new_channels[0], new_channels[1], new_channels[2], new_channels[3],
                                new_channels[4], new_channels[5], new_channels[6], new_channels[7]);
#endif
}

void AP_IBus2_Slave::handle_frame2(const IBUS2_Pkt<IBUS2_Frame2> *f2)
{
    // Only respond if not already waiting to respond
    if (_response_pending) {
        return;
    }
    memcpy(&_pending_cmd, f2, sizeof(_pending_cmd));
    _frame2_end_us = AP_HAL::micros();
    _response_pending = true;
}

void AP_IBus2_Slave::send_frame3()
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

void AP_IBus2_Slave::send_resp_get_type()
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
    port_write((const uint8_t *)&r, sizeof(r));
}

void AP_IBus2_Slave::send_resp_get_value()
{
    IBUS2_Pkt<IBUS2_Resp_GetValue> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::GET_VALUE,
        IBUS2_Resp_GetValue{{}, IBUS2_TELEM_VID, IBUS2_TELEM_PID},
    };
    populate_sensor_data(r.msg.value);
    r.update_crc();
    port_write((const uint8_t *)&r, sizeof(r));
}

void AP_IBus2_Slave::send_resp_get_param(const IBUS2_Cmd_GetParam *cmd)
{
    const IBUS2_Pkt<IBUS2_Resp_GetParam> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::GET_PARAM,
        IBUS2_Resp_GetParam{cmd->param_type, 0},  // param_length: not supported
    };
    port_write((const uint8_t *)&r, sizeof(r));
}

void AP_IBus2_Slave::send_resp_set_param(const IBUS2_Cmd_SetParam *cmd)
{
    const IBUS2_Pkt<IBUS2_Resp_SetParam> r{
        IBUS2_PKT_RESPONSE,
        (uint8_t)IBUS2Cmd::SET_PARAM,
        IBUS2_Resp_SetParam{cmd->param_type, 0},  // param_length: not supported
    };
    port_write((const uint8_t *)&r, sizeof(r));
}

uint8_t AP_IBus2_Slave::populate_sensor_data(uint8_t *value14)
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
