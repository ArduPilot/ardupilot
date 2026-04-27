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
#include <AP_IBus2/AP_IBus2_Slave.h>
#include <stdio.h>

using namespace SITL;

// ~7 ms between master cycles
#define IBUS2_MASTER_CYCLE_US 7000

// After the slave confirms it wants channel types, send Frame 1 subtype=1
// immediately and then every SUBTYPE1_INTERVAL cycles (~350 ms at 7 ms/cycle).
#define SUBTYPE1_INTERVAL 50

// ---------------------------------------------------------------------------
// SES channel encoding constants.
// All simulated channels use ChannelType 0x1B (NbBits=11, factor=0x0020C49C),
// matching what real FlySky receivers use for standard RC channels.
// ---------------------------------------------------------------------------
static const uint8_t  SIM_CHAN_TYPE = 0x1B;
static const uint8_t  SIM_NB_BITS  = 11;
static const uint32_t SIM_FACTOR   = 0x0020C49CU;
// SIM_MAX_SES = ((2^(NB_BITS-1) - 1) * FACTOR + 0x8000) >> 16
static const uint32_t SIM_MAX_SES  = 33521U;

// Fallback RC channels used when no UDP input is available (µs).
// Values match the AP_RCProtocol_UDP defaults so the two sources agree.
static const uint16_t sim_channels[AP_IBUS2_MAX_CHANNELS] = {
    1500, 1500, 1000, 1500,   // roll, pitch, throttle, yaw
    1800, 1000, 1000, 1800,   // aux 5-8 (matches AP_RCProtocol_UDP defaults)
    1500, 1500, 1500, 1500,   // aux 9-12
    1500, 1500, 1500, 1500,   // aux 13-16
    1500, 1500,               // aux 17-18
};

// ---------------------------------------------------------------------------
// Local CRC-8 helpers (polynomial 0x25, initial value 0xFF).
// Duplicated from AP_IBus2.cpp so the SITL library is independent of the
// vehicle library.
// ---------------------------------------------------------------------------
static uint8_t sitl_ibus2_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0xFF;
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
// SES encoding helpers
// ---------------------------------------------------------------------------

// Write nb_bits of value (LSB-first) into payload at bit_pos, then advance bit_pos.
static void ses_write_bits(uint8_t *payload, uint16_t &bit_pos, uint32_t value, uint8_t nb_bits)
{
    for (uint8_t b = 0; b < nb_bits; b++) {
        const uint8_t byte_idx = (bit_pos + b) / 8;
        const uint8_t bit_off  = (bit_pos + b) % 8;
        if ((value >> b) & 1U) {
            payload[byte_idx] |= (1U << bit_off);
        }
    }
    bit_pos += nb_bits;
}

// Encode a PWM µs value to an SIM_NB_BITS-wide SES raw value (two's-complement),
// matching the SES_UnpackChannels decode in AP_IBus2_Slave.
static uint32_t ses_encode_us(uint16_t us_val)
{
    const uint32_t mag_mask  = (1U << (SIM_NB_BITS - 1)) - 1U;  // 1023
    const uint32_t full_mask = (1U << SIM_NB_BITS) - 1U;        // 2047

    int32_t offset = (int32_t)us_val - 1500;
    const bool negative = (offset < 0);
    uint32_t abs_offset = negative ? (uint32_t)(-offset) : (uint32_t)offset;
    if (abs_offset > 512U) {
        abs_offset = 512U;
    }

    // Inverse of: offset = ses * 512 / MAX_SES
    const uint32_t ses = abs_offset * SIM_MAX_SES / 512U;

    // Inverse of: ses = (raw * FACTOR + 0x8000) >> 16
    uint32_t raw_mag = (uint32_t)(((uint64_t)ses * 65536U + SIM_FACTOR / 2U) / SIM_FACTOR);
    if (raw_mag > mag_mask) {
        raw_mag = mag_mask;
    }

    if (!negative || raw_mag == 0U) {
        return raw_mag;
    }
    // Negative: store as two's complement in SIM_NB_BITS
    return (uint32_t)(-(int32_t)raw_mag) & full_mask;
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

    // Inform AP_RCProtocol_UDP whether we are routing its data through the
    // IBus2 encode/decode chain.  This suppresses UDP's own add_input() call.
    auto *udp = AP_RCProtocol_UDP::get_singleton();
    if (udp != nullptr) {
        udp->set_ibus2_active(AP_IBus2_Slave::get_singleton() != nullptr);
    }

    const uint32_t now_us = AP_HAL::micros();
    if ((now_us - _last_send_us) < IBUS2_MASTER_CYCLE_US) {
        read_frame3();
        return;
    }
    _last_send_us = now_us;

    send_frame1(aircraft);
    send_frame2();
}

void IBus2Master::send_frame1(const Aircraft &aircraft)
{
    // Resolve channel source: prefer live UDP data, fall back to sim_channels[].
    uint16_t udp_channels[AP_IBUS2_MAX_CHANNELS];
    uint8_t udp_count = 0;
    auto *udp = AP_RCProtocol_UDP::get_singleton();
    const bool have_udp = (udp != nullptr) && udp->get_channels(udp_channels, udp_count);

    const uint16_t *src = have_udp ? udp_channels : sim_channels;
    const uint8_t   n   = have_udp ? udp_count    : (uint8_t)AP_IBUS2_MAX_CHANNELS;

    // Only include the decompression key if the slave has confirmed it needs it.
    if (_device_wants_channel_types) {
        const bool time_for_subtype1 = (_frame1_cycle % SUBTYPE1_INTERVAL == 0);
        if (_send_subtype1_now || time_for_subtype1) {
            send_frame1_subtype1(src, n);
            _send_subtype1_now = false;
        }
    }
    _frame1_cycle++;
    send_frame1_subtype0(src, n);
}

void IBus2Master::send_frame1_subtype1(const uint16_t *channels, uint8_t n)
{
    // Payload: n × 5-bit ChannelType values packed LSB-first.
    const uint8_t payload_len = (n * 5 + 7) / 8;
    const uint8_t total_len   = 3 + payload_len + 1;

    uint8_t buf[IBUS2_FRAME1_MAX] {};
    buf[0] = IBUS2_PKT_CHANNELS | (1U << 2);  // subtype=1
    buf[1] = total_len;
    buf[2] = 7;  // addr_level1=7

    uint8_t *payload = buf + 3;
    uint16_t bit_pos = 0;
    for (uint8_t i = 0; i < n; i++) {
        ses_write_bits(payload, bit_pos, SIM_CHAN_TYPE, 5);
    }

    sitl_ibus2_crc8_write(buf, total_len);
    write_to_autopilot((const char *)buf, total_len);
}

void IBus2Master::send_frame1_subtype0(const uint16_t *channels, uint8_t n)
{
    // Payload: n channels × SIM_NB_BITS bits each, packed LSB-first.
    const uint8_t payload_len = (n * SIM_NB_BITS + 7) / 8;
    const uint8_t total_len   = 3 + payload_len + 1;

    uint8_t buf[IBUS2_FRAME1_MAX] {};
    buf[0] = IBUS2_PKT_CHANNELS;  // subtype=0, sync_lost=0, failsafe=0
    buf[1] = total_len;
    buf[2] = 7;  // addr_level1=7

    uint8_t *payload = buf + 3;
    uint16_t bit_pos = 0;
    for (uint8_t i = 0; i < n; i++) {
        ses_write_bits(payload, bit_pos, ses_encode_us(channels[i]), SIM_NB_BITS);
    }

    sitl_ibus2_crc8_write(buf, total_len);
    write_to_autopilot((const char *)buf, total_len);
}

void IBus2Master::send_frame2()
{
    // Before we have a GET_TYPE response, keep sending GET_TYPE so the slave can
    // tell us what data it needs (channels_types, failsafe).
    // Once connected, cycle GET_VALUE for telemetry.
    IBUS2Cmd cmd;
    if (!_device_wants_channel_types && !_device_wants_failsafe) {
        cmd = IBUS2Cmd::GET_TYPE;
    } else {
        // Primarily GET_VALUE; occasionally GET_PARAM / SET_PARAM to exercise those paths.
        static const IBUS2Cmd connected_cmds[] = {
            IBUS2Cmd::GET_VALUE,
            IBUS2Cmd::GET_VALUE,
            IBUS2Cmd::GET_VALUE,
            IBUS2Cmd::GET_VALUE,
            IBUS2Cmd::GET_PARAM,
            IBUS2Cmd::SET_PARAM,
        };
        cmd = connected_cmds[_cmd_cycle % ARRAY_SIZE(connected_cmds)];
        _cmd_cycle++;
    }

    uint8_t buf[IBUS2_FRAME2_SIZE] {};
    buf[0] = (uint8_t)(IBUS2_PKT_COMMAND | ((uint8_t)cmd << 2));

    if (cmd == IBUS2Cmd::SET_PARAM) {
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
                    ::fprintf(stderr, "IBus2Master: GET_TYPE type=0x%02x vlen=%u channels_types=%u failsafe=%u\n",
                              (unsigned)r->type, (unsigned)r->value_length,
                              (unsigned)r->channels_types, (unsigned)r->failsafe);
                    // Record what the slave needs and trigger immediate subtype=1 if required.
                    if (r->channels_types && !_device_wants_channel_types) {
                        _device_wants_channel_types = true;
                        _send_subtype1_now = true;
                    }
                    _device_wants_failsafe = r->failsafe;
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
