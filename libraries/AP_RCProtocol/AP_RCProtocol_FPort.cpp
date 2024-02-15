/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  FRSky FPort implementation, with thanks to BetaFlight for
  specification and code reference
 */

#include "AP_RCProtocol_FPort.h"

#if AP_RCPROTOCOL_FPORT_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

#define FRAME_HEAD 0x7E
#define FRAME_DLE  0x7D
#define FRAME_XOR  0x20
#define FRAME_LEN_CONTROL 0x19
#define FRAME_LEN_DOWNLINK 0x08
#define MIN_FRAME_SIZE 12
#define MAX_CHANNELS 16

#define FLAGS_FAILSAFE_BIT	3
#define FLAGS_FRAMELOST_BIT	2

#define CHAN_SCALE_FACTOR1 1000U
#define CHAN_SCALE_FACTOR2 1600U
#define CHAN_SCALE_OFFSET 875U

#define FPORT_TYPE_CONTROL 0
#define FPORT_TYPE_DOWNLINK 1
#define FPORT_PRIM_NULL 0x00
#define FPORT_PRIM_DATA 0x10
#define FPORT_PRIM_READ 0x30
#define FPORT_PRIM_WRITE 0x31

#define MAX_FPORT_CONSECUTIVE_FRAMES 2

struct PACKED FPort_Frame {
    uint8_t header; // 0x7E
    uint8_t len;    // 0x19 for control, 0x08 for downlink
    uint8_t type;
    union {
        struct PACKED {
            uint8_t data[22]; // 16 11-bit channels
            uint8_t flags;
            uint8_t rssi;
            uint8_t crc;
            uint8_t end;
        } control;
        struct PACKED {
            uint8_t prim;
            uint16_t appid;
            uint8_t data[4];
            uint8_t crc;
            uint8_t end;
        } downlink;
    };
};

static_assert(sizeof(FPort_Frame) == FPORT_CONTROL_FRAME_SIZE, "FPort_Frame incorrect size");

// constructor
AP_RCProtocol_FPort::AP_RCProtocol_FPort(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted)
{}

// decode a full FPort control frame
void AP_RCProtocol_FPort::decode_control(const FPort_Frame &frame)
{
    uint16_t values[MAX_CHANNELS];

    decode_11bit_channels(frame.control.data, MAX_CHANNELS, values, CHAN_SCALE_FACTOR1, CHAN_SCALE_FACTOR2, CHAN_SCALE_OFFSET);

    bool failsafe = ((frame.control.flags & (1 << FLAGS_FAILSAFE_BIT)) != 0);

    // fport rssi 0-50, ardupilot rssi 0-255, scale factor 255/50=5.1
    const uint8_t scaled_rssi = MIN(frame.control.rssi * 5.1f, 255);

    add_input(MAX_CHANNELS, values, failsafe, scaled_rssi);
}

/*
  decode a full FPort downlink frame
*/
void AP_RCProtocol_FPort::decode_downlink(const FPort_Frame &frame)
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && AP_FRSKY_SPORT_TELEM_ENABLED
    switch (frame.downlink.prim) {
        case FPORT_PRIM_DATA:
            // we've seen at least one 0x10 frame
            rx_driven_frame_rate = true;
            break;
        case FPORT_PRIM_NULL:
            if (rx_driven_frame_rate) {
                return;
            }
            // with 0x00 and no rx control we have a constraint
            // on max consecutive frames
            if (consecutive_telemetry_frame_count >= MAX_FPORT_CONSECUTIVE_FRAMES) {
                consecutive_telemetry_frame_count = 0;
                return;
            } else {
                consecutive_telemetry_frame_count++;
            }
            break;
        case FPORT_PRIM_READ:
        case FPORT_PRIM_WRITE:
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL        
            AP_Frsky_Telem::set_telem_data(frame.downlink.prim, frame.downlink.appid, le32toh_ptr(frame.downlink.data));
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL            
            // do not respond to 0x30 and 0x31
            return;
    }

    /*
      if we are getting FPORT over a UART then we can ask the FrSky
      telem library for some passthrough data to send back, enabling
      telemetry on the receiver via the same uart pin as we use for
      incoming RC frames
     */

    AP_HAL::UARTDriver *uart = get_UART();
    if (!uart) {
        return;
    }

    /*
      get SPort data from FRSky_Telem or send a null frame.
      We save the data to a variable so in case we're late we'll
      send it in the next call, this prevents corruption of
      status text messages
     */
    if (!telem_data.available) {
        uint8_t packet_count;
        if (!AP_Frsky_Telem::get_telem_data(&telem_data.packet, packet_count, 1)) {
            // nothing to send, send a null frame
            telem_data.packet.frame = 0x00;
            telem_data.packet.appid = 0x00;
            telem_data.packet.data = 0x00;
        }
        telem_data.available = true;
    }
    /*
      check that we haven't been too slow in responding to the new
      UART data. If we respond too late then we will corrupt the next
      incoming control frame
     */
    uint64_t tend = uart->receive_time_constraint_us(1);
    uint64_t now = AP_HAL::micros64();
    uint64_t tdelay = now - tend;
    if (tdelay > 2500) {
        // we've been too slow in responding
        return;
    }
    uint8_t buf[10];

    buf[0] = 0x08;
    buf[1] = 0x81;
    buf[2] = telem_data.packet.frame;
    buf[3] = telem_data.packet.appid & 0xFF;
    buf[4] = telem_data.packet.appid >> 8;
    memcpy(&buf[5], &telem_data.packet.data, 4);
    buf[9] = crc_sum8_with_carry(&buf[0], 9);

    // perform byte stuffing per FPort spec
    uint8_t len = 0;
    uint8_t buf2[sizeof(buf)*2+1];

    if (rc().option_is_enabled(RC_Channels::Option::FPORT_PAD)) {
        // this padding helps on some uarts that have hw pullups
        buf2[len++] = 0xff;
    }

    for (uint8_t i=0; i<sizeof(buf); i++) {
        uint8_t c = buf[i];
        if (c == FRAME_DLE || buf[i] == FRAME_HEAD) {
            buf2[len++] = FRAME_DLE;
            buf2[len++] = c ^ FRAME_XOR;
        } else {
            buf2[len++] = c;
        }
    }
    uart->write(buf2, len);
    // get fresh telem_data in the next call
    telem_data.available = false;
#endif
}

/*
  process a FPort input pulse of the given width
 */
void AP_RCProtocol_FPort::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    if (have_UART()) {
        // if we can use a UART we would much prefer to, as it allows
        // us to send SPORT data out
        return;
    }
    uint32_t w0 = width_s0;
    uint32_t w1 = width_s1;
    if (inverted) {
        w0 = saved_width;
        w1 = width_s0;
        saved_width = width_s1;
    }
    uint8_t b;
    if (ss.process_pulse(w0, w1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

// support byte input
void AP_RCProtocol_FPort::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= 2000U);

    byte_input.last_byte_us = timestamp_us;

    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
        byte_input.got_DLE = false;
    }
    if (b != FRAME_HEAD && byte_input.ofs == 0) {
        // definately not FPort, missing header byte
        return;
    }

    // handle byte-stuffing decode
    if (byte_input.got_DLE) {
        b ^= FRAME_XOR;
        byte_input.got_DLE = false;
    } else if (b == FRAME_DLE) {
        byte_input.got_DLE = true;
        return;
    }

    byte_input.buf[byte_input.ofs++] = b;

    const FPort_Frame *frame = (const FPort_Frame *)&byte_input.buf[0];

    if (byte_input.ofs == 2) {
        // check for valid lengths
        if (frame->len != FRAME_LEN_CONTROL &&
            frame->len != FRAME_LEN_DOWNLINK) {
            // invalid, reset
            goto reset;
        }
    }

    if (byte_input.ofs == 3) {
        // check for valid lengths
        if ((frame->type == FPORT_TYPE_CONTROL && frame->len != FRAME_LEN_CONTROL) ||
            (frame->type == FPORT_TYPE_DOWNLINK && frame->len != FRAME_LEN_DOWNLINK)) {
            goto reset;
        }
        if (frame->type != FPORT_TYPE_CONTROL && frame->type != FPORT_TYPE_DOWNLINK) {
            // invalid type
            goto reset;
        }
    }

    if (frame->type == FPORT_TYPE_CONTROL && byte_input.ofs == FRAME_LEN_CONTROL + 4) {
        log_data(AP_RCProtocol::FPORT, timestamp_us, byte_input.buf, byte_input.ofs);
        if (check_checksum()) {
            decode_control(*frame);
        }
        goto reset;
    } else if (frame->type == FPORT_TYPE_DOWNLINK && byte_input.ofs == FRAME_LEN_DOWNLINK + 4) {
        log_data(AP_RCProtocol::FPORT, timestamp_us, byte_input.buf, byte_input.ofs);
        if (check_checksum()) {
            decode_downlink(*frame);
        }
        goto reset;
    }
    if (byte_input.ofs == sizeof(byte_input.buf)) {
        goto reset;
    }
    return;

reset:
    byte_input.ofs = 0;
    byte_input.got_DLE = false;
}

// check checksum byte
bool AP_RCProtocol_FPort::check_checksum(void)
{
    const uint8_t len = byte_input.buf[1]+2;
    return crc_sum8_with_carry(&byte_input.buf[1], len) == 0x00;
}

// support byte input
void AP_RCProtocol_FPort::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}

#endif  // AP_RCPROTOCOL_FPORT_ENABLED
