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
  FRSky FPort2 implementation, with thanks to FrSky for
  specification
 */

#include "AP_RCProtocol_FPort2.h"

#if AP_RCPROTOCOL_FPORT2_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define FRAME_LEN_16 0x18
#define FRAME_LEN_8  0x0D
#define FRAME_LEN_24 0x23
#define FRAME_LEN_DOWNLINK 0x08

#define FRAME_TYPE_CHANNEL 0xFF
#define FRAME_TYPE_FC 0x1B

#define MAX_CHANNELS 24

#define FLAGS_FAILSAFE_BIT	3
#define FLAGS_FRAMELOST_BIT	2

#define CHAN_SCALE_FACTOR1 1000U
#define CHAN_SCALE_FACTOR2 1600U
#define CHAN_SCALE_OFFSET 875U

#define FPORT2_PRIM_NULL 0x00
#define FPORT2_PRIM_READ 0x30
#define FPORT2_PRIM_WRITE 0x31

struct PACKED FPort2_Frame {
    uint8_t len;
    uint8_t type;
    union {
        uint8_t data[36];
        struct PACKED {
            uint8_t prim;
            uint16_t appid;
            uint8_t data[4];
            uint8_t crc;
        } downlink;
    };
};

static_assert(sizeof(FPort2_Frame) == FPORT2_CONTROL_FRAME_SIZE, "FPort2_Frame incorrect size");

// constructor
AP_RCProtocol_FPort2::AP_RCProtocol_FPort2(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted)
{}

// decode a full FPort2 control frame
void AP_RCProtocol_FPort2::decode_control(const FPort2_Frame &frame)
{
    uint16_t values[MAX_CHANNELS];

    decode_11bit_channels(frame.data, chan_count, values, CHAN_SCALE_FACTOR1, CHAN_SCALE_FACTOR2, CHAN_SCALE_OFFSET);

    const uint8_t b_flags = frame.data[byte_input.control_len-5];
    const uint8_t b_rssi = frame.data[byte_input.control_len-4];

    bool failsafe = ((b_flags & (1 << FLAGS_FAILSAFE_BIT)) != 0);

    // fport2 rssi 0-50, ardupilot rssi 0-255, scale factor 255/50=5.1
    const uint8_t scaled_rssi = MIN(b_rssi * 5.1f, 255);

    add_input(chan_count, values, failsafe, scaled_rssi);
}

/*
  decode a full FPort2 downlink frame
*/
void AP_RCProtocol_FPort2::decode_downlink(const FPort2_Frame &frame)
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware)
    /*
      if we are getting FPORT2 over a UART then we can ask the FrSky
      telem library for some passthrough data to send back, enabling
      telemetry on the receiver via the same uart pin as we use for
      incoming RC frames
     */

    AP_HAL::UARTDriver *uart = get_UART();
    if (!uart) {
        return;
    }

    // no telemetry for 24ch fport2, timing is too tight
    if (chan_count > 16) {
        return;
    }

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // 0x30,0x31 read/write frames
    // allowed reply prim: 0x00,0x32
    if (frame.downlink.prim == FPORT2_PRIM_READ || frame.downlink.prim == FPORT2_PRIM_WRITE) {
        AP_Frsky_Telem::set_telem_data(frame.downlink.prim, frame.downlink.appid, le32toh_ptr(frame.downlink.data));
    }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

    /*
     we're only interested in "flight controller" requests i.e. 0x1B
     Notes:
     need to check how to handle multiple sensor IDs (we probably should respond to 0x1E as well)
     with fport2 we need to account for the IDs we send sensor data from
     if we respond to multiple sensors like we do for sport we need to make sure GPS data
     is always sent with the same sensor ID or else OpenTX will see multiple sensor instances (one for each sensor we respond to)
     (need to double check on OpenTX if fport2 carries sensor IDs up to the OpenTX sensor tables)
    */
    if (frame.type != FRAME_TYPE_FC) {
        /*
         get SPort data from FRSky_Telem
         when we are not polled for data we prepare telemetry data for the next poll
         we save the data to a variable so in case we're late we'll
         send it in the next call, this prevents corruption of status text messages
        */
        if (!telem_data.available) {
            uint8_t packet_count;
            if (!AP_Frsky_Telem::get_telem_data(&telem_data.packet, packet_count, 1)) {
                telem_data.packet.frame = 0x00;
                telem_data.packet.appid = 0x00;
                telem_data.packet.data = 0x00;
            }
            telem_data.available = true;
        }
        return;
    }

    /*
      check that we haven't been too slow in responding to the new
      UART data. If we respond too late then we will corrupt the next
      incoming control frame.
      16ch fport2: 4.5ms window before next control frame
      specs require to release the bus at least 1.5ms before next control frame (uplink frame takes 0.851ms)
     */
    uint64_t tend = uart->receive_time_constraint_us(1);
    uint64_t now = AP_HAL::micros64();
    uint64_t tdelay = now - tend;
    if (tdelay > 2500) {
        // we've been too slow in responding
        return;
    }

    // we initialize to a default null frame
    uint8_t buf[10] {};

    buf[0] = 0x08;
    buf[1] = frame.type;
    // do not consume telemetry data for invalid downlink frames, i.e. incoming prim == 0x00
    if (frame.downlink.prim != FPORT2_PRIM_NULL) {
        buf[2] = telem_data.packet.frame;
        buf[3] = telem_data.packet.appid & 0xFF;
        buf[4] = telem_data.packet.appid >> 8;
        memcpy(&buf[5], &telem_data.packet.data, 4);
        // get fresh telem_data in the next call
        telem_data.available = false;
    }
    buf[9] = crc_sum8_with_carry(&buf[1], 8);
    
    uart->write(buf, sizeof(buf));
#endif
}

/*
  process a FPort2 input pulse of the given width
 */
void AP_RCProtocol_FPort2::process_pulse(uint32_t width_s0, uint32_t width_s1)
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
void AP_RCProtocol_FPort2::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= 2000U);

    byte_input.last_byte_us = timestamp_us;

    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
    }

    if (byte_input.ofs == 0) {
        switch (b) {
        case FRAME_LEN_8:
            byte_input.control_len = 16;
            chan_count = 8;
            byte_input.is_downlink = false;
            break;
        case FRAME_LEN_16:
            byte_input.control_len = 27;
            chan_count = 16;
            byte_input.is_downlink = false;
            break;
        case FRAME_LEN_24:
            byte_input.control_len = 38;
            chan_count = 24;
            byte_input.is_downlink = false;
            break;
        case FRAME_LEN_DOWNLINK:
            byte_input.control_len = 10;
            byte_input.is_downlink = true;
            break;
        default:
            // definately not FPort2, missing header byte
            return;
        }
    }

    if (byte_input.ofs == 1) {
        if (!byte_input.is_downlink && b != FRAME_TYPE_CHANNEL) {
            // not channel data
            byte_input.ofs = 0;
            return;
        }
    }

    byte_input.buf[byte_input.ofs++] = b;

    const FPort2_Frame *frame = (const FPort2_Frame *)&byte_input.buf[0];

    if (byte_input.control_len > 2 && byte_input.ofs == byte_input.control_len) {
        if (!byte_input.is_downlink) {
            log_data(AP_RCProtocol::FPORT2, timestamp_us, byte_input.buf, byte_input.ofs);
            if (check_checksum()) {
                decode_control(*frame);
            }
        } else {
            // downlink packet
            if (check_checksum()) {
                decode_downlink(*frame);
            }
        }
        goto reset;
    }
    if (byte_input.ofs >= sizeof(byte_input.buf)) {
        goto reset;
    }
    return;

reset:
    byte_input.ofs = 0;
}

// check checksum byte
bool AP_RCProtocol_FPort2::check_checksum(void)
{
    return crc_sum8_with_carry(&byte_input.buf[1], byte_input.control_len-1) == 0;
}

// support byte input
void AP_RCProtocol_FPort2::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}

#endif  // AP_RCPROTOCOL_FPORT2_ENABLED
