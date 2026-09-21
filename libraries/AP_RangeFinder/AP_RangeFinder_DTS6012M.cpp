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


#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_DTS6012M_ENABLED

#include "AP_RangeFinder_DTS6012M.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>

// DTS6012M protocol constants
#define DTS6012M_FRAME_HEADER       0xA5
#define DTS6012M_DEVICE_ID          0x03
#define DTS6012M_DEVICE_TYPE        0x20
#define DTS6012M_CMD_START_STREAM   0x01
#define DTS6012M_HEADER_LEN         7       // header(1) + devid(1) + devtype(1) + cmd(1) + reserved(1) + length(2)
#define DTS6012M_DATA_LEN           14      // measurement data length
#define DTS6012M_CRC_LEN            2
#define DTS6012M_FRAME_LEN          (DTS6012M_HEADER_LEN + DTS6012M_DATA_LEN + DTS6012M_CRC_LEN)  // 23 bytes
#define DTS6012M_DIST_MAX_MM        20000   // 20m max range
#define DTS6012M_DIST_INVALID       0xFFFF

// set to 0 to disable CRC verification if the sensor's CRC proves unreliable
#ifndef DTS6012M_CRC_ENABLE
#define DTS6012M_CRC_ENABLE         1
#endif

extern const AP_HAL::HAL& hal;

/*
  send the start stream command (0x01) to begin periodic measurement output
  Frame: A5 03 20 01 00 00 00 CRC16_H CRC16_L
*/
void AP_RangeFinder_DTS6012M::send_start_command()
{
    static const uint8_t cmd[] = {
        DTS6012M_FRAME_HEADER,  // 0xA5
        DTS6012M_DEVICE_ID,     // 0x03
        DTS6012M_DEVICE_TYPE,   // 0x20
        DTS6012M_CMD_START_STREAM,  // 0x01
        0x00,                   // reserved
        0x00, 0x00              // length = 0 (no data)
    };

    // calculate CRC over the command bytes
    const uint16_t crc = calc_crc_modbus(cmd, sizeof(cmd));

    uart->write(cmd, sizeof(cmd));
    // CRC is sent high byte first per protocol spec
    uart->write(uint8_t(crc >> 8));
    uart->write(uint8_t(crc & 0xFF));
}

/*
  search for frame header byte in buffer starting at offset,
  shifting remaining data to re-sync after line noise
*/
void AP_RangeFinder_DTS6012M::find_signature_in_buffer(uint8_t start)
{
    uint8_t *buf = (uint8_t *)&frame;
    for (uint8_t i = start; i < linebuf_len; i++) {
        if (buf[i] == DTS6012M_FRAME_HEADER) {
            memmove(&buf[0], &buf[i], linebuf_len - i);
            linebuf_len -= i;
            return;
        }
    }
    linebuf_len = 0;
}

/*
  read from the sensor and return distance in meters
  see frame struct in header for protocol layout
*/
bool AP_RangeFinder_DTS6012M::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // keep sending start command until we receive a valid reading,
    // allowing for slow-startup or late connection of the device
    if (!got_reading) {
        send_start_command();
    }

    // bulk read available bytes into buffer
    uint8_t *buf = (uint8_t *)&frame;
    const auto num_read = uart->read(&buf[linebuf_len], sizeof(frame) - linebuf_len);
    linebuf_len += num_read;

    if (linebuf_len == 0) {
        return false;
    }

    // ensure buffer starts with frame header
    if (frame.header != DTS6012M_FRAME_HEADER) {
        find_signature_in_buffer(1);
        return false;
    }

    // wait for a complete frame
    if (linebuf_len < DTS6012M_FRAME_LEN) {
        return false;
    }

    // validate header fields
    if (frame.device_id != DTS6012M_DEVICE_ID ||
        frame.device_type != DTS6012M_DEVICE_TYPE ||
        frame.cmd_echo != DTS6012M_CMD_START_STREAM) {
        find_signature_in_buffer(1);
        return false;
    }

    // verify length field
    if (be16toh(frame.data_len) != DTS6012M_DATA_LEN) {
        find_signature_in_buffer(1);
        return false;
    }

    // verify CRC-16 over header + data (all bytes except last 2)
#if DTS6012M_CRC_ENABLE
    const uint16_t crc_calc = calc_crc_modbus(buf, DTS6012M_FRAME_LEN - DTS6012M_CRC_LEN);
    if (crc_calc != be16toh(frame.crc)) {
        find_signature_in_buffer(1);
        return false;
    }
#endif

    // extract primary target distance (little-endian, in mm)
    const uint16_t dist_mm = le16toh(frame.primary_distance_mm);

    // capture primary target intensity for signal quality
    const int32_t intensity = int32_t(le16toh(frame.primary_intensity));
    // map intensity to 0-100%, clamping at 10000 as practical maximum
    _signal_quality_pct = int8_t(constrain_int32(intensity * 100 / 10000, 0, 100));

    // frame consumed, reset buffer and discard any stale data so the
    // next call reads the freshest frame from this high-rate sensor
    linebuf_len = 0;
    uart->discard_input();
    got_reading = true;

    if (dist_mm == DTS6012M_DIST_INVALID || dist_mm > DTS6012M_DIST_MAX_MM) {
        reading_m = max_distance() + 1.0f;
        return true;
    }

    reading_m = dist_mm * 0.001f;
    return true;
}

#endif  // AP_RANGEFINDER_DTS6012M_ENABLED
